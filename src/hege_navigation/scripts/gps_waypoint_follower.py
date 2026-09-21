#!/usr/bin/env python3
"""Drive a list of GPS waypoints.

    ros2 run hege_navigation gps_waypoint_follower.py --ros-args \
        -p waypoints_file:=/path/to/waypoints.yaml

The file is a list of latitude/longitude pairs, each optionally carrying the
heading to arrive on:

    waypoints:
      - {latitude: 52.46612, longitude: 12.95830}
      - {latitude: 52.46620, longitude: 12.95845, heading: 270}

Humble's nav2_waypoint_follower only offers FollowWaypoints, which takes poses
in the map frame; the FollowGPSWaypoints action arrived in a later release. So
each fix is converted first, through robot_localization's /fromLL service, which
is the same service navsat_transform uses internally and therefore agrees with
whatever datum the run happens to have. That matters here: the datum is taken
from the first GPS fix rather than surveyed, so map coordinates differ between
runs while the latitude and longitude in the file stay valid forever.

HEADINGS

`heading` is a COMPASS BEARING in degrees - 0 north, 90 east, 180 south,
270 west - to match the latitude and longitude it sits next to. Everything
downstream is ENU yaw in radians, where 0 is EAST and angles grow
anticlockwise, so the two are not the same number and must not be confused:
bearing 0 is yaw +90. bearing_to_yaw() below is the only place that conversion
happens.

Leave `heading` out and the waypoint is given the bearing of the leg INTO it -
the direction it is approached from, taken from the waypoint before it, or for
the first waypoint from where the machine is standing when the mission starts.
That is the heading the machine would naturally arrive on anyway, so the goal
checker's yaw test passes without constraining anything. It is how "I do not
care which way it faces here" gets said.

Saying nothing at all is NOT how that gets said, and this is the trap. An
untouched geometry_msgs/Quaternion is not "unspecified" - there is no such
value - it is yaw=0, "arrive facing east". The planner plans to a POSE, so it
takes that literally: on a route that turns north, it plans a loop so the
machine can come back round and cross the point heading east, and the excess
manoeuvring looks like a controller fault. Hence the derived default above.

Each waypoint is also drawn twice for the benefit of whoever is watching: as a
marker on /gps_waypoints for RViz, and as a tall thin post spawned into the
Gazebo world. The post is visual only, with no collision geometry, so the rover
drives straight through it.

The planned path is not drawn in Gazebo. It is replanned every second, and
keeping a line of that up to date would mean spawning and deleting hundreds of
entities per second. RViz draws /plan natively and is the right tool for it.
"""

import copy
import math
import sys
import time

import rclpy
import yaml
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from visualization_msgs.msg import Marker, MarkerArray


# A visual-only post. No collision element, so it cannot be driven into, and
# static so physics ignores it entirely. Tall and thin to stay visible from
# across the field without getting in the way.
POST_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="post">
        <pose>0 0 2.0 0 0 0</pose>
        <geometry><cylinder><radius>0.12</radius><length>4.0</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <emissive>{r} {g} {b} 1</emissive>
        </material>
      </visual>
      <visual name="base">
        <pose>0 0 0.05 0 0 0</pose>
        <geometry><cylinder><radius>0.8</radius><length>0.1</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class GpsWaypointFollower(Node):

    def __init__(self):
        super().__init__('gps_waypoint_follower')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('gazebo_posts', True)
        self.from_ll = self.create_client(FromLL, '/fromLL')
        self.reported_waypoint = -1
        # Transient local, so RViz gets the waypoints whenever it connects
        # rather than only if it happened to be listening at the moment they
        # were sent. They are published once and never change.
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.markers = self.create_publisher(MarkerArray, '/gps_waypoints', latched)
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete = self.create_client(DeleteEntity, '/delete_entity')
        self.follow = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def load(self, path):
        with open(path) as handle:
            doc = yaml.safe_load(handle)
        points = doc['waypoints']
        self.get_logger().info('Loaded %d waypoints from %s' % (len(points), path))
        return points

    def to_map(self, point):
        """One latitude/longitude pair into a map-frame pose."""
        request = FromLL.Request()
        request.ll_point = GeoPoint(latitude=float(point['latitude']),
                                    longitude=float(point['longitude']),
                                    altitude=float(point.get('altitude', 0.0)))
        future = self.from_ll.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            raise RuntimeError('/fromLL did not answer')
        p = future.result().map_point

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = p.x
        pose.pose.position.y = p.y
        # Altitude is dropped on purpose: both EKFs run in two_d_mode and the
        # costmaps are flat, so a z from the fix would only be noise.
        pose.pose.position.z = 0.0
        # Placeholder. set_headings() overwrites every one of these, and it has
        # to: left alone this asks for an eastward arrival at every waypoint.
        pose.pose.orientation.w = 1.0
        return pose

    # --------------------------------------------------------------- headings
    @staticmethod
    def bearing_to_yaw(bearing_deg):
        """Compass bearing in degrees -> ENU yaw in radians.

        Bearing counts clockwise from north, yaw anticlockwise from east, so
        the two run in opposite directions and are offset by a quarter turn:
        yaw = 90 - bearing. Normalised through atan2 to [-pi, pi], which is
        the range atan2 produces for the derived headings too, so a heading
        from the file and a heading from the route are directly comparable.
        """
        return math.atan2(math.sin(math.radians(90.0 - bearing_deg)),
                          math.cos(math.radians(90.0 - bearing_deg)))

    @staticmethod
    def yaw_to_bearing(yaw):
        """ENU yaw in radians -> compass bearing in degrees, [0, 360).

        For display only - nothing steers by this. Rounded to a tenth of a
        degree before the wrap, and that order matters: a waypoint a hair west
        of north is 359.9994, which every sensible print format renders as
        "360". Rounding first turns it into 360.0, and the second wrap turns
        that into the 0 it should have read as all along.
        """
        bearing = math.degrees(math.atan2(math.cos(yaw), math.sin(yaw)))
        return round(bearing % 360.0, 1) % 360.0

    @staticmethod
    def set_yaw(pose, yaw):
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

    def start_pose(self):
        """Where the machine is standing now, or None if nothing answers.

        Only the first waypoint needs it, and only when that waypoint has no
        heading of its own: every later one is approached from the waypoint
        before it, but the first is approached from wherever the run begins.
        """
        got = []
        sub = self.create_subscription(
            Odometry, '/odometry/filtered_map', got.append, 1)
        try:
            deadline = time.time() + 5.0
            while not got and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.2)
        finally:
            self.destroy_subscription(sub)
        return got[0].pose.pose.position if got else None

    def set_headings(self, points, poses, start=None):
        """Give every waypoint its arrival heading, in place.

        A waypoint with a `heading` in the file gets exactly that, converted
        from compass bearing to ENU yaw. One without gets the bearing of the
        leg into it, which is the heading it would arrive on anyway.

        Returns a list of 'file' / 'route' / 'default' saying where each
        heading came from, so the log can show it rather than leaving the
        reader to guess which waypoints are actually constrained.
        """
        sources = []
        for i, (point, pose) in enumerate(zip(points, poses)):
            given = point.get('heading')
            if given is not None:
                self.set_yaw(pose, self.bearing_to_yaw(float(given)))
                sources.append('file')
                continue

            previous = start if i == 0 else poses[i - 1].pose.position
            if previous is None:
                # First waypoint, no heading in the file, and no odometry to
                # take an approach direction from. Nothing here is a good
                # answer; yaw=0 (east) at least matches what the file would
                # have produced before headings existed.
                self.get_logger().warn(
                    'waypoint 1 has no heading and /odometry/filtered_map did '
                    'not answer, so it is left facing east - give it a '
                    'heading in the file to be sure of it')
                sources.append('default')
                continue

            dx = pose.pose.position.x - previous.x
            dy = pose.pose.position.y - previous.y
            if math.hypot(dx, dy) < 1e-3:
                # Nothing to take a bearing from: atan2(0, 0) is 0, which would
                # quietly mean east. Keep whatever the point before settled on.
                self.get_logger().warn(
                    'waypoint %d sits on top of what comes before it; heading '
                    'kept from the previous one' % (i + 1))
                if i > 0:
                    pose.pose.orientation = poses[i - 1].pose.orientation
                sources.append(sources[i - 1] if i > 0 else 'default')
                continue

            self.set_yaw(pose, math.atan2(dy, dx))
            sources.append('route')
        return sources

    # ---------------------------------------------------------------- drawing
    def draw_rviz(self, poses):
        """Publish the waypoints as RViz markers: a sphere and a number each.

        Both markers take a COPY of the waypoint pose. Assigning a message
        field in rclpy stores the reference, so sharing one Pose between the
        sphere, the label and the goal means the last z written wins for all
        three: the number ends up buried in the ball, and the poses handed to
        FollowWaypoints below carry a marker's drawing height instead of the
        zero to_map() deliberately put there.
        """
        array = MarkerArray()
        for i, pose in enumerate(poses):
            ball = Marker()
            ball.header.frame_id = 'map'
            ball.header.stamp = self.get_clock().now().to_msg()
            ball.ns = 'gps_waypoints'
            ball.id = i
            ball.type = Marker.SPHERE
            ball.action = Marker.ADD
            ball.pose = copy.deepcopy(pose.pose)
            ball.pose.position.z = 1.0
            ball.scale.x = ball.scale.y = ball.scale.z = 1.5
            ball.color.r, ball.color.g, ball.color.b, ball.color.a = 1.0, 0.3, 0.0, 0.9
            array.markers.append(ball)

            label = Marker()
            label.header = ball.header
            label.ns = 'gps_waypoint_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = copy.deepcopy(pose.pose)
            label.pose.position.z = 3.0
            label.scale.z = 2.0
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(i + 1)
            array.markers.append(label)
        self.markers.publish(array)

    def draw_gazebo(self, poses):
        """Spawn a post at each waypoint so they are visible in Gazebo itself.

        Map and world coordinates line up here because the rover spawns at the
        world origin and the datum is taken from its first fix there, so the two
        frames differ by no more than the GPS noise on that one reading. That is
        centimetres, which does not matter for something being used as a visual
        marker.
        """
        if not self.spawn.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('No /spawn_entity; skipping the Gazebo posts')
            return

        for i, pose in enumerate(poses):
            name = 'waypoint_%d' % (i + 1)

            # Clear a post left behind by an earlier run. Failure is expected
            # on a fresh world and is not worth reporting.
            if self.delete.wait_for_service(timeout_sec=2.0):
                req = DeleteEntity.Request()
                req.name = name
                fut = self.delete.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

            # First waypoint green, last red, the rest orange - so the order of
            # the mission is readable at a glance.
            if i == 0:
                colour = dict(r=0.1, g=0.9, b=0.1)
            elif i == len(poses) - 1:
                colour = dict(r=0.9, g=0.1, b=0.1)
            else:
                colour = dict(r=1.0, g=0.55, b=0.0)

            req = SpawnEntity.Request()
            req.name = name
            req.xml = POST_SDF.format(name=name, **colour)
            req.initial_pose.position.x = pose.pose.position.x
            req.initial_pose.position.y = pose.pose.position.y
            req.initial_pose.orientation.w = 1.0
            fut = self.spawn.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            if fut.result() is None or not fut.result().success:
                self.get_logger().warn('Could not spawn %s' % name)
        self.get_logger().info('Posted %d waypoint markers in Gazebo' % len(poses))

    def run(self, path):
        points = self.load(path)

        if not self.from_ll.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(
                '/fromLL is not up. navsat_transform has to be running, and it '
                'only offers the service once it has a datum.')
            return 1

        poses = [self.to_map(point) for point in points]

        # The start pose is only fetched when something actually needs it: a
        # first waypoint with no heading of its own. Asking for it otherwise
        # would spend five seconds waiting on a topic nobody is reading.
        start = None
        if points and points[0].get('heading') is None:
            start = self.start_pose()
        sources = self.set_headings(points, poses, start=start)

        for i, (point, pose, source) in enumerate(zip(points, poses, sources)):
            yaw = 2.0 * math.atan2(pose.pose.orientation.z,
                                   pose.pose.orientation.w)
            self.get_logger().info(
                '  %d: %.7f, %.7f  ->  map (%.2f, %.2f)  arrive on bearing '
                '%3.0f deg (%s)'
                % (i + 1, point['latitude'], point['longitude'],
                   pose.pose.position.x, pose.pose.position.y,
                   self.yaw_to_bearing(yaw), source))

        self.draw_rviz(poses)
        if self.get_parameter('gazebo_posts').value:
            self.draw_gazebo(poses)

        if not self.follow.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('follow_waypoints action server is not up')
            return 1

        goal = FollowWaypoints.Goal()
        goal.poses = poses
        send = self.follow.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('The waypoint goal was rejected')
            return 1

        self.get_logger().info('Following %d waypoints' % len(poses))
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)

        missed = list(result.result().result.missed_waypoints)
        if missed:
            self.get_logger().warn('Missed waypoints: %s' % missed)
        else:
            self.get_logger().info('Every waypoint reached')
        return 0

    def on_feedback(self, msg):
        # Feedback arrives at the action server's rate, which over a four-point
        # mission is a few thousand messages. Only the changes are worth saying.
        current = msg.feedback.current_waypoint
        if current != self.reported_waypoint:
            self.reported_waypoint = current
            self.get_logger().info('now heading for waypoint %d' % (current + 1))


def main():
    rclpy.init()
    node = GpsWaypointFollower()
    path = node.get_parameter('waypoints_file').value
    if not path:
        node.get_logger().error('Set waypoints_file, e.g. -p waypoints_file:=waypoints.yaml')
        code = 1
    else:
        try:
            code = node.run(path)
        except (KeyboardInterrupt, RuntimeError) as exc:
            node.get_logger().error(str(exc))
            code = 1
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
