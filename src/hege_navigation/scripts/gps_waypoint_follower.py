#!/usr/bin/env python3
"""Drive a list of GPS waypoints.

    ros2 run hege_navigation gps_waypoint_follower.py --ros-args \
        -p waypoints_file:=/path/to/waypoints.yaml

The file is a list of latitude/longitude pairs:

    waypoints:
      - {latitude: 52.46612, longitude: 12.95830}
      - {latitude: 52.46620, longitude: 12.95845}

Humble's nav2_waypoint_follower only offers FollowWaypoints, which takes poses
in the map frame; the FollowGPSWaypoints action arrived in a later release. So
each fix is converted first, through robot_localization's /fromLL service, which
is the same service navsat_transform uses internally and therefore agrees with
whatever datum the run happens to have. That matters here: the datum is taken
from the first GPS fix rather than surveyed, so map coordinates differ between
runs while the latitude and longitude in the file stay valid forever.

Headings are not specified. The rover is car-like and arrives facing whatever
direction the path brought it in on, which is why the goal checker's yaw
tolerance is deliberately loose.
"""

import math
import sys

import rclpy
import yaml
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_localization.srv import FromLL


class GpsWaypointFollower(Node):

    def __init__(self):
        super().__init__('gps_waypoint_follower')
        self.declare_parameter('waypoints_file', '')
        self.from_ll = self.create_client(FromLL, '/fromLL')
        self.reported_waypoint = -1
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
        pose.pose.orientation.w = 1.0
        return pose

    def run(self, path):
        points = self.load(path)

        if not self.from_ll.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(
                '/fromLL is not up. navsat_transform has to be running, and it '
                'only offers the service once it has a datum.')
            return 1

        poses = []
        for i, point in enumerate(points):
            pose = self.to_map(point)
            poses.append(pose)
            self.get_logger().info(
                '  %d: %.7f, %.7f  ->  map (%.2f, %.2f)'
                % (i + 1, point['latitude'], point['longitude'],
                   pose.pose.position.x, pose.pose.position.y))

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
