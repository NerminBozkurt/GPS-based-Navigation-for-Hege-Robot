#!/usr/bin/env python3
"""Live side-by-side comparison of every position estimate against the truth.

Run it next to the simulation and teleop:

    ros2 launch hege_description spawn_hege.launch.py
    ros2 launch hege_localization localization.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -p speed:=1.5 -p turn:=0.3
    ros2 run hege_localization localization_monitor.py

Ground truth comes from the simulator itself, not from GPS, so GPS noise shows
up as an error like every other estimate rather than being taken for the right
answer. Which topic carries it depends on which Gazebo is running: Harmonic
publishes /hege/ground_truth/odom through the OdometryPublisher system in the
URDF's gz branch, and Classic publishes /model_states through the
gazebo_ros_state world plugin. The monitor listens for both and uses whichever
arrives.

All frames line up because the world's spherical_coordinates origin, the EKF
datum and the spawn point are the same place.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix

# Gazebo Classic only, and gazebo_msgs is not installable next to Harmonic on
# every machine. The Harmonic path uses /hege/ground_truth/odom instead, so a
# missing gazebo_msgs must not stop the monitor from running.
try:
    from gazebo_msgs.msg import ModelStates
except ImportError:
    ModelStates = None

MODEL_NAME = 'hege'

DATUM_LAT = 52.466
DATUM_LON = 12.958
LAT_M = 111132.0
LON_M = 111320.0 * math.cos(math.radians(DATUM_LAT))

ESC = '\033['
DIM, BOLD, RESET = ESC + '2m', ESC + '1m', ESC + '0m'
RED, YELLOW, GREEN, CYAN = (ESC + '31m', ESC + '33m', ESC + '32m', ESC + '36m')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def err_colour(e):
    if e is None:
        return DIM
    if e < 0.25:
        return GREEN
    if e < 1.5:
        return YELLOW
    return RED


class Monitor(Node):

    def __init__(self):
        super().__init__('localization_monitor')
        self.truth = None
        self.glob = None
        self.local = None
        self.wheel = None
        self.gps = None
        self.imu = None
        self.cmd = Twist()
        self.peak = {}

        # Two ground-truth sources, one per simulator, and whichever is
        # actually publishing wins. Harmonic has no /model_states: the pose
        # comes from the OdometryPublisher system in the URDF's gz branch,
        # bridged by spawn_hege.launch.py. Both carry the simulator's exact
        # pose, so the numbers stay comparable across the two.
        self.create_subscription(Odometry, '/hege/ground_truth/odom',
                                 lambda m: setattr(self, 'truth', m.pose.pose), 10)
        if ModelStates is not None:
            self.create_subscription(ModelStates, '/model_states',
                                     self.on_model_states, 10)
        self.create_subscription(Odometry, '/odometry/filtered_map',
                                 lambda m: setattr(self, 'glob', m), 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: setattr(self, 'local', m), 10)
        self.create_subscription(Odometry, '/ackermann_steering_controller/odometry',
                                 lambda m: setattr(self, 'wheel', m), 10)
        self.create_subscription(NavSatFix, '/gps/fix',
                                 lambda m: setattr(self, 'gps', m), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data',
                                 lambda m: setattr(self, 'imu', m), qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel',
                                 lambda m: setattr(self, 'cmd', m), 10)

        self.create_timer(0.2, self.draw)
        print(ESC + '2J', end='')          # clear once; afterwards we redraw in place

    def on_model_states(self, msg):
        try:
            self.truth = msg.pose[msg.name.index(MODEL_NAME)]
        except ValueError:
            pass

    # ------------------------------------------------------------------ rows
    def rows(self):
        t = self.truth
        tx = t.position.x if t else None
        ty = t.position.y if t else None
        tyaw = yaw_of(t.orientation) if t else None

        out = [('Ground truth (Gazebo)', tx, ty, tyaw, None)]

        def add(label, odom):
            if odom is None:
                out.append((label, None, None, None, None))
                return
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            yw = yaw_of(odom.pose.pose.orientation)
            e = math.hypot(x - tx, y - ty) if tx is not None else None
            out.append((label, x, y, yw, e))

        add('Global EKF  (map)', self.glob)
        add('Local EKF   (odom)', self.local)
        add('Wheel odometry', self.wheel)

        if self.gps is not None:
            gx = (self.gps.longitude - DATUM_LON) * LON_M
            gy = (self.gps.latitude - DATUM_LAT) * LAT_M
            e = math.hypot(gx - tx, gy - ty) if tx is not None else None
            out.append(('GPS raw fix', gx, gy, None, e))
        else:
            out.append(('GPS raw fix', None, None, None, None))
        return out

    # ------------------------------------------------------------------ draw
    def draw(self):
        rows = self.rows()
        w = 78
        L = [ESC + 'H']          # cursor home, then overwrite every line

        speed = self.cmd.linear.x
        turn = self.cmd.angular.z
        moving = abs(speed) > 1e-3 or abs(turn) > 1e-3
        drive = ('%s%+.2f m/s   %+.2f rad/s%s' % (CYAN, speed, turn, RESET)
                 if moving else DIM + 'stopped' + RESET)

        imu_yaw = yaw_of(self.imu.orientation) if self.imu else None

        L.append(BOLD + 'HEGE LOCALIZATION MONITOR'.ljust(40) + RESET + 'cmd: ' + drive)
        L.append(DIM + '-' * w + RESET)
        L.append(BOLD + '%-22s %10s %10s %9s %12s' % (
            'SOURCE', 'X east', 'Y north', 'YAW deg', 'ERROR') + RESET)
        L.append(DIM + '-' * w + RESET)

        for label, x, y, yw, e in rows:
            if x is None:
                L.append('%-22s %s' % (label, DIM + 'waiting for data...' + RESET))
                continue
            peak = self.peak.get(label)
            if e is not None and (peak is None or e > peak):
                self.peak[label] = e
                peak = e
            yaw_s = '%+9.1f' % math.degrees(yw) if yw is not None else '        -'
            if e is None:
                err_s = DIM + '        (ref)' + RESET
            else:
                err_s = '%s%9.3f m%s' % (err_colour(e), e, RESET)
            L.append('%-22s %10.3f %10.3f %s %s' % (label, x, y, yaw_s, err_s))

        L.append(DIM + '-' * w + RESET)
        L.append(DIM + 'worst error so far:' + RESET)
        for label, *_ in rows[1:]:
            peak = self.peak.get(label)
            if peak is not None:
                L.append('   %-22s %s%7.3f m%s' % (label, err_colour(peak), peak, RESET))
        if imu_yaw is not None:
            L.append(DIM + ('   IMU heading %+.1f deg  (0 = east, 90 = north)'
                            % math.degrees(imu_yaw)) + RESET)
        L.append('')
        L.append(DIM + 'green < 0.25 m   yellow < 1.5 m   red above' + RESET)
        L.append(DIM + 'Ctrl-C to quit' + RESET)

        # pad so shorter frames do not leave stale text behind
        print('\n'.join(s + ESC + 'K' for s in L) + ESC + 'J', end='', flush=True)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print(RESET)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
