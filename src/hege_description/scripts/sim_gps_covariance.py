#!/usr/bin/env python3
"""Stamp the simulated GPS fix with the covariance it actually has.

Gazebo's NavSat sensor publishes a fix and leaves `position_covariance` at
zero with `position_covariance_type = COVARIANCE_TYPE_UNKNOWN`. Downstream that
is not harmless: navsat_transform carries the covariance into /odometry/gps,
and the global EKF uses it to decide how far to move towards each fix. Given
nothing, it has to guess.

We do know the answer, because we configured the noise ourselves in
hege.urdf.xacro. This node republishes the fix with that number written in.

    /gps/fix_raw  ->  /gps/fix

The one subtlety is that the two horizontal axes do NOT get the same metric
error. Gazebo applies the same ANGULAR noise to latitude and longitude, and one
degree of longitude is cos(latitude) fewer metres than one of latitude. At
52.466 deg that makes the east error about 0.61 of the north one, so the
covariance published here is deliberately anisotropic.

Adapted from sim_gps_covariance.py in oguzissik/hege_gps_navigation, where
the whole units question was found.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class SimGpsCovariance(Node):

    def __init__(self):
        super().__init__('sim_gps_covariance')

        self.declare_parameter('input_topic', '/gps/fix_raw')
        self.declare_parameter('output_topic', '/gps/fix')
        # Metres, and it has to match gps_noise in hege.urdf.xacro. The launch
        # file passes it so the two cannot drift apart silently.
        self.declare_parameter('horizontal_stddev_m', 0.02)
        self.declare_parameter('vertical_stddev_m', 0.04)
        # Only used for the cos(latitude) term, so the exact value matters
        # little; it should be the world's spherical_coordinates origin.
        self.declare_parameter('origin_latitude_deg', 52.466)

        p = self.get_parameter
        sigma_h = float(p('horizontal_stddev_m').value)
        sigma_v = float(p('vertical_stddev_m').value)
        latitude = float(p('origin_latitude_deg').value)

        if sigma_h < 0.0 or sigma_v < 0.0:
            raise ValueError('GPS standard deviations must be non-negative')

        self.sigma_north = sigma_h
        self.sigma_east = sigma_h * math.cos(math.radians(latitude))
        self.sigma_up = sigma_v

        self.pub = self.create_publisher(NavSatFix, p('output_topic').value, 10)
        self.create_subscription(NavSatFix, p('input_topic').value,
                                 self.on_fix, 10)

        self.get_logger().info(
            'republishing %s -> %s with sigma_e=%.3f m, sigma_n=%.3f m, '
            'sigma_u=%.3f m' % (p('input_topic').value, p('output_topic').value,
                                self.sigma_east, self.sigma_north, self.sigma_up))

    def on_fix(self, msg):
        # Row-major [E, N, U] on the diagonal, which is what NavSatFix means by
        # position_covariance regardless of the frame the fix is stamped in.
        msg.position_covariance = [
            self.sigma_east ** 2, 0.0, 0.0,
            0.0, self.sigma_north ** 2, 0.0,
            0.0, 0.0, self.sigma_up ** 2,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimGpsCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
