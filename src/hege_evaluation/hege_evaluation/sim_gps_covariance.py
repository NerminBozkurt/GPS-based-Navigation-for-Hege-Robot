"""Add known simulation covariance to Gazebo NavSatFix messages.

Gazebo publishes the fix with position_covariance all zeros and
COVARIANCE_TYPE_UNKNOWN. navsat_transform carries that into /odometry/gps and
the global EKF uses it to decide how far to move towards each fix, so leaving
it empty makes the filter guess. We know the answer, because the noise was
configured in hege.urdf.xacro.

The defaults here track gps_noise in that file; spawn_hege_gz.launch.py passes
them explicitly so the two cannot drift apart silently.

From oguzissik/hege_gps_navigation.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class SimGpsCovariance(Node):
    """Republish simulated GPS with covariance consistent with Gazebo noise."""

    def __init__(self) -> None:
        super().__init__("hege_sim_gps_covariance")

        self.declare_parameter("input_topic", "/gps/fix_raw")
        self.declare_parameter("output_topic", "/gps/fix")
        self.declare_parameter("horizontal_stddev_m", 0.02)
        self.declare_parameter("vertical_stddev_m", 0.04)
        self.declare_parameter("origin_latitude_deg", 52.466)

        parameter = self.get_parameter

        input_topic = str(parameter("input_topic").value)
        output_topic = str(parameter("output_topic").value)
        sigma_horizontal = float(parameter("horizontal_stddev_m").value)
        sigma_up = float(parameter("vertical_stddev_m").value)
        latitude_deg = float(parameter("origin_latitude_deg").value)

        if sigma_horizontal < 0.0 or sigma_up < 0.0:
            raise ValueError("GPS standard deviations must be non-negative")

        # Gazebo applies the same angular noise to latitude and longitude.
        # One longitude degree represents cos(latitude) fewer metres than
        # one latitude degree.
        self.sigma_north = sigma_horizontal
        self.sigma_east = (
            sigma_horizontal * math.cos(math.radians(latitude_deg))
        )
        self.sigma_up = sigma_up

        self.publisher = self.create_publisher(NavSatFix, output_topic, 10)
        self.subscription = self.create_subscription(
            NavSatFix,
            input_topic,
            self.on_fix,
            10,
        )

        self.get_logger().info(
            f"republishing {input_topic} -> {output_topic}; "
            f"sigma_e={self.sigma_east:.3f} m, "
            f"sigma_n={self.sigma_north:.3f} m, "
            f"sigma_u={self.sigma_up:.3f} m"
        )

    def on_fix(self, msg: NavSatFix) -> None:
        msg.position_covariance = [
            self.sigma_east ** 2, 0.0, 0.0,
            0.0, self.sigma_north ** 2, 0.0,
            0.0, 0.0, self.sigma_up ** 2,
        ]
        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )

        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimGpsCovariance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()