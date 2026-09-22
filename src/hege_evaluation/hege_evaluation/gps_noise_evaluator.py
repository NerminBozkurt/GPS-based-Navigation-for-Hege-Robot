"""Compare simulated NavSat fixes with noise-free Gazebo ground truth."""

from __future__ import annotations

from collections import deque
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from hege_evaluation.geodesy import geodetic_to_enu


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class GpsNoiseEvaluator(Node):
    """Timestamp-match GPS and truth, then publish running error statistics."""

    def __init__(self) -> None:
        super().__init__("hege_gps_noise_evaluator")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("truth_topic", "/hege/ground_truth/odom")
        self.declare_parameter("report_topic", "/hege/evaluation/gps_noise")
        self.declare_parameter("origin_latitude_deg", 52.466)
        self.declare_parameter("origin_longitude_deg", 12.958)
        self.declare_parameter("origin_altitude_m", 35.0)
        self.declare_parameter("max_time_offset_s", 0.15)
        self.declare_parameter("report_every_samples", 25)

        p = self.get_parameter
        self.origin = (
            float(p("origin_latitude_deg").value),
            float(p("origin_longitude_deg").value),
            float(p("origin_altitude_m").value),
        )
        self.max_time_offset = float(p("max_time_offset_s").value)
        self.report_every = int(p("report_every_samples").value)
        if self.max_time_offset <= 0.0 or self.report_every <= 0:
            raise ValueError("max_time_offset_s and report_every_samples must be > 0")

        self.truth = deque(maxlen=500)
        self.count = 0
        self.sum_e = 0.0
        self.sum_n = 0.0
        self.sum_e2 = 0.0
        self.sum_n2 = 0.0
        self.sum_h2 = 0.0

        self.pub_report = self.create_publisher(String, p("report_topic").value, 10)
        self.create_subscription(Odometry, p("truth_topic").value, self.on_truth, 50)
        self.create_subscription(NavSatFix, p("gps_topic").value, self.on_gps, 10)
        self.get_logger().info(
            f"evaluating {p('gps_topic').value} against {p('truth_topic').value}")

    def on_truth(self, msg: Odometry) -> None:
        self.truth.append((stamp_seconds(msg.header.stamp),
                           float(msg.pose.pose.position.x),
                           float(msg.pose.pose.position.y)))

    def on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status == NavSatStatus.STATUS_NO_FIX or not self.truth:
            return
        values = (msg.latitude, msg.longitude, msg.altitude)
        if not all(math.isfinite(value) for value in values):
            return

        gps_time = stamp_seconds(msg.header.stamp)
        truth_time, truth_e, truth_n = min(
            self.truth, key=lambda sample: abs(sample[0] - gps_time))
        time_offset = abs(truth_time - gps_time)
        if time_offset > self.max_time_offset:
            return

        gps_e, gps_n, _ = geodetic_to_enu(*values, *self.origin)
        error_e = gps_e - truth_e
        error_n = gps_n - truth_n
        self.count += 1
        self.sum_e += error_e
        self.sum_n += error_n
        self.sum_e2 += error_e * error_e
        self.sum_n2 += error_n * error_n
        self.sum_h2 += error_e * error_e + error_n * error_n

        if self.count % self.report_every == 0:
            report = {
                "samples": self.count,
                "mean_e_m": self.sum_e / self.count,
                "mean_n_m": self.sum_n / self.count,
                "rmse_e_m": math.sqrt(self.sum_e2 / self.count),
                "rmse_n_m": math.sqrt(self.sum_n2 / self.count),
                "rmse_horizontal_m": math.sqrt(self.sum_h2 / self.count),
                "last_time_offset_s": time_offset,
            }
            message = String(data=json.dumps(report, sort_keys=True))
            self.pub_report.publish(message)
            self.get_logger().info(message.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsNoiseEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
