"""
dry_run  --  show what the wheels WOULD be told to do, without telling them.

    ros2 run hege_px4_bridge dry_run --ros-args \
      -p bridge_params:=<path to bridge_real.yaml>

This node is the bridge with its output disconnected. It subscribes to the same
velocity topic, applies the same ackermann.limit_command() with the same limits
read from the same YAML file, and then publishes the answer for a human to read
instead of sending it to PX4. It never publishes to /fmu/in/*, never arms
anything and holds no PX4 connection at all, so running it cannot move the
vehicle no matter what state PX4 is in.

What it is for: the first time Nav2 plans for the real rover, you want to see
the velocities and the steering angle a 2D Goal Pose produces before any of it
reaches an actuator. Give the goal in RViz, watch these numbers, and only then
arm.

Published:

    /hege/dry_run/cmd             geometry_msgs/TwistStamped  (v, omega) after limiting
    /hege/dry_run/steering_angle  std_msgs/Float64            delta [rad], + is left
    /hege/dry_run/report          std_msgs/String             the same as one line of text

The steering angle is the number worth watching. (v, omega) is what Nav2 asked
for; delta = atan(L * omega / v) is what the front wheels have to do to deliver
it, and it is the one that has a mechanical limit you can see on the vehicle.

Note what this node does NOT tell you: it reproduces the bridge's limiting, not
PX4's. The bridge hands PX4 a heading setpoint, and PX4's own controller,
slew-rate limits and the tyres decide the steering angle that actually results.
delta here is the kinematic request, not a prediction of the actuator.
"""

from __future__ import annotations

import math

import rclpy
import yaml
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64, String

from hege_px4_bridge import ackermann

# Keys taken from the bridge YAML when bridge_params is given. Anything else in
# that file is the bridge's business (PX4 topics, timeouts, arming policy) and
# is deliberately ignored here.
_LIMIT_KEYS = ("wheel_base", "max_steering_angle", "max_speed",
               "max_yaw_rate", "min_moving_speed")


def load_bridge_params(path: str) -> dict:
    """
    Read the limits out of a hege_px4_bridge parameter file.

    The file is read directly rather than through --params-file because ROS 2
    matches parameter files by node name, and bridge_real.yaml is keyed under
    `hege_px4_bridge`. Loading it the normal way would mean naming this node
    `hege_px4_bridge` too, which collides with the real bridge when both run -
    and on hardware both do run, because the bridge is what reports
    /hege/bridge/status. Reading the file is the honest way to guarantee that
    the numbers shown here are the numbers the bridge will use.
    """
    with open(path, 'r') as handle:
        document = yaml.safe_load(handle)
    for section in document.values():
        if isinstance(section, dict) and 'ros__parameters' in section:
            return section['ros__parameters']
    raise ValueError(f"{path} contains no ros__parameters block")


class DryRunNode(Node):

    def __init__(self) -> None:
        super().__init__("hege_dry_run")

        # Defaults are bridge_real.yaml's, so the node is useful with no
        # arguments at all; bridge_params overrides them from the real file.
        self.declare_parameter("bridge_params", "")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel/selected")
        self.declare_parameter("wheel_base", 1.90)
        self.declare_parameter("max_steering_angle", 0.610865)
        self.declare_parameter("max_speed", 0.3)
        self.declare_parameter("max_yaw_rate", 0.10)
        self.declare_parameter("min_moving_speed", 0.05)
        self.declare_parameter("report_rate_hz", 5.0)

        values = {name: self.get_parameter(name).value for name in _LIMIT_KEYS}
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        path = self.get_parameter("bridge_params").value
        if path:
            loaded = load_bridge_params(path)
            for name in _LIMIT_KEYS:
                if name in loaded:
                    values[name] = float(loaded[name])
            # Only if the caller left the topic at its default: an explicit
            # -p cmd_vel_topic:= on the command line should still win.
            if (self.get_parameter("cmd_vel_topic").value == "/cmd_vel/selected"
                    and "cmd_vel_topic" in loaded):
                cmd_vel_topic = loaded["cmd_vel_topic"]
            self.get_logger().info(f"limits from {path}")

        self.limits = ackermann.AckermannLimits(**values)
        self.limits.validate()

        self.last_cmd: Twist | None = None
        self.last_cmd_time: float = 0.0

        self.cmd_pub = self.create_publisher(TwistStamped, "/hege/dry_run/cmd", 10)
        self.steer_pub = self.create_publisher(Float64, "/hege/dry_run/steering_angle", 10)
        self.report_pub = self.create_publisher(String, "/hege/dry_run/report", 10)
        self.create_subscription(Twist, cmd_vel_topic, self.on_cmd, 10)

        period = 1.0 / float(self.get_parameter("report_rate_hz").value)
        self.create_timer(period, self.tick)

        self.get_logger().warn(
            "DRY RUN: reading %s, publishing nothing to PX4. Nothing here can "
            "move the vehicle." % cmd_vel_topic)
        self.get_logger().info(
            "L=%.2f m  delta_max=%.1f deg  v_max=%.2f m/s  omega_max=%.2f rad/s  "
            "v_min=%.2f m/s" % (
                self.limits.wheel_base,
                math.degrees(self.limits.max_steering_angle),
                self.limits.max_speed, self.limits.max_yaw_rate,
                self.limits.min_moving_speed))

    def on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last_cmd is None:
            self.publish_report("no command yet on the velocity topic")
            return

        # The same 0.5 s the bridge treats as a command timeout. Past it the
        # bridge stops the vehicle, so reporting the last value would be a lie.
        age = now - self.last_cmd_time
        if age > 0.5:
            self.publish_report(f"stale: last command {age:.1f} s ago -> bridge would stop")
            return

        v_cmd = self.last_cmd.linear.x
        omega_cmd = self.last_cmd.angular.z
        result = ackermann.limit_command(v_cmd, omega_cmd, self.limits)

        delta = ackermann.steering_angle(result.v, result.omega, self.limits.wheel_base)

        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "base_link"
        stamped.twist.linear.x = result.v
        stamped.twist.angular.z = result.omega
        self.cmd_pub.publish(stamped)
        self.steer_pub.publish(Float64(data=delta))

        if result.stopped:
            self.publish_report(
                f"asked v={v_cmd:+.2f} omega={omega_cmd:+.3f} -> STOP ({result.reason})")
            return

        # Radius is what a turn looks like on the ground, and at these speeds it
        # is the number that decides whether a goal is reachable at all.
        if abs(delta) < 1e-6:
            radius = "straight"
        else:
            radius = f"R={self.limits.wheel_base / math.tan(abs(delta)):.1f} m"

        note = "" if result.reason == "ok" else f"  [{result.reason}]"
        self.publish_report(
            f"v={result.v:.2f} m/s  omega={result.omega:+.3f} rad/s  "
            f"steer={math.degrees(delta):+.1f} deg  {radius}"
            f"  (asked {v_cmd:+.2f}, {omega_cmd:+.3f}){note}")

    def publish_report(self, text: str) -> None:
        self.report_pub.publish(String(data=text))
        self.get_logger().info(text, throttle_duration_sec=0.5)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DryRunNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
