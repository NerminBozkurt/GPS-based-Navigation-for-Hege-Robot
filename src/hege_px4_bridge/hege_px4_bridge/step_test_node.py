"""
step_test  --  publishes a fixed, slow /cmd_vel sequence for the first bridge tests
(development-order step 4: slow forward, small steering left, small steering right, stop).

The sequence is a parameter `phases` with entries "duration_s,v_mps,omega_radps".
The default is for a secured real rover with its driven wheels lifted:

    3 s   stop
    5 s   v = 0.30 m/s, omega =  0.00      straight
    5 s   v = 0.30 m/s, omega = +0.05      gentle left  (ROS: +omega = CCW = left)
    5 s   v = 0.30 m/s, omega = -0.05      gentle right
    3 s   stop

With the measured real-rover L = 1.91 m, the steering angle is
    delta = atan(L * omega / v) = atan(1.91 * 0.05 / 0.30) = 0.308 rad = 17.7 deg
which is below the reported delta_max of about 35 deg. The node exits when the
sequence ends. sim.launch.py overrides the turn phases to +/-0.20 rad/s because
the much smaller SITL model would only steer about 3 degrees at +/-0.05 rad/s.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StepTestNode(Node):

    def __init__(self) -> None:
        super().__init__("hege_step_test")
        self.declare_parameter("confirm_motion_test", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel/test")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("phases", [
            "3.0,0.0,0.0",
            "5.0,0.3,0.0",
            "5.0,0.3,0.05",
            "5.0,0.3,-0.05",
            "3.0,0.0,0.0",
        ])

        self.finished = False
        if not self.get_parameter("confirm_motion_test").value:
            self.get_logger().error(
                "motion test disabled: pass --ros-args -p confirm_motion_test:=true only in SITL "
                "or with the real rover secured and wheels lifted")
            self.finished = True
            return

        self.phases: list[tuple[float, float, float]] = []
        for text in self.get_parameter("phases").value:
            d, v, w = (float(x) for x in text.split(","))
            self.phases.append((d, v, w))

        self.pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.period = 1.0 / self.get_parameter("publish_rate_hz").value
        self.phase_index = 0
        self.phase_elapsed = 0.0
        self.create_timer(self.period, self.tick)
        self.announce()

    def announce(self) -> None:
        d, v, w = self.phases[self.phase_index]
        self.get_logger().info(f"phase {self.phase_index + 1}/{len(self.phases)}: "
                               f"{d:.1f} s  v={v:.2f} m/s  omega={w:+.2f} rad/s")

    def tick(self) -> None:
        if self.phase_index >= len(self.phases):
            return
        _, v, w = self.phases[self.phase_index]
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

        self.phase_elapsed += self.period
        if self.phase_elapsed >= self.phases[self.phase_index][0]:
            self.phase_index += 1
            self.phase_elapsed = 0.0
            if self.phase_index < len(self.phases):
                self.announce()
            else:
                self.get_logger().info("sequence finished (bridge falls back to neutral by cmd timeout)")
                self.finished = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StepTestNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
