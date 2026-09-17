#!/usr/bin/env python3
"""Bridge /cmd_vel to the Ackermann controller's reference topic, with a watchdog.

Three jobs:

1. Convert the message type. ackermann_steering_controller wants
   ``geometry_msgs/TwistStamped`` on ``~/reference``; Nav2 and
   teleop_twist_keyboard publish plain ``geometry_msgs/Twist`` on ``/cmd_vel``.
   The unstamped path the controller also offers is deprecated and disappears
   in ROS 2 J-Turtle, so the conversion happens here instead.

2. Stamp it. The timestamp is what lets the controller distinguish a fresh
   command from one that has been sitting in a queue, which is what its
   reference_timeout acts on. The stamp comes from this node's clock, which
   runs on sim time in Gazebo, so it is comparable to the controller's own.

3. Hold the controller at zero when nobody is commanding it, so the rover stops
   if whatever was driving it (Nav2, teleop) dies mid-run instead of coasting on
   the last reference it received.

The topic names cannot be fixed with remap rules: controllers spawned inside
gazebo_ros2_control do not accept them.
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

TARGET = '/ackermann_steering_controller/reference'
BASE_FRAME = 'base_footprint'
WATCHDOG_PERIOD = 0.1   # s, how often we tick
COMMAND_TIMEOUT = 0.5   # s without /cmd_vel before we force zero


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(TwistStamped, TARGET, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.last_stamp = None
        self.create_timer(WATCHDOG_PERIOD, self.tick)
        self.get_logger().info(
            'Relaying /cmd_vel -> %s (zero after %.1fs idle)' % (TARGET, COMMAND_TIMEOUT))

    def stamped(self, twist):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.twist = twist
        return msg

    def on_cmd(self, msg):
        self.last_stamp = self.get_clock().now()
        self.pub.publish(self.stamped(msg))

    def tick(self):
        stale = (self.last_stamp is None or
                 (self.get_clock().now() - self.last_stamp).nanoseconds * 1e-9 > COMMAND_TIMEOUT)
        if stale:
            self.pub.publish(self.stamped(Twist()))


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
