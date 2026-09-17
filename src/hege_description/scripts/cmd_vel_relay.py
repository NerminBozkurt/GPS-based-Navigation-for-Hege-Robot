#!/usr/bin/env python3
"""Relay /cmd_vel to the Ackermann controller's reference topic, with a watchdog.

Two jobs:

1. Bridge the topic names. ackermann_steering_controller subscribes on
   ``~/reference_unstamped``, i.e.
   ``/ackermann_steering_controller/reference_unstamped``. Nav2 and
   teleop_twist_keyboard publish plain ``/cmd_vel``, and controllers spawned
   inside gazebo_ros2_control cannot be given remap rules.

2. Hold the controller at zero when nobody is commanding it, so the rover stops
   if whatever was driving it (Nav2, teleop) dies mid-run instead of coasting on
   the last reference it received.

Equivalent to ``ros2 run topic_tools relay`` plus the watchdog - swap the first
half for that if topic_tools is ever installed.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

TARGET = '/ackermann_steering_controller/reference_unstamped'
WATCHDOG_PERIOD = 0.1   # s, how often we tick
COMMAND_TIMEOUT = 0.5   # s without /cmd_vel before we force zero


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(Twist, TARGET, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.last_stamp = None
        self.create_timer(WATCHDOG_PERIOD, self.tick)
        self.get_logger().info(
            'Relaying /cmd_vel -> %s (zero after %.1fs idle)' % (TARGET, COMMAND_TIMEOUT))

    def on_cmd(self, msg):
        self.last_stamp = self.get_clock().now()
        self.pub.publish(msg)

    def tick(self):
        stale = (self.last_stamp is None or
                 (self.get_clock().now() - self.last_stamp).nanoseconds * 1e-9 > COMMAND_TIMEOUT)
        if stale:
            self.pub.publish(Twist())


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
