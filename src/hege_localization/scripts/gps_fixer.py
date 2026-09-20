#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSFixer(Node):
    def __init__(self):
        super().__init__('gps_fixer')
        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.cb, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix_fixed', 10)
        self.get_logger().info('GPS Fixer started!')

    def cb(self, msg):
        # Gazebo Classic spherical coordinates omits cos(lat) when projecting X (East) meters to longitude degrees.
        # We scale the longitude delta by 1 / cos(lat) so UTM/map X matches Gazebo world X exactly 1:1.
        import math
        cos_lat = math.cos(math.radians(52.466))
        msg.latitude += 52.466
        msg.longitude = (msg.longitude / cos_lat) + 12.958
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GPSFixer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
