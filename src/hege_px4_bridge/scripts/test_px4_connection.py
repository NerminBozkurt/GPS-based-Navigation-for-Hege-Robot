#!/usr/bin/env python3
"""Diagnostic script to quickly verify telemetry and communication with PX4 on DOMAIN_ID=73."""

import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

try:
    from px4_msgs.msg import VehicleStatus, VehicleGlobalPosition, VehicleOdometry, VehicleAttitude
except ImportError:
    print("[ERROR] px4_msgs not found in ROS 2 environment. Did you run 'source install/setup.bash'?")
    sys.exit(1)


class PX4ConnectionTester(Node):
    def __init__(self):
        super().__init__('px4_connection_tester')

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.status_count = 0
        self.gps_count = 0
        self.odom_count = 0
        self.attitude_count = 0

        self.last_status = None
        self.last_gps = None
        self.last_odom = None
        self.last_attitude = None

        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.cb_status, self.qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.cb_status, self.qos)
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.cb_gps, self.qos)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.cb_odom, self.qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.cb_attitude, self.qos)

    def cb_status(self, msg):
        self.status_count += 1
        self.last_status = msg

    def cb_gps(self, msg):
        self.gps_count += 1
        self.last_gps = msg

    def cb_odom(self, msg):
        self.odom_count += 1
        self.last_odom = msg

    def cb_attitude(self, msg):
        self.attitude_count += 1
        self.last_attitude = msg


def main():
    domain_id = os.environ.get('ROS_DOMAIN_ID', 'not set')
    print("=" * 65)
    print(f"  HEGE PX4 CONNECTION TESTER (ROS_DOMAIN_ID: {domain_id})")
    print("=" * 65)
    print("Listening for PX4 telemetry on /fmu/out/* for 5 seconds...")

    rclpy.init()
    tester = PX4ConnectionTester()

    start_time = time.time()
    while time.time() - start_time < 5.0:
        rclpy.spin_once(tester, timeout_sec=0.1)

    print("\n--- RESULTS ---")
    print(f"VehicleStatus packets received:         {tester.status_count}")
    print(f"VehicleGlobalPosition packets received: {tester.gps_count}")
    print(f"VehicleOdometry packets received:       {tester.odom_count}")
    print(f"VehicleAttitude packets received:       {tester.attitude_count}")

    if tester.status_count > 0:
        s = tester.last_status
        armed_str = "ARMED" if s.arming_state == 2 else "DISARMED"
        print(f"\n[VehicleStatus] State: {armed_str}, Nav State ID: {s.nav_state}")
    else:
        print("\n[VehicleStatus] NO DATA RECEIVED from /fmu/out/vehicle_status")

    if tester.gps_count > 0:
        g = tester.last_gps
        print(f"[GPS/RTK] Lat: {g.lat:.7f}, Lon: {g.lon:.7f}, Alt: {g.alt:.2f} m, Valid: {g.lat_lon_valid}, EPH: {g.eph:.2f} m")
    else:
        print("[GPS/RTK] NO DATA RECEIVED from /fmu/out/vehicle_global_position")

    if tester.odom_count > 0:
        o = tester.last_odom
        print(f"[Odometry] Position (NED): [{o.position[0]:.2f}, {o.position[1]:.2f}, {o.position[2]:.2f}] m")
        print(f"[Odometry] Velocity (FRD): [{o.velocity[0]:.2f}, {o.velocity[1]:.2f}, {o.velocity[2]:.2f}] m/s")
    else:
        print("[Odometry] NO DATA RECEIVED from /fmu/out/vehicle_odometry")

    print("=" * 65)
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
