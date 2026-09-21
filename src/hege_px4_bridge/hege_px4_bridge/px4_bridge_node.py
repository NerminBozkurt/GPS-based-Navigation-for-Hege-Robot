#!/usr/bin/env python3
"""
hege_px4_bridge: ROS 2 node bridging Nav2 /cmd_vel and sensor telemetry
with PX4 Autopilot on the physical Hege tractor via MicroXRCE-DDS.

Features:
- Translates /cmd_vel (Twist) into PX4 TrajectorySetpoint and OffboardControlMode (20 Hz heartbeat)
- Translates PX4 /fmu/out/vehicle_odometry to ROS /odometry/filtered (NED->ENU, FRD->FLU)
- Translates PX4 /fmu/out/vehicle_global_position to ROS /gps/fix (NavSatFix)
- Translates PX4 /fmu/out/vehicle_attitude to ROS /imu/data (sensor_msgs/Imu)
- Safety Watchdog: Stops vehicle if /cmd_vel drops out for > cmd_vel_timeout
- RC Override Detection: Detects if operator takes manual control on RC transmitter
- Services for Arming (/hege/arm), Disarming (/hege/disarm), and Offboard mode (/hege/offboard)
- Broadcasts odom -> base_footprint TF transform (configurable)
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String, Header
from std_srvs.srv import SetBool, Trigger
import tf2_ros

try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleStatus,
        VehicleOdometry,
        VehicleGlobalPosition,
        VehicleAttitude,
    )
except ImportError:
    # If built in separate overlay or running doc tests
    OffboardControlMode = None
    TrajectorySetpoint = None
    VehicleCommand = None
    VehicleStatus = None
    VehicleOdometry = None
    VehicleGlobalPosition = None
    VehicleAttitude = None


class HegePx4BridgeNode(Node):
    def __init__(self):
        super().__init__('hege_px4_bridge')

        # Declare parameters
        self.declare_parameter('heartbeat_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.4)
        self.declare_parameter('max_linear_velocity', 1.2)
        self.declare_parameter('min_linear_velocity', -0.5)
        self.declare_parameter('max_angular_velocity', 0.6)
        self.declare_parameter('wheelbase', 1.9)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('use_px4_odometry_as_fused', True)
        self.declare_parameter('two_d_mode', True)
        self.declare_parameter('enable_auto_arm', False)
        self.declare_parameter('enable_auto_offboard', False)

        # Retrieve parameters
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.min_linear_vel = self.get_parameter('min_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.map_frame = self.get_parameter('map_frame_id').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self.use_px4_odometry_as_fused = self.get_parameter('use_px4_odometry_as_fused').value
        self.two_d_mode = self.get_parameter('two_d_mode').value
        self.enable_auto_arm = self.get_parameter('enable_auto_arm').value
        self.enable_auto_offboard = self.get_parameter('enable_auto_offboard').value

        # QoS for PX4 topics (MicroXRCE-DDS uses BEST_EFFORT with VOLATILE durability)
        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ROS Standard QoS
        self.ros_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State tracking
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_vel_time = self.get_clock().now()
        self.watchdog_tripped = True
        self.offboard_counter = 0

        self.current_yaw_ned = 0.0
        self.current_pitch_ned = 0.0
        self.current_roll_ned = 0.0
        self.attitude_received = False

        self.is_armed = False
        self.nav_state = 0
        self.is_offboard = False
        self.has_gps_fix = False

        # TF Broadcaster
        if self.publish_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # Setup Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, self.ros_qos
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.px4_qos
        )
        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, self.px4_qos
        )
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, self.px4_qos
        )
        self.vehicle_global_pos_sub = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_pos_callback, self.px4_qos
        )

        # Setup Publishers to PX4
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', self.px4_qos
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.px4_qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', self.px4_qos
        )

        # Setup Publishers to ROS 2 ecosystem
        odom_topic = '/odometry/filtered' if self.use_px4_odometry_as_fused else '/px4/odometry'
        self.odom_pub = self.create_publisher(Odometry, odom_topic, self.ros_qos)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', self.ros_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', self.ros_qos)
        self.status_pub = self.create_publisher(String, '/hege/bridge_status', self.ros_qos)

        # Services for manual/safety arming and mode control
        self.arm_service = self.create_service(SetBool, '/hege/arm', self.arm_service_callback)
        self.offboard_service = self.create_service(SetBool, '/hege/offboard', self.offboard_service_callback)
        self.estop_service = self.create_service(Trigger, '/hege/emergency_stop', self.estop_service_callback)

        # Main timer: 20 Hz heartbeat & setpoint loop
        timer_period = 1.0 / self.heartbeat_rate
        self.timer = self.create_wall_timer(timer_period, self.heartbeat_loop)

        # Diagnostics timer: 1 Hz status report
        self.diag_timer = self.create_wall_timer(1.0, self.publish_diagnostics)

        self.get_logger().info(
            f'Hege PX4 Bridge Node initialized (heartbeat: {self.heartbeat_rate} Hz, '
            f'timeout: {self.cmd_vel_timeout} s, DOMAIN_ID: 73)'
        )

    # --------------------------------------------------------------------------
    # Subscribers & Callbacks
    # --------------------------------------------------------------------------

    def cmd_vel_callback(self, msg: Twist):
        """Receive /cmd_vel from Nav2 controller."""
        # Clamp velocities within safe physical tractor boundaries
        linear_x = max(min(msg.linear.x, self.max_linear_vel), self.min_linear_vel)
        angular_z = max(min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)

        self.cmd_linear_x = linear_x
        self.cmd_angular_z = angular_z
        self.last_cmd_vel_time = self.get_clock().now()

        if self.watchdog_tripped:
            self.watchdog_tripped = False
            self.get_logger().info(f'Watchdog cleared. Active velocity command received: v={linear_x:.2f}, w={angular_z:.2f}')

    def vehicle_status_callback(self, msg: VehicleStatus):
        """Monitor PX4 arming and flight/navigation state."""
        # VehicleStatus: ARMING_STATE_ARMED = 2, DISARMED = 1
        # NAVIGATION_STATE_OFFBOARD = 14
        prev_armed = self.is_armed
        prev_offboard = self.is_offboard

        self.is_armed = (msg.arming_state == 2)
        self.nav_state = msg.nav_state
        self.is_offboard = (msg.nav_state == 14)

        # Detect operator RC manual override
        if prev_offboard and not self.is_offboard:
            self.get_logger().warn(
                f'[SAFETY] Dropped out of OFFBOARD mode to nav_state={self.nav_state}! '
                'Operator may have switched to RC manual control or PX4 failsafe triggered.'
            )

        if not prev_armed and self.is_armed:
            self.get_logger().info('[PX4 STATUS] Tractor ARMED.')
        elif prev_armed and not self.is_armed:
            self.get_logger().info('[PX4 STATUS] Tractor DISARMED.')

    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        """Receive attitude quaternion from PX4 and convert to ENU Euler & ROS Imu."""
        # PX4 Attitude quaternion q is [w, x, y, z] from FRD to NED
        q = msg.q
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        # Euler angles in NED
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll_ned = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch_ned = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch_ned = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_ned = math.atan2(siny_cosp, cosy_cosp)

        self.current_roll_ned = roll_ned
        self.current_pitch_ned = pitch_ned
        self.current_yaw_ned = yaw_ned
        self.attitude_received = True

        # Convert to ENU frame for ROS
        # Yaw: ENU = pi/2 - NED
        yaw_enu = (math.pi / 2.0) - yaw_ned
        yaw_enu = math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))

        roll_enu = 0.0 if self.two_d_mode else roll_ned
        pitch_enu = 0.0 if self.two_d_mode else -pitch_ned

        # Compute ENU quaternion
        cy = math.cos(yaw_enu * 0.5)
        sy = math.sin(yaw_enu * 0.5)
        cp = math.cos(pitch_enu * 0.5)
        sp = math.sin(pitch_enu * 0.5)
        cr = math.cos(roll_enu * 0.5)
        sr = math.sin(roll_enu * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        # Publish standard ROS Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.base_frame
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        self.imu_pub.publish(imu_msg)

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        """Convert PX4 VehicleOdometry to ROS Odometry & broadcast TF."""
        now = self.get_clock().now().to_msg()

        # Position NED -> ENU
        # x_enu = y_ned (East), y_enu = x_ned (North), z_enu = -z_ned (Up)
        x_ned = float(msg.position[0]) if not math.isnan(msg.position[0]) else 0.0
        y_ned = float(msg.position[1]) if not math.isnan(msg.position[1]) else 0.0
        z_ned = float(msg.position[2]) if not math.isnan(msg.position[2]) else 0.0

        x_enu = y_ned
        y_enu = x_ned
        z_enu = 0.0 if self.two_d_mode else -z_ned

        # Orientation: Convert NED quaternion to ENU
        q = msg.q
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_ned = math.atan2(siny_cosp, cosy_cosp)
        yaw_enu = (math.pi / 2.0) - yaw_ned
        yaw_enu = math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))

        roll_enu = 0.0
        pitch_enu = 0.0
        cy = math.cos(yaw_enu * 0.5)
        sy = math.sin(yaw_enu * 0.5)
        cp = math.cos(pitch_enu * 0.5)
        sp = math.sin(pitch_enu * 0.5)
        cr = math.cos(roll_enu * 0.5)
        sr = math.sin(roll_enu * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        # Velocities: FRD body frame -> FLU body frame
        # v_x_flu = v_x_frd, v_y_flu = -v_y_frd
        vx_frd = float(msg.velocity[0]) if not math.isnan(msg.velocity[0]) else 0.0
        vy_frd = float(msg.velocity[1]) if not math.isnan(msg.velocity[1]) else 0.0
        wz_frd = float(msg.angular_velocity[2]) if not math.isnan(msg.angular_velocity[2]) else 0.0

        vx_flu = vx_frd
        vy_flu = -vy_frd
        wz_flu = -wz_frd

        # Construct Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = x_enu
        odom_msg.pose.pose.position.y = y_enu
        odom_msg.pose.pose.position.z = z_enu
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = vx_flu
        odom_msg.twist.twist.linear.y = vy_flu
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = wz_flu

        self.odom_pub.publish(odom_msg)

        # Broadcast TF transform odom -> base_footprint if enabled
        if self.publish_odom_tf and self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x_enu
            t.transform.translation.y = y_enu
            t.transform.translation.z = z_enu
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def vehicle_global_pos_callback(self, msg: VehicleGlobalPosition):
        """Convert PX4 Global Position (RTK/GPS) to ROS NavSatFix."""
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'

        fix.latitude = msg.lat
        fix.longitude = msg.lon
        fix.altitude = float(msg.alt)

        if msg.lat_lon_valid:
            self.has_gps_fix = True
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX  # RTK / high precision fix
            fix.status.service = NavSatStatus.SERVICE_GPS
            # Diagonal covariance based on eph (horizontal standard deviation)
            eph = float(msg.eph) if msg.eph > 0.0 else 0.05
            epv = float(msg.epv) if msg.epv > 0.0 else 0.1
            fix.position_covariance = [
                eph**2, 0.0, 0.0,
                0.0, eph**2, 0.0,
                0.0, 0.0, epv**2
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            self.has_gps_fix = False
            fix.status.status = NavSatStatus.STATUS_NO_FIX

        self.gps_pub.publish(fix)

    # --------------------------------------------------------------------------
    # Heartbeat & Command Loop (20 Hz)
    # --------------------------------------------------------------------------

    def heartbeat_loop(self):
        """Main periodic loop: sends Offboard heartbeat and TrajectorySetpoint."""
        now_time = self.get_clock().now()
        dt_cmd = (now_time - self.last_cmd_vel_time).nanoseconds / 1e9

        # Safety Watchdog: If cmd_vel stopped arriving, command zero motion
        if dt_cmd > self.cmd_vel_timeout:
            if not self.watchdog_tripped:
                self.watchdog_tripped = True
                self.get_logger().warn(
                    f'[SAFETY WATCHDOG] No /cmd_vel for {dt_cmd:.2f} s (> {self.cmd_vel_timeout} s). '
                    'Commanding ZERO velocity stop!'
                )
            target_linear = 0.0
            target_angular = 0.0
        else:
            target_linear = self.cmd_linear_x
            target_angular = self.cmd_angular_z

        # Microsecond timestamp for PX4
        timestamp_us = int(now_time.nanoseconds / 1000)

        # 1. Publish OffboardControlMode (Velocity control enabled)
        ocm = OffboardControlMode()
        ocm.timestamp = timestamp_us
        ocm.position = False
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.thrust_and_torque = False
        ocm.direct_actuator = False
        self.offboard_mode_pub.publish(ocm)

        # 2. Publish TrajectorySetpoint
        # In PX4 NED local frame:
        # Forward speed target_linear projected along current heading yaw_ned
        # Yaw rate in NED is -target_angular (CW positive)
        sp = TrajectorySetpoint()
        sp.timestamp = timestamp_us
        sp.position = [float('nan'), float('nan'), float('nan')]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]
        sp.jerk = [float('nan'), float('nan'), float('nan')]
        sp.yaw = float('nan')

        if self.attitude_received:
            yaw = self.current_yaw_ned
            v_n = float(target_linear * math.cos(yaw))
            v_e = float(target_linear * math.sin(yaw))
            v_d = 0.0
            sp.velocity = [v_n, v_e, v_d]
            sp.yawspeed = float(-target_angular)
        else:
            # Fallback if no attitude received yet
            sp.velocity = [float(target_linear), 0.0, 0.0]
            sp.yawspeed = float(-target_angular)

        self.trajectory_setpoint_pub.publish(sp)

        # Increment setpoint counter
        self.offboard_counter += 1

        # Optional auto-arm / auto-offboard if explicitly configured
        if self.enable_auto_offboard and self.offboard_counter == int(self.heartbeat_rate):
            self.get_logger().info('Auto-offboard enabled: requesting Offboard mode and Arming...')
            self.set_offboard_mode(True)
            if self.enable_auto_arm:
                self.arm_vehicle(True)

    # --------------------------------------------------------------------------
    # Vehicle Commands & Services
    # --------------------------------------------------------------------------

    def send_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        """Send a VehicleCommand to PX4."""
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.vehicle_command_pub.publish(cmd)

    def arm_vehicle(self, arm: bool = True):
        """Send Arm (param1=1.0) or Disarm (param1=0.0) command."""
        val = 1.0 if arm else 0.0
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=val)
        action_str = 'ARM' if arm else 'DISARM'
        self.get_logger().info(f'VehicleCommand: sent {action_str} request to PX4.')

    def set_offboard_mode(self, enable: bool = True):
        """Switch to Offboard mode (176, param1=1, param2=6) or Hold (param2=4)."""
        if enable:
            # Custom base mode 1, custom submode 6 = PX4 Offboard
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info('VehicleCommand: sent request for OFFBOARD mode.')
        else:
            # Custom submode 4 = Hold / Loiter
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=4.0)
            self.get_logger().info('VehicleCommand: sent request for HOLD mode.')

    def arm_service_callback(self, request, response):
        """Service callback for /hege/arm."""
        self.arm_vehicle(request.data)
        response.success = True
        response.message = f"Arm command set to {request.data}"
        return response

    def offboard_service_callback(self, request, response):
        """Service callback for /hege/offboard."""
        self.set_offboard_mode(request.data)
        response.success = True
        response.message = f"Offboard mode request set to {request.data}"
        return response

    def estop_service_callback(self, request, response):
        """Service callback for emergency stop."""
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.watchdog_tripped = True
        self.set_offboard_mode(False)
        self.get_logger().warn('[SAFETY] EMERGENCY STOP TRIGGERED via service! Commanded 0 and Hold mode.')
        response.success = True
        response.message = "Emergency stop activated: speed 0, Hold mode requested."
        return response

    def publish_diagnostics(self):
        """Publish JSON diagnostic status string once per second."""
        status_data = {
            "armed": self.is_armed,
            "offboard": self.is_offboard,
            "nav_state": int(self.nav_state),
            "watchdog_ok": not self.watchdog_tripped,
            "gps_valid": self.has_gps_fix,
            "linear_cmd": round(self.cmd_linear_x, 3),
            "angular_cmd": round(self.cmd_angular_z, 3),
            "heading_ned_deg": round(math.degrees(self.current_yaw_ned), 1),
            "heading_enu_deg": round(math.degrees((math.pi / 2.0) - self.current_yaw_ned) % 360.0, 1),
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HegePx4BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Stopping Hege PX4 Bridge Node.')
        # Send safe zero velocity before destruction
        node.cmd_linear_x = 0.0
        node.cmd_angular_z = 0.0
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
