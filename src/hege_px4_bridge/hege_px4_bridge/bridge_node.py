"""
hege_px4_bridge  --  geometry_msgs/Twist (/cmd_vel)  ->  PX4 offboard velocity setpoints

Data flow (runs at `rate_hz`, default 20 Hz):

    /cmd_vel (Nav2, teleop) ─┐
    /fmu/out/vehicle_attitude ─┼─> safety.evaluate() ─> ackermann.limit_command()
    /fmu/out/vehicle_status_v1 ┘        │                 ackermann.velocity_setpoint_ned()
    /hege/software_stop                 │                          │
                                        └─> /fmu/in/offboard_control_mode  (state-dependent heartbeat)
                                            /fmu/in/trajectory_setpoint    (motion or zero)
                                            /hege/bridge/status            (std_msgs/String)

Services (std_srvs/Trigger) -- all optional, the operator may instead use the
RC mode switch / QGroundControl:
    /hege/bridge/arm            /hege/bridge/disarm
    /hege/bridge/set_offboard   /hege/bridge/clear_software_stop

Every PX4 message field used here was checked against PX4 v1.16.1
(msg/versioned/*.msg, msg/*.msg) and px4_msgs branch release/1.16.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleAttitude,
                          VehicleCommand, VehicleCommandAck, VehicleStatus)

from hege_px4_bridge import ackermann
from hege_px4_bridge.command_ack import CommandAckWaiter
from hege_px4_bridge.safety import (BridgeState, Decision, SafetyConfig, SafetyInputs,
                                    evaluate)

# PX4 custom main mode number for OFFBOARD (PX4 custom_mode, used with
# VEHICLE_CMD_DO_SET_MODE: param1 = 1 "custom mode enabled", param2 = main mode).
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
SOURCE_SYSTEM = 1
SOURCE_COMPONENT = 191  # MAV_COMP_ID_ONBOARD_COMPUTER

ACK_RESULT_NAMES = {
    VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: "ACCEPTED",
    VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED",
    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: "DENIED",
    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: "UNSUPPORTED",
    VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: "FAILED",
    VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: "IN_PROGRESS",
    VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: "CANCELLED",
}

# QoS that matches the uXRCE-DDS agent publishers (see PX4 ROS 2 user guide).
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

PX4_IN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class Px4BridgeNode(Node):

    def __init__(self) -> None:
        super().__init__("hege_px4_bridge")

        # ---------------- parameters (see config/*.yaml) ----------------
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("status_timeout", 2.0)
        self.declare_parameter("attitude_timeout", 0.5)
        self.declare_parameter("command_ack_timeout", 1.5)
        self.declare_parameter("wheel_base", 0.0)            # RA_WHEEL_BASE   [m]
        self.declare_parameter("max_steering_angle", 0.0)    # RA_MAX_STR_ANG  [rad]
        self.declare_parameter("max_speed", 0.0)             # <= RO_SPEED_LIM [m/s]
        self.declare_parameter("max_yaw_rate", 0.0)          # <= RO_YAW_RATE_LIM [rad/s]
        self.declare_parameter("min_moving_speed", 0.05)     # [m/s]
        self.declare_parameter("px4_yaw_p", 0.0)             # must equal RO_YAW_P
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("vehicle_command_ack_topic", "/fmu/out/vehicle_command_ack")
        self.declare_parameter("allow_remote_vehicle_commands", False)

        p = self.get_parameter
        self.limits = ackermann.AckermannLimits(
            wheel_base=p("wheel_base").value,
            max_steering_angle=p("max_steering_angle").value,
            max_speed=p("max_speed").value,
            max_yaw_rate=p("max_yaw_rate").value,
            min_moving_speed=p("min_moving_speed").value,
        )
        self.limits.validate()                       # refuse to start with 0 / negative limits
        self.px4_yaw_p: float = p("px4_yaw_p").value
        if self.px4_yaw_p <= 0.0:
            raise ValueError("px4_yaw_p must be set to the PX4 parameter RO_YAW_P (> 0)")
        self.safety_cfg = SafetyConfig(
            cmd_timeout=p("cmd_timeout").value,
            status_timeout=p("status_timeout").value,
            attitude_timeout=p("attitude_timeout").value,
        )
        self.safety_cfg.validate()
        self.command_ack_timeout = float(p("command_ack_timeout").value)
        if not math.isfinite(self.command_ack_timeout) or self.command_ack_timeout <= 0.0:
            raise ValueError("command_ack_timeout must be finite and > 0")
        self.rate_hz = float(p("rate_hz").value)
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 2.0:
            raise ValueError("rate_hz must be finite and > 2 Hz for PX4 Offboard")
        self.allow_remote_vehicle_commands = bool(p("allow_remote_vehicle_commands").value)

        # ---------------- state ----------------
        self.estop = False
        self.software_stop_input = False
        self.cmd_v = 0.0                 # latest /cmd_vel linear.x  [m/s]
        self.cmd_omega = 0.0             # latest /cmd_vel angular.z [rad/s], ROS sign
        self.cmd_time: float | None = None
        self.psi = 0.0                   # yaw in NED [rad]
        self.attitude_time: float | None = None
        self.nav_state: int | None = None
        self.arming_state: int | None = None
        self.status_time: float | None = None
        self.last_state: BridgeState | None = None
        self.last_ack = "none"
        self.last_disarm_request = float("-inf")
        self.heartbeat_stream_started: float | None = None
        self.command_group = ReentrantCallbackGroup()
        self.command_service_lock = threading.Lock()
        self.ack_waiter = CommandAckWaiter(
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS)

        # ---------------- publishers ----------------
        self.pub_offboard = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_IN_QOS)
        self.pub_setpoint = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_IN_QOS)
        self.pub_command = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_IN_QOS)
        self.pub_status = self.create_publisher(String, "/hege/bridge/status", 10)

        # ---------------- subscribers ----------------
        self.create_subscription(Twist, p("cmd_vel_topic").value, self.on_cmd_vel, 10)
        self.create_subscription(Bool, "/hege/software_stop", self.on_software_stop, 10)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.on_attitude, PX4_QOS)
        self.create_subscription(VehicleStatus, p("vehicle_status_topic").value, self.on_status, PX4_QOS)
        self.create_subscription(VehicleCommandAck, p("vehicle_command_ack_topic").value,
                                 self.on_command_ack, PX4_QOS,
                                 callback_group=self.command_group)

        # ---------------- services ----------------
        self.create_service(Trigger, "/hege/bridge/arm", self.srv_arm,
                            callback_group=self.command_group)
        self.create_service(Trigger, "/hege/bridge/disarm", self.srv_disarm,
                            callback_group=self.command_group)
        self.create_service(Trigger, "/hege/bridge/set_offboard", self.srv_set_offboard,
                            callback_group=self.command_group)
        self.create_service(Trigger, "/hege/bridge/clear_software_stop", self.srv_clear_software_stop)

        # ---------------- control loop ----------------
        self.create_timer(1.0 / self.rate_hz, self.control_step)
        self.get_logger().info(
            f"bridge started: L={self.limits.wheel_base} m, delta_max={self.limits.max_steering_angle} rad, "
            f"v_max={self.limits.max_speed} m/s, omega_lim={self.limits.max_yaw_rate} rad/s, "
            f"RO_YAW_P={self.px4_yaw_p}")

    # ------------------------------------------------------------------ helpers
    def now_s(self) -> float:
        """Monotonic time for watchdogs; immune to NTP, UTC and /clock jumps."""
        return time.monotonic()

    def now_us(self) -> int:
        """PX4 timestamps are microseconds; the uXRCE-DDS agent re-bases them."""
        return int(self.get_clock().now().nanoseconds / 1000)

    @staticmethod
    def ack_result_name(result: int) -> str:
        return ACK_RESULT_NAMES.get(result, f"UNKNOWN({result})")

    # ------------------------------------------------------------------ callbacks
    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_v = msg.linear.x
        self.cmd_omega = msg.angular.z
        self.cmd_time = self.now_s()

    def on_software_stop(self, msg: Bool) -> None:
        self.software_stop_input = bool(msg.data)
        if msg.data and not self.estop:
            self.get_logger().error(
                "SOFTWARE STOP received: zero setpoint, heartbeat drop and disarm request; "
                "this does not replace the physical emergency stop")
        if msg.data:
            self.cmd_v = 0.0
            self.cmd_omega = 0.0
            self.cmd_time = None
        self.estop = self.estop or msg.data          # latch: only the service can clear

    def on_attitude(self, msg: VehicleAttitude) -> None:
        q = msg.q                                     # (w, x, y, z), FRD body -> NED
        if all(math.isfinite(c) for c in q):
            self.psi = ackermann.yaw_from_px4_quaternion((q[0], q[1], q[2], q[3]))
            self.attitude_time = self.now_s()

    def on_status(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.status_time = self.now_s()
        if (msg.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                or msg.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            # Never reuse a command that arrived before arming/entering Offboard.
            self.cmd_v = 0.0
            self.cmd_omega = 0.0
            self.cmd_time = None

    def on_command_ack(self, msg: VehicleCommandAck) -> None:
        # A vehicle may have other MAVLink/ROS command sources. Only an ACK
        # addressed back to this bridge can complete one of our services.
        if (msg.target_system != SOURCE_SYSTEM
                or msg.target_component != SOURCE_COMPONENT):
            return

        result = self.ack_result_name(msg.result)
        self.last_ack = f"command={msg.command} result={result}"
        self.ack_waiter.update(msg.command, msg.result)
        log = self.get_logger().info if result == "ACCEPTED" else self.get_logger().warning
        log(f"PX4 command acknowledgement: {self.last_ack}")

    # ------------------------------------------------------------------ control loop
    def control_step(self) -> None:
        now = self.now_s()

        # 1) may we move, and should Offboard proof-of-life continue?
        decision: Decision = evaluate(
            SafetyInputs(now=now, estop=self.estop, status_time=self.status_time,
                         attitude_time=self.attitude_time, cmd_time=self.cmd_time,
                         nav_state=self.nav_state, arming_state=self.arming_state),
            self.safety_cfg)

        # 2) heartbeat is deliberately dropped for SOFTWARE_STOP/PX4_STALE.
        if decision.stream_heartbeat:
            if self.heartbeat_stream_started is None:
                self.heartbeat_stream_started = now
            self.publish_offboard_control_mode()
        else:
            self.heartbeat_stream_started = None

        # 3) compute the setpoint
        if decision.allow_motion:
            limited = ackermann.limit_command(self.cmd_v, self.cmd_omega, self.limits)
            if limited.stopped:
                v_ned = ackermann.stop_setpoint_ned()
            else:
                omega_ned = ackermann.ros_yaw_rate_to_ned(limited.omega)
                v_ned = ackermann.velocity_setpoint_ned(limited.v, omega_ned, self.psi, self.px4_yaw_p)
            detail = f"v={limited.v:.2f} omega={limited.omega:.2f} ({limited.reason})"
        else:
            v_ned = ackermann.stop_setpoint_ned()
            detail = "neutral"

        # A ROS software stop is only an additional layer. Request disarm once
        # per second while latched and rely on the independent hardware E-stop.
        if decision.state == BridgeState.SOFTWARE_STOP and now - self.last_disarm_request >= 1.0:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
            self.last_disarm_request = now

        # 4) send it
        self.publish_trajectory_setpoint(v_ned)
        self.publish_status(decision, detail)

    # ------------------------------------------------------------------ PX4 messages
    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = False
        msg.velocity = True          # -> PX4 AckermannVelControl offboard branch
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.pub_offboard.publish(msg)

    def publish_trajectory_setpoint(self, v_ned: tuple[float, float, float]) -> None:
        nan = float("nan")
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [nan, nan, nan]          # NaN = "do not control this quantity"
        msg.velocity = [float(v_ned[0]), float(v_ned[1]), float(v_ned[2])]   # NED, m/s
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]
        msg.yaw = nan
        msg.yawspeed = nan
        self.pub_setpoint.publish(msg)

    def publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = SOURCE_SYSTEM
        msg.source_component = SOURCE_COMPONENT
        msg.from_external = True
        self.pub_command.publish(msg)

    def send_vehicle_command_and_wait(self, command: int, param1: float = 0.0,
                                      param2: float = 0.0) -> tuple[bool, str]:
        """Publish one command and return true only for a matching ACCEPTED ACK.

        The service lock rejects concurrent command services. The waiter is
        armed before publication so an immediate ACK cannot be missed.
        """
        if not self.command_service_lock.acquire(blocking=False):
            return False, "another PX4 command service is already waiting for an acknowledgement"
        try:
            self.ack_waiter.begin(command)
            try:
                self.publish_vehicle_command(command, param1=param1, param2=param2)
            except Exception:
                self.ack_waiter.cancel()
                raise

            result = self.ack_waiter.wait(self.command_ack_timeout)
            if result is None:
                self.last_ack = f"command={command} result=TIMEOUT"
                return False, (f"PX4 acknowledgement timed out after "
                               f"{self.command_ack_timeout:.1f} s")

            result_name = self.ack_result_name(result)
            accepted = result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
            return accepted, f"PX4 acknowledgement: {result_name}"
        finally:
            self.command_service_lock.release()

    def publish_status(self, decision: Decision, detail: str) -> None:
        if decision.state != self.last_state:
            self.get_logger().info(f"bridge state -> {decision.state.value}")
            self.last_state = decision.state
        msg = String()
        msg.data = (f"{decision.state.value} | nav_state={self.nav_state} arming={self.arming_state} "
                    f"| psi={self.psi:+.2f} rad | {detail} | ack={self.last_ack}")
        self.pub_status.publish(msg)

    # ------------------------------------------------------------------ services
    def srv_arm(self, _req, res):
        if not self.allow_remote_vehicle_commands:
            res.success, res.message = False, "remote arming disabled; use RC/QGroundControl"
            return res
        res.success, res.message = self.send_vehicle_command_and_wait(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        return res

    def srv_disarm(self, _req, res):
        res.success, res.message = self.send_vehicle_command_and_wait(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        return res

    def srv_set_offboard(self, _req, res):
        if not self.allow_remote_vehicle_commands:
            res.success, res.message = False, "remote mode changes disabled; use RC/QGroundControl"
            return res
        if (self.heartbeat_stream_started is None
                or self.now_s() - self.heartbeat_stream_started < 1.0):
            res.success, res.message = False, "wait until Offboard heartbeat has streamed for 1 second"
            return res
        # PX4 only accepts the switch if OffboardControlMode is already streaming (we always do).
        res.success, res.message = self.send_vehicle_command_and_wait(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        )
        return res

    def srv_clear_software_stop(self, _req, res):
        if self.software_stop_input:
            res.success, res.message = False, "publish software_stop=false before clearing the latch"
            return res
        self.estop = False
        self.cmd_v = 0.0
        self.cmd_omega = 0.0
        self.cmd_time = None
        res.success, res.message = True, "software-stop latch cleared; PX4 remains disarmed"
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4BridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
