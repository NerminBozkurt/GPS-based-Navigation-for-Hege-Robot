"""
hege_px4_sensors  --  PX4 /fmu/out/* topics  ->  standard ROS 2 sensor topics

    /fmu/out/vehicle_odometry     -> /px4/odom
    /fmu/out/vehicle_attitude     -> orientation used inside /px4/imu
    /fmu/out/sensor_combined      -> /px4/imu
    /fmu/out/vehicle_gps_position -> /px4/gps/fix

All frame math lives in frames.py (unit-tested).  This file only unpacks and
packs messages.  Field names were checked against PX4 v1.16.1 message
definitions (msg/versioned/VehicleOdometry.msg, msg/SensorGps.msg,
msg/SensorCombined.msg, msg/versioned/VehicleAttitude.msg).

Timestamps: PX4 fills `timestamp` in microseconds.  With the default
UXRCE_DDS_SYNCT = 1 the uXRCE-DDS agent re-bases these to the companion's OS
clock, so they can be used directly as ROS stamps (stamp_source = "px4").
If time sync is disabled or you run with a simulated /clock, use
stamp_source = "ros" (stamp = time of arrival).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import SensorCombined, SensorGps, VehicleAttitude, VehicleOdometry

from hege_px4_sensors import frames as fr

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def diag6(values) -> list[float]:
    """6x6 row-major covariance with only the diagonal set (ROS convention)."""
    cov = [0.0] * 36
    for i, v in enumerate(values):
        cov[i * 7] = float(v)
    return cov


class Px4SensorsNode(Node):

    def __init__(self) -> None:
        super().__init__("hege_px4_sensors")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "base_link")   # PX4 reports IMU already rotated into body
        self.declare_parameter("gps_frame", "base_link")   # change only after publishing a measured static TF
        self.declare_parameter("publish_tf", False)         # one node only may own odom->base_link
        self.declare_parameter("stamp_source", "px4")      # "px4" | "ros"
        self.declare_parameter("odom_topic", "/px4/odom")
        self.declare_parameter("imu_topic", "/px4/imu")
        self.declare_parameter("gps_topic", "/px4/gps/fix")
        self.declare_parameter("max_attitude_age", 0.2)     # [s] orientation age allowed in Imu
        self.declare_parameter("unknown_variance", 1.0e6)   # never publish false zero certainty

        p = self.get_parameter
        self.odom_frame: str = p("odom_frame").value
        self.base_frame: str = p("base_frame").value
        self.imu_frame: str = p("imu_frame").value
        self.gps_frame: str = p("gps_frame").value
        self.publish_tf: bool = p("publish_tf").value
        self.stamp_source: str = p("stamp_source").value
        self.max_attitude_age_us = int(float(p("max_attitude_age").value) * 1_000_000)
        self.unknown_variance = float(p("unknown_variance").value)
        if self.stamp_source not in ("px4", "ros"):
            raise ValueError("stamp_source must be 'px4' or 'ros'")
        if self.max_attitude_age_us <= 0 or self.unknown_variance <= 0.0:
            raise ValueError("max_attitude_age and unknown_variance must be > 0")

        self.q_ros_latest: fr.Quat | None = None       # latest attitude, ROS convention
        self.q_timestamp_us: int | None = None
        self.warned_pose_frames: set[int] = set()
        self.warned_velocity_frames: set[int] = set()

        self.pub_odom = self.create_publisher(Odometry, p("odom_topic").value, 10)
        self.pub_imu = self.create_publisher(Imu, p("imu_topic").value, 50)
        self.pub_fix = self.create_publisher(NavSatFix, p("gps_topic").value, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odometry, PX4_QOS)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.on_attitude, PX4_QOS)
        self.create_subscription(SensorCombined, "/fmu/out/sensor_combined", self.on_imu, PX4_QOS)
        self.create_subscription(SensorGps, "/fmu/out/vehicle_gps_position", self.on_gps, PX4_QOS)
        self.get_logger().info(f"sensors node started (stamp_source={self.stamp_source})")

    # ------------------------------------------------------------------ helpers
    def stamp(self, px4_timestamp_us: int):
        if self.stamp_source == "px4":
            return Time(seconds=px4_timestamp_us // 1_000_000,
                        nanoseconds=(px4_timestamp_us % 1_000_000) * 1000).to_msg()
        return self.get_clock().now().to_msg()

    @staticmethod
    def to_quaternion_msg(q: fr.Quat) -> Quaternion:
        return Quaternion(w=float(q[0]), x=float(q[1]), y=float(q[2]), z=float(q[3]))

    @staticmethod
    def to_vector3(v) -> Vector3:
        return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))

    # ------------------------------------------------------------------ callbacks
    def on_attitude(self, msg: VehicleAttitude) -> None:
        q = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        if fr.is_finite(q):
            self.q_ros_latest = fr.px4_attitude_to_ros(q)
            self.q_timestamp_us = int(msg.timestamp)

    def on_odometry(self, msg: VehicleOdometry) -> None:
        q_px4 = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        if not (fr.is_finite(msg.position) and fr.is_finite(q_px4)):
            return                                   # PX4 uses NaN for "invalid"
        # POSE_FRAME_FRD is world-fixed but has an arbitrary heading reference.
        # Treating it as NED would silently rotate position and attitude. Reject it
        # until a measured/reference transform is explicitly available.
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            if msg.pose_frame not in self.warned_pose_frames:
                self.get_logger().warning(
                    f"ignoring VehicleOdometry pose_frame={msg.pose_frame}; only earth-fixed NED is supported")
                self.warned_pose_frames.add(msg.pose_frame)
            return

        stamp = self.stamp(msg.timestamp)
        p_enu = fr.ned_to_enu(tuple(msg.position))
        q_ros = fr.px4_attitude_to_ros(q_px4)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = p_enu
        odom.pose.pose.orientation = self.to_quaternion_msg(q_ros)

        # position variance (N,E,D) -> (E,N,U): swap first two, Down/Up flip keeps variance
        unknown = (self.unknown_variance,) * 3
        pv = msg.position_variance if fr.is_finite(msg.position_variance) else unknown
        ov = msg.orientation_variance if fr.is_finite(msg.orientation_variance) else unknown
        odom.pose.covariance = diag6([pv[1], pv[0], pv[2], ov[0], ov[1], ov[2]])

        if fr.is_finite(msg.velocity):
            v_flu = fr.body_velocity_flu_from_px4(tuple(msg.velocity), msg.velocity_frame, q_px4)
            if v_flu is not None:
                odom.twist.twist.linear = self.to_vector3(v_flu)
            elif msg.velocity_frame not in self.warned_velocity_frames:
                self.get_logger().warning(
                    f"publishing pose without linear twist: unsupported VehicleOdometry "
                    f"velocity_frame={msg.velocity_frame}; only NED and BODY_FRD are supported")
                self.warned_velocity_frames.add(msg.velocity_frame)
        if fr.is_finite(msg.angular_velocity):
            odom.twist.twist.angular = self.to_vector3(fr.frd_to_flu(tuple(msg.angular_velocity)))
        vv = msg.velocity_variance if fr.is_finite(msg.velocity_variance) else None
        vv_body = (fr.body_velocity_variance_flu_from_px4(tuple(vv), msg.velocity_frame, q_px4)
                   if vv is not None else None)
        linear_var = vv_body if vv_body is not None else unknown
        odom.twist.covariance = diag6([
            linear_var[0], linear_var[1], linear_var[2],
            self.unknown_variance, self.unknown_variance, self.unknown_variance,
        ])
        self.pub_odom.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation = self.to_vector3(p_enu)
            tf.transform.rotation = self.to_quaternion_msg(q_ros)
            self.tf_broadcaster.sendTransform(tf)

    def on_imu(self, msg: SensorCombined) -> None:
        imu = Imu()
        imu.header.stamp = self.stamp(msg.timestamp)
        imu.header.frame_id = self.imu_frame
        imu.angular_velocity = self.to_vector3(fr.frd_to_flu(tuple(msg.gyro_rad)))
        if msg.accelerometer_timestamp_relative != SensorCombined.RELATIVE_TIMESTAMP_INVALID:
            imu.linear_acceleration = self.to_vector3(fr.frd_to_flu(tuple(msg.accelerometer_m_s2)))
        else:
            imu.linear_acceleration_covariance[0] = -1.0
        attitude_is_fresh = (self.q_ros_latest is not None and self.q_timestamp_us is not None
                             and abs(int(msg.timestamp) - self.q_timestamp_us) <= self.max_attitude_age_us)
        if attitude_is_fresh:
            imu.orientation = self.to_quaternion_msg(self.q_ros_latest)
        else:
            imu.orientation_covariance[0] = -1.0     # ROS convention: orientation unknown
        self.pub_imu.publish(imu)

    def on_gps(self, msg: SensorGps) -> None:
        coordinates = (msg.latitude_deg, msg.longitude_deg, msg.altitude_ellipsoid_m)
        if not fr.is_finite(coordinates):
            return
        fix = NavSatFix()
        fix.header.stamp = self.stamp(msg.timestamp)
        fix.header.frame_id = self.gps_frame
        fix.latitude = float(msg.latitude_deg)
        fix.longitude = float(msg.longitude_deg)
        fix.altitude = float(msg.altitude_ellipsoid_m)  # NavSatFix altitude is above the WGS-84 ellipsoid
        fix.status.service = NavSatStatus.SERVICE_GPS
        if msg.fix_type >= SensorGps.FIX_TYPE_RTK_FLOAT:
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX      # RTK float / fixed
        elif msg.fix_type >= SensorGps.FIX_TYPE_2D:
            fix.status.status = NavSatStatus.STATUS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        if (fix.status.status != NavSatStatus.STATUS_NO_FIX
                and fr.is_finite((msg.eph, msg.epv)) and msg.eph >= 0.0 and msg.epv >= 0.0):
            eph2, epv2 = float(msg.eph) ** 2, float(msg.epv) ** 2
            fix.position_covariance = [eph2, 0.0, 0.0, 0.0, eph2, 0.0, 0.0, 0.0, epv2]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_fix.publish(fix)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4SensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
