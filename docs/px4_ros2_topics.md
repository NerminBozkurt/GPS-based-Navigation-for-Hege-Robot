# What the Pixhawk offers over ROS 2

This is the topic list the Pixhawk actually exposes on our setup, captured with
`ros2 topic list -t` on the companion computer. It is written down so we can
plan against the real interface instead of against the PX4 documentation, which
lists messages that a given firmware build may not bridge.

Everything below comes over the uXRCE-DDS bridge, so the topics only exist
while the agent is running on the companion and the client is running on the
flight controller.

## How it was captured

    ping 192.168.1.3                    # flight controller link, 0% loss
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=73
    ros2 daemon stop && ros2 daemon start
    ros2 topic list -t

Two things matter for anyone repeating this. The domain ID is **73** — with the
default domain 0 the list comes back empty and it looks like the bridge is
down. And the daemon has to be restarted after changing the domain, otherwise
it keeps serving the discovery cache of the previous domain.

All message types are from `px4_msgs`, so that package has to be in the
workspace and built against the same firmware version as the board; a version
mismatch shows up as topics that exist but never deliver a message.

## Direction

`/fmu/in/...` is what PX4 subscribes to — we publish on these to command the
vehicle. `/fmu/out/...` is what PX4 publishes — we subscribe to these to read
state. The names are from PX4's point of view, not ours.

## /fmu/out — what we can read (24 topics)

| Topic | Type |
| --- | --- |
| `/fmu/out/airspeed_validated` | `AirspeedValidated` |
| `/fmu/out/arming_check_request` | `ArmingCheckRequest` |
| `/fmu/out/battery_status` | `BatteryStatus` |
| `/fmu/out/collision_constraints` | `CollisionConstraints` |
| `/fmu/out/estimator_status_flags` | `EstimatorStatusFlags` |
| `/fmu/out/failsafe_flags` | `FailsafeFlags` |
| `/fmu/out/home_position` | `HomePosition` |
| `/fmu/out/manual_control_setpoint` | `ManualControlSetpoint` |
| `/fmu/out/message_format_response` | `MessageFormatResponse` |
| `/fmu/out/mode_completed` | `ModeCompleted` |
| `/fmu/out/position_setpoint_triplet` | `PositionSetpointTriplet` |
| `/fmu/out/register_ext_component_reply` | `RegisterExtComponentReply` |
| `/fmu/out/sensor_combined` | `SensorCombined` |
| `/fmu/out/timesync_status` | `TimesyncStatus` |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` |
| `/fmu/out/vehicle_command_ack` | `VehicleCommandAck` |
| `/fmu/out/vehicle_control_mode` | `VehicleControlMode` |
| `/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` |
| `/fmu/out/vehicle_gps_position` | `SensorGps` |
| `/fmu/out/vehicle_land_detected` | `VehicleLandDetected` |
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` |
| `/fmu/out/vehicle_odometry` | `VehicleOdometry` |
| `/fmu/out/vehicle_status_v1` | `VehicleStatus` |
| `/fmu/out/vtol_vehicle_status` | `VtolVehicleStatus` |

## /fmu/in — what we can command (27 topics)

| Topic | Type |
| --- | --- |
| `/fmu/in/actuator_motors` | `ActuatorMotors` |
| `/fmu/in/actuator_servos` | `ActuatorServos` |
| `/fmu/in/arming_check_reply` | `ArmingCheckReply` |
| `/fmu/in/aux_global_position` | `VehicleGlobalPosition` |
| `/fmu/in/config_control_setpoints` | `VehicleControlMode` |
| `/fmu/in/config_overrides_request` | `ConfigOverrides` |
| `/fmu/in/distance_sensor` | `DistanceSensor` |
| `/fmu/in/goto_setpoint` | `GotoSetpoint` |
| `/fmu/in/manual_control_input` | `ManualControlSetpoint` |
| `/fmu/in/message_format_request` | `MessageFormatRequest` |
| `/fmu/in/mode_completed` | `ModeCompleted` |
| `/fmu/in/obstacle_distance` | `ObstacleDistance` |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` |
| `/fmu/in/onboard_computer_status` | `OnboardComputerStatus` |
| `/fmu/in/register_ext_component_request` | `RegisterExtComponentRequest` |
| `/fmu/in/sensor_optical_flow` | `SensorOpticalFlow` |
| `/fmu/in/telemetry_status` | `TelemetryStatus` |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` |
| `/fmu/in/unregister_ext_component` | `UnregisterExtComponent` |
| `/fmu/in/vehicle_attitude_setpoint` | `VehicleAttitudeSetpoint` |
| `/fmu/in/vehicle_command` | `VehicleCommand` |
| `/fmu/in/vehicle_command_mode_executor` | `VehicleCommand` |
| `/fmu/in/vehicle_mocap_odometry` | `VehicleOdometry` |
| `/fmu/in/vehicle_rates_setpoint` | `VehicleRatesSetpoint` |
| `/fmu/in/vehicle_thrust_setpoint` | `VehicleThrustSetpoint` |
| `/fmu/in/vehicle_torque_setpoint` | `VehicleTorqueSetpoint` |
| `/fmu/in/vehicle_visual_odometry` | `VehicleOdometry` |

Besides these there are only the two ROS defaults, `/parameter_events` and
`/rosout`.

## What this means for our navigation stack

The simulation stack localizes with `robot_localization` from a raw GPS fix and
an IMU, and drives an Ackermann controller. On the real robot the Pixhawk sits
between us and both ends of that chain, so the mapping is:

**Localization input.** `vehicle_gps_position` is the raw receiver output
(`SensorGps`: lat/lon/alt, fix type, satellite count, reported accuracies) and
is the closest match to what `navsat_transform` consumes today.
`vehicle_global_position` and `vehicle_local_position` are the EKF2 output
instead — already fused with the IMU. So there is a choice to make: feed our
own filter the raw fix and keep the simulation pipeline intact, or take EKF2's
estimate and run two filters in series. `sensor_combined` gives the raw IMU if
we go the first way, and `estimator_status_flags` tells us whether EKF2 trusts
its own solution if we go the second.

**Heading.** Nothing here publishes a magnetometer reading directly;
`vehicle_attitude` is the estimated attitude quaternion. With a single GPS
antenna the yaw is only observable while moving, which is the same limitation
we already have in simulation.

**Control output.** There is no `cmd_vel` on this interface. For a ground
vehicle the usable paths are `trajectory_setpoint` (position/velocity setpoint
in NED, needs `offboard_control_mode` published continuously at >2 Hz or PX4
drops out of offboard) or `goto_setpoint` for a simpler position goal.
`actuator_motors` / `actuator_servos` bypass PX4's controllers entirely and go
straight to the outputs — direct, but it means we own the low-level control.
Either way arming and mode changes go through `vehicle_command`, with the
result coming back on `vehicle_command_ack`.

**Watch out for.** `failsafe_flags` and `vehicle_status_v1` are what tell us
PX4 has taken control away from us; anything we build should react to them
rather than keep publishing setpoints into a vehicle that stopped listening.
`timesync_status` matters because PX4 timestamps are microseconds since boot,
not ROS time, and every setpoint we publish has to carry a PX4-clock timestamp
or it is rejected as stale.

`airspeed_validated`, `vtol_vehicle_status` and `vehicle_land_detected` come
from the airframe-agnostic message set and mean nothing for a ground rover.
