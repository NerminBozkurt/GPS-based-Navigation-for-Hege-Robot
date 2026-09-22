# Talking to the Pixhawk: the bridge and the sensor conversion

`docs/px4_ros2_topics.md` writes down what the Pixhawk exposes over ROS 2 and
ends on the awkward part: there is no `/cmd_vel` on that interface, and nothing
on it speaks `sensor_msgs` either. This document covers the three packages that
close both ends of that gap. They were written by Oğuzhan Enes Işık against the
real Pixhawk and against PX4 SITL, and were developed in
[oguzissik/hege_gps_navigation](https://github.com/oguzissik/hege_gps_navigation)
before being brought into this repository.

| Package | What it does |
| --- | --- |
| `hege_px4_bridge` | `geometry_msgs/Twist` → PX4 offboard velocity setpoints, with a safety supervisor |
| `hege_px4_sensors` | PX4 NED/FRD topics → standard ROS `Odometry`, `Imu`, `NavSatFix` |
| `hege_bringup` | `twist_mux` command arbitration plus the real and SITL launch files |

Everything here needs `px4_msgs` from branch `release/1.16` in the workspace,
built against the same firmware the board runs. That package is cloned rather
than vendored, so it is in `.gitignore`; `EXTERNAL_DEPS.txt` pins the revisions
the code was developed against.

## The command path

PX4 offboard mode does not accept a yaw rate for a rover. In v1.16 the
Ackermann velocity controller reads a velocity *vector* in NED, takes its
heading as the heading setpoint and its magnitude as the speed setpoint, and
then runs its own proportional heading controller on top:

    psi_sp      = atan2(v_E, v_N)
    speed_sp    = |v_NE|
    yaw_rate_sp = RO_YAW_P * wrap_pi(psi_sp - psi)

So the only way to ask PX4 for a yaw rate is to hand it a heading that is
offset from the current one by exactly the amount its own P controller will
turn back into that rate:

    psi_sp = psi + omega_ned / RO_YAW_P

That is what `ackermann.velocity_setpoint_ned()` computes, and it is why
`px4_yaw_p` in the bridge config **has to equal the PX4 parameter `RO_YAW_P`**.
The two numbers cancel; if they disagree the rover turns at the wrong rate.
`config/bridge_real.yaml` deliberately ships `px4_yaw_p: 0.0` and the node
refuses to start at that value — the real number has to be read out of
QGroundControl after `RO_YAW_P` has been tuned in Stabilized mode.

Before any of that, `ackermann.limit_command()` decides what the rover is
allowed to do at all. Reverse is rejected outright, because PX4's offboard
velocity branch drives forwards only. Rotating in place is rejected, because a
car cannot. Anything below `min_moving_speed` becomes a full stop. What
survives is clamped to `max_speed`, and the yaw rate is clamped to the smaller
of `max_yaw_rate` and the kinematic limit `v * tan(delta_max) / L`.

`safety.evaluate()` runs once per control cycle and answers a narrower
question: may the bridge send a *moving* setpoint right now, and should the
offboard heartbeat keep streaming? The distinction matters. An ordinary command
timeout keeps the heartbeat alive and sends zero velocity, so PX4 stays in
offboard and stops cleanly. A latched software stop or a stale PX4 link drops
the heartbeat as well, which hands control to PX4's own
`COM_OF_LOSS_T` / `COM_OBL_RC_ACT` failsafe. The states, in the order they are
checked, are `SOFTWARE_STOP`, `WAITING_PX4`, `PX4_STALE`, `NOT_OFFBOARD`,
`CMD_TIMEOUT`, `ACTIVE`, and the first one that matches is what
`/hege/bridge/status` reports.

Arm, disarm, offboard and clearing the software stop are `std_srvs/Trigger`
services under `/hege/bridge/`. They do not return until PX4 sends back a
matching `VehicleCommandAck` addressed to this bridge, or the ACK times out —
`command_ack.py` handles that matching and the fast-ACK race. On the real
vehicle `allow_remote_vehicle_commands` is `false`, so arming and mode changes
stay with RC and QGroundControl; the services are there for a supervised
acceptance test, not for routine use.

## The sensor path

`hege_px4_sensors` is the mirror image and is much simpler: it unpacks PX4
messages, converts frames, and republishes.

| PX4 topic | ROS topic | Type |
| --- | --- | --- |
| `/fmu/out/vehicle_odometry` | `/px4/odom` | `nav_msgs/Odometry` |
| `/fmu/out/sensor_combined` (+ `vehicle_attitude`) | `/px4/imu` | `sensor_msgs/Imu` |
| `/fmu/out/vehicle_gps_position` | `/px4/gps/fix` | `sensor_msgs/NavSatFix` |

All the frame maths lives in `frames.py` and is unit-tested without ROS: NED to
ENU for the world frame, FRD to FLU for the body frame, and
`q_ros = Q_ENU_NED * q_px4 * Q_FRD_FLU` for attitude. Two details are worth
knowing. The node refuses `VehicleOdometry` whose `pose_frame` is not NED,
because `POSE_FRAME_FRD` is world-fixed with an arbitrary heading reference
that the message does not carry — treating it as NED would silently rotate
everything. And when PX4 reports an invalid variance the node substitutes
`unknown_variance` (1e6) rather than publishing a zero, which an EKF would read
as perfect certainty.

`/px4/odom` is EKF2's fused output, not wheel odometry. It already contains the
GPS and the IMU. Feeding it to `ekf_global` alongside `/odometry/gps` would be
fusing the same measurement twice.

## How this lines up with the rest of the repository

The three packages were developed as a standalone PX4 stack, so they do not yet
plug straight into `hege_localization` and `hege_navigation`. Nothing is broken
by their presence — `hege_bringup`'s launch files are separate from
`spawn_hege.launch.py`, `localization.launch.py` and `navigation.launch.py`,
and none of the existing packages changed. But five things have to be settled
before the real robot drives a waypoint route, and they are listed here so they
do not have to be rediscovered.

**Where Nav2's output lands.** In simulation the chain ends at `/cmd_vel`,
which `cmd_vel_relay` picks up (`navigation.launch.py` remaps
`velocity_smoother`'s `cmd_vel_smoothed` onto it). The bridge instead reads
`/cmd_vel/selected`, which `twist_mux` produces from `/cmd_vel/nav`,
`/cmd_vel/test` and `/cmd_vel/teleop` at priorities 10, 50 and 100. On the real
robot the smoother's output has to be remapped to `/cmd_vel/nav` instead, which
also buys the teleop override for free.

**Reverse.** `nav2_params.yaml` plans with `REEDS_SHEPP` and sets
`allow_reversing: true`, and `behavior_server` still offers `BackUp`. The
bridge turns every `v < 0` into a full stop. Left as it is, Nav2 would plan a
reverse manoeuvre, command it, and watch the rover sit still until the action
times out. Before real runs the planner needs `DUBIN`, the controller needs
`allow_reversing: false`, and `BackUp` has to go.

**Speed.** `desired_linear_vel` is 1.0 m/s in `nav2_params.yaml`;
`bridge_real.yaml` caps `max_speed` at 0.3 m/s for the first tests. The bridge
clamps silently, so Nav2's controller would be tracking a path at a third of
the speed it planned for. The two numbers have to be set together.

**Turning radius.** This one already agrees. The planner asks for at least
6.0 m, well outside the 2.73 m geometric minimum and outside the 3.0 m
operational radius that `max_speed: 0.3` and `max_yaw_rate: 0.10` imply, so the
bridge's yaw clamp will not bite. `hege_bringup/config/nav2_ackermann_hints.yaml`
is the author's own starting point for these values and is kept as a reference;
`hege_navigation/config/nav2_params.yaml` remains the configuration this
repository actually runs.

**Frames and topic names.** `hege_localization` consumes `/gps/fix` and
`/imu/data` and calls the base frame `base_footprint`; `sensors.yaml` publishes
`/px4/gps/fix` and `/px4/imu` with `base_frame: base_link`. Both are
parameters, so this is a config change rather than a code change. Two settings
in `sensors.yaml` should not move, though: `publish_tf` stays `false`, because
`ekf_local` owns `odom -> base_footprint` and only one node may, and `gps_frame`
should become `gps_link` — the URDF already places that link at the measured
1.30 m — so the EKF applies the antenna offset instead of assuming the fix was
taken at the vehicle origin.

## Running it

SITL, with PX4 started separately from a `v1.16.1` checkout:

    make px4_sitl gz_rover_ackermann          # in PX4-Autopilot
    ros2 launch hege_bringup sim.launch.py
    ros2 launch hege_bringup sim.launch.py step_test:=true   # scripted slow drive

Real robot, on the Jetson, with the agent already running:

    ROS_DOMAIN_ID=73 ros2 launch hege_bringup real.launch.py start_agent:=false

Domain 73 and UDP 8888 are the values the hardware is set up for; the same 73
appears in `docs/px4_ros2_topics.md`. What to check once it is up:

    ros2 topic hz /fmu/out/vehicle_attitude      # about 100 Hz
    ros2 topic hz /fmu/in/offboard_control_mode  # about 20 Hz
    ros2 topic echo /hege/bridge/status --once

The pure-Python parts of both packages are tested without a ROS installation:

    python3 -m pytest src/hege_px4_bridge/test src/hege_px4_sensors/test

`scripts/collect_interface_inventory.sh` and `scripts/record_discovery_bag.sh`
capture the node/topic/service inventory and a full rosbag, which is how the
topic tables in `docs/px4_ros2_topics.md` were produced.

## Before the vehicle moves

The rover is about 1300 kg and carries no lidar, radar or depth camera. This
software stack cannot see a person, an animal or an obstacle, and nothing in it
will stop for one. The software stop is a ROS-level latch, not a substitute for
the physical emergency stop. First motion tests go with the driven wheels
lifted, RC takeover ready, and the 0.3 m/s limit in place.
