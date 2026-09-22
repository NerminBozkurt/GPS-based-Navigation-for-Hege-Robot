# Driving the Hege model with PX4 SITL

The project has had two simulations that did not meet. Ours plans and tracks a
GPS route across open ground, but drives the rover through a Gazebo Classic
Ackermann controller that the real machine does not have. The PX4 side proves
the Pixhawk link, but drives PX4's stock 0.321 m demo rover, which is not our
vehicle in any respect that matters.

This document covers joining them: our Nav2 and our localization, on our
measured vehicle, with PX4 in the middle.

## Why the two simulators cannot simply merge

PX4 v1.16 SITL runs **Gazebo Harmonic**. The ROS simulation originally ran
**Gazebo Classic** — a different simulator with different plugin names,
different SDF handling and a different launch API — and there is no
configuration that makes one host the other.

Worse, the two cannot even be installed side by side: their Debian packages
both ship `/usr/bin/gz` and conflict, so installing Harmonic apt-removes the
whole Classic ROS stack. A machine therefore runs one or the other.

That is what `spawn_hege_gz.launch.py` is for: the navigation simulation ported
to Harmonic, so a machine set up for PX4 can still run it.
`docs/gazebo_harmonic.md` covers that port. On a navigation machine,
`spawn_hege.launch.py` and Gazebo Classic stay the default.

The other thing that does not carry over is `ros2_control`. PX4 does not use
it: PX4 owns the actuators and writes the wheel and steering joints directly
over gz-transport. In the PX4 path there is no `controller_manager`, no
`ackermann_steering_controller` and no `cmd_vel_relay`.

## What the rover looks like to PX4

PX4 finds a vehicle's sensors at topic paths that are **hardcoded** in
`GZBridge.cpp`:

    /world/$WORLD/model/$MODEL/link/base_link/sensor/imu_sensor/imu
    /world/$WORLD/model/$MODEL/link/base_link/sensor/navsat_sensor/navsat
    /world/$WORLD/model/$MODEL/link/base_link/sensor/magnetometer_sensor/magnetometer
    /world/$WORLD/model/$MODEL/link/base_link/sensor/air_pressure_sensor/air_pressure

The link name and all four sensor names are fixed by PX4. A model that calls
its IMU anything else boots, appears in Gazebo, looks healthy, and never gives
EKF2 an IMU reading.

That constraint is the reason for the one subtle thing in `hege.urdf.xacro`.
URDF to SDF conversion lumps a fixed joint's child into its parent, and
`base_link` is `base_footprint`'s fixed child — so without intervention the
converted model has no link called `base_link` at all. The `px4` branch sets
`<preserveFixedJoint>` on `base_joint` to stop that, which in turn makes
`base_footprint` a real root link, which is why it grows a 1e-4 kg inertia in
that mode and no other.

The sensors themselves sit on `base_link` with an explicit pose carrying the
measured lever arms (IMU at 1.20 m, GNSS antenna at 1.30 m above the ground).
`imu_link` and `gps_link` still exist in the URDF for `robot_state_publisher`
and `navsat_transform`; they are simply not where the simulated hardware is
mounted.

## One source of truth for the geometry

The model PX4 spawns is **generated from the same xacro** the Gazebo Classic
simulation uses:

    hege.urdf.xacro --(xacro drive:=px4)--> URDF --(gz sdf -p)--> model.sdf

`hege_px4_sim/scripts/generate_hege_model.sh` does both steps, renames the
model to `hege_rover`, and then checks that the five names PX4 needs survived
the conversion. `model.sdf` is a build artefact and is not committed; the
xacro is the thing to edit.

The airframe file `4100_gz_hege_rover` carries the same numbers again as PX4
parameters, because PX4 cannot read a URDF. Those are the ones to keep in
step by hand:

| Quantity | xacro | PX4 airframe | bridge config |
|---|---|---|---|
| Wheelbase | `wheelbase` 1.91 | `RA_WHEEL_BASE` 1.91 | `wheel_base` 1.91 |
| Max steering | `max_steer` 0.6 rad | `RA_MAX_STR_ANG` 0.6 rad, `SIM_GZ_SV_MAXA1` 34.38 deg | `max_steering_angle` 0.6 |
| Heading gain | — | `RO_YAW_P` 3 | `px4_yaw_p` 3.0 |

The third column matters most. The bridge asks PX4 for a yaw rate by offsetting
the heading setpoint by `omega / RO_YAW_P`; if the two numbers disagree the
rover turns at the wrong rate and nothing reports an error.

## Wheel speeds, and the one number to measure

PX4 commands the gz joint controller in rad/s, as `mixer output − 100`. So
`SIM_GZ_WH_MAX1` sets the top wheel speed directly, and on the 0.40 m rear
wheels:

    SIM_GZ_WH_MAX1 = 108  ->  8 rad/s  ->  8 x 0.40 = 3.2 m/s

`RO_MAX_THR_SPEED` in the airframe is set to that 3.2. It is a calculation, not
a measurement: tyre slip and the contact parameters in the URDF both pull the
real figure below it. Drive at full throttle in SITL, read the steady-state
ground speed, and correct the parameter. Everything PX4 does with throttle
depends on it being right.

## Setting it up

Once, in the devcontainer, with a PX4 v1.16.1 checkout that has its submodules
initialised (`Tools/simulation/gz` is one):

    cd src/hege_px4_sim
    ./scripts/generate_hege_model.sh
    ./scripts/install_to_px4.sh ~/PX4-Autopilot

The install symlinks the model into `Tools/simulation/gz/models/hege_rover`,
copies the airframe into the ROMFS and registers it in the airframe
`CMakeLists.txt`. `--uninstall` reverses all three. Then:

    cd ~/PX4-Autopilot
    make px4_sitl gz_hege_rover

The first build after registering an airframe regenerates the ROMFS and is
slower than an incremental one.

## Running it

PX4 SITL first, in its own terminal, as above. Then:

    ros2 launch hege_bringup hege_sitl.launch.py
    ros2 launch hege_bringup hege_sitl.launch.py rviz:=true

The chain that comes up:

    PX4 SITL + Gazebo Harmonic
        | uXRCE-DDS
        v
    hege_px4_sensors   -> /imu/data, /gps/fix, /px4/odom
        v
    hege_localization  (dual EKF + navsat_transform)
        v
    hege_navigation    (Nav2)
        v
    /cmd_vel/nav -> twist_mux -> /cmd_vel/selected
        v
    hege_px4_bridge    -> /fmu/in/trajectory_setpoint

Before the rover will move, PX4 has to be armed and in offboard mode. In SITL
`allow_remote_vehicle_commands` is true, so the bridge's own services do it:

    ros2 service call /hege/bridge/set_offboard std_srvs/srv/Trigger
    ros2 service call /hege/bridge/arm std_srvs/srv/Trigger
    ros2 topic echo /hege/bridge/status --once

The status line should read `ACTIVE` once a command is flowing. The states
before that are diagnostic: `WAITING_PX4` means no `VehicleStatus` has arrived
(check the agent and `ROS_DOMAIN_ID`), `NOT_OFFBOARD` means PX4 has not
accepted the mode or the arm.

## How the existing stack was reused

`hege_localization` and `hege_navigation` are included **unchanged**. The whole
difference is arguments, which is the point of doing it this way:

| Argument | Gazebo | PX4 | Why |
|---|---|---|---|
| `use_sim_time` | `true` | `false` | There is no `/clock` in the PX4 path. The agent re-bases PX4 timestamps onto the companion clock. |
| `odom_topic` | `/ackermann_steering_controller/odometry` | `/px4/odom` | No wheel encoders exist behind a Pixhawk. |
| `cmd_vel_topic` | `cmd_vel` | `/cmd_vel/nav` | Into `twist_mux`, so teleop and the step test can override Nav2. |
| `bt_variant` | `ackermann` | `px4` | The PX4 trees drop the `BackUp` recovery. |
| `params_overlay` | (none) | `nav2_px4_overlay.yaml` | Forward-only planner and controller. |

Two of those are worth expanding.

**Velocity input.** `ekf_local` fuses forward speed from wheel odometry, and
there is no wheel odometry on the PX4 interface. `/px4/odom` takes its place.
That is EKF2's own output, so its *position* must not be fused as well — it is
built from the same GPS that `navsat_transform` already feeds in. Only `vx` is
taken, which `odom0_config` in `dual_ekf_navsat.yaml` was already doing for the
wheels. It is still a partial double-count, since EKF2's velocity is partly
GPS-derived, and it is the honest cost of keeping one navigation stack for both
worlds.

**Reverse.** PX4's offboard velocity branch for an Ackermann rover drives
forwards only, and `ackermann.limit_command()` turns any negative `linear.x`
into a full stop. Our `nav2_params.yaml` plans with `REEDS_SHEPP`, sets
`allow_reversing: true` and has a `BackUp` recovery — all three would produce
motion the bridge silently refuses. The overlay switches the planner to
`DUBIN` and the controller to forward-only, and the `px4` behaviour trees drop
`BackUp`, leaving costmap clearing and waiting as the recoveries.

Speed needs no overlay *in SITL*: `bridge_hege_sitl.yaml` caps `max_speed` at
1.0 m/s, exactly the `desired_linear_vel` Nav2 was tuned at. The planner's
6.0 m minimum turning radius only ever asks for 1/6 = 0.167 rad/s, well inside
the bridge's 0.35 rad/s limit, so neither clamp bites.

## Against the real rover

`hege_sitl.launch.py` is SITL only. The same wiring would work against the real
Pixhawk with `real.launch.py`'s agent settings, but `bridge_real.yaml` caps
`max_speed` at 0.3 m/s deliberately, and at that speed Nav2's tuning does not
carry over:

- `desired_linear_vel` 1.0 would be clamped to 0.3 by the bridge, with the
  controller tracking against a speed it is not getting;
- `min_approach_linear_velocity` is 0.3, the same as the cap, leaving nothing
  for the approach to slow down into;
- the yaw-rate cap of 0.10 rad/s at 0.3 m/s implies a 3.0 m operational
  turning radius, which is tighter than the planner's 6.0 m — so the planner is
  still the binding constraint, but only just.

A real-vehicle overlay is a separate piece of tuning and should be measured,
not guessed. `bridge_real.yaml` also ships `px4_yaw_p: 0.0`, which the node
refuses to start on, until somebody reads the tuned `RO_YAW_P` out of
QGroundControl.

## What has actually been verified

Being precise about this, because a model that looks right and is wrong costs
a day:

- **Verified here.** All three `drive` modes expand through xacro cleanly, with
  no cross-contamination: `planar` and `ros2_control` contain no gz-Harmonic
  plugins and `px4` contains no Gazebo Classic plugins and no `ros2_control`
  block. The generated `px4` URDF carries `base_link`, the preserved
  `base_joint` and all four PX4 sensor names. All YAML, XML and shell files
  parse. The bridge and sensor unit tests pass, 141 of them.
- **Not verified here.** Nothing has been run. This container has neither
  Gazebo, nor PX4, nor a ROS installation, so `gz sdf -p` has never been
  executed against this URDF, `model.sdf` has never been generated, the launch
  files have only been syntax-checked, and no vehicle has moved. The numbers in
  the airframe are derived from the geometry rather than measured.

The first run is where this gets tested. Likely first failures, in the order
worth checking: the steering turning the wrong way (flip `SIM_GZ_SV_REV`), the
full-throttle speed not matching `RO_MAX_THR_SPEED`, and `sdformat` warning
about something it dropped during the URDF conversion — which is why
`generate_hege_model.sh` leaves its stderr visible and greps the result.
