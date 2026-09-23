# Driving the real Hege autonomously

You can send `/cmd_vel` and the wheels turn. That means the whole command
chain works on hardware: `twist_mux` picks a source, `hege_px4_bridge` converts
it to a PX4 offboard velocity setpoint, PX4 accepts it and drives the
actuators. This document is about the step after that — giving the vehicle a
destination instead of a velocity, and letting it work out the velocities
itself.

## What is actually missing

Less than it sounds. `hege_localization` and `hege_navigation` already do this
job; they have driven a full waypoint mission in simulation. What was missing
was a launch file that starts them against the Pixhawk instead of against
Gazebo, because `real.launch.py` starts only the plumbing.

That file is now `hege_real.launch.py`. The difference between "I send
velocities" and "it decides its own velocities" is:

| | `real.launch.py` | `hege_real.launch.py` |
|---|---|---|
| Agent, `twist_mux`, bridge, sensors | yes | yes |
| `robot_state_publisher` (TF) | no | yes |
| Dual EKF + `navsat_transform` | no | yes |
| Nav2 | no | yes |

## Before it runs: three things to measure

These are not optional and two of them are enforced in code.

**1. `RO_YAW_P`, and `px4_yaw_p` to match.** The bridge asks PX4 for a yaw rate
by offsetting the heading setpoint by `omega / RO_YAW_P`. The two cancel only
if the number is right; if it is wrong the rover turns at the wrong rate and
nothing reports an error. Tune `RO_YAW_P` in Stabilized mode following PX4's
"Configuration/Tuning (Ackermann Rover)" page, read it off QGroundControl, and
put the same value in `bridge_real.yaml`. It ships as `0.0` and **the node
refuses to start** until you change it. That refusal is the safety feature.

**2. The geometry, in QGroundControl.** These have to agree with
`hege.urdf.xacro`, which is where the repository keeps the vehicle:

| Parameter | Value |
|---|---|
| `RA_WHEEL_BASE` | 1.90 |
| `RA_MAX_STR_ANG` | 0.611 rad (35°) |

**3. `RO_MAX_THR_SPEED`.** The ground speed at full throttle. Everything PX4
does with throttle scales off it. Measure it — drive at full throttle in a
straight line, in a cleared area, and read the steady-state speed. Then set
`RO_SPEED_LIM` to something at or above the 0.3 m/s the bridge caps at.

**Also worth setting before the first autonomous run:** the magnetic
declination. `navsat_transform` in `dual_ekf_navsat.yaml` has
`magnetic_declination_radians: 0.0`, which is the simulation value — the
simulated IMU reports true ENU heading and has no magnetic field. The real
compass does. QGroundControl shows the declination for your position; convert
to radians and put it in. Left at zero the rover drives at a constant angle to
where it thinks it is going, a few degrees off, which looks like a controller
problem and is not.

## Running it

On the Jetson, with a good GPS fix already:

```bash
ROS_DOMAIN_ID=73 ros2 launch hege_bringup hege_real.launch.py
```

**Launching this does not make the rover move.** `bridge_real.yaml` keeps
`allow_remote_vehicle_commands: false`, so nothing in the stack can arm the
vehicle or change its mode — that stays with RC and QGroundControl,
deliberately. The launch brings everything up and waits.

Check before arming, from a second sourced terminal:

```bash
ros2 topic echo /hege/bridge/status --once   # NOT_OFFBOARD is correct here
ros2 topic echo /gps/fix --once              # a plausible latitude/longitude
ros2 topic echo /odometry/filtered_map --once
ros2 run tf2_ros tf2_echo map base_footprint
```

The last one is the real test: if `map -> odom -> base_footprint` resolves, the
vehicle knows where it is and Nav2 has something to plan against.

## The dry run: a goal, and nothing moves

Before any of the above, do this. It answers "does the rover know where it is
and does Nav2 plan something sane" without a single actuator being involved,
and it works before the three measurements exist.

```bash
ROS_DOMAIN_ID=73 ros2 launch hege_bringup hege_real_dry_run.launch.py
```

That is `hege_real.launch.py` with `hege_px4_bridge` swapped for its `dry_run`
node. The bridge is the only thing in the stack that publishes to `/fmu/in/*`,
so with it not running there is no path from Nav2 to the vehicle at all - arming
would change nothing. It also sidesteps the `px4_yaw_p` refusal: the dry run
needs the geometry and the limits out of `bridge_real.yaml`, not the gain.

RViz opens by default. Wait for `map -> odom -> base_footprint` to resolve,
give a **2D Goal Pose** a few metres ahead, and watch:

```bash
ros2 topic echo /hege/dry_run/report        # what the wheels would be told
ros2 topic echo /cmd_vel/nav                # what Nav2 decided, before limiting
```

A line of the report reads:

```
v=0.30 m/s  omega=+0.100 rad/s  steer=+32.3 deg  R=3.0 m  (asked +0.35, +0.180)  [saturated]
```

Left to right: the speed and yaw rate the bridge would act on, the front-wheel
angle that delivers them, the radius of the circle that is, what Nav2 actually
asked for, and whether anything was clipped on the way. `steer` is the one to
watch - it is the number with a mechanical limit you can see on the vehicle, and
its sign is where a steering reversal shows up.

Three things worth checking in that output, because each is a real fault that
looks like nothing:

- **The sign.** Positive `steer` is left. If the plan bends left and `steer` is
  negative, something is mirrored, and finding that here costs nothing.
- **`[saturated]` on a straight line.** Means Nav2 is asking for more than the
  bridge allows even when it should not be, usually a speed mismatch between
  `nav2_real_overlay.yaml` and `bridge_real.yaml`.
- **`-> STOP (...)`** with a plan on screen. The reason says which rule fired.
  `reverse requested` means the planner produced a path this vehicle cannot
  drive; check that the PX4 overlay is really loaded.

The same thing works in the Gazebo Classic simulation, where it is worth doing
first because nothing there is expensive. Start the simulation as usual but
send Nav2's output somewhere the controller is not listening, and read it with
the dry run instead:

```bash
ros2 launch hege_description spawn_hege.launch.py
ros2 launch hege_localization localization.launch.py
ros2 launch hege_navigation navigation.launch.py rviz:=true cmd_vel_topic:=/cmd_vel/nav
ros2 run hege_px4_bridge dry_run --ros-args -p cmd_vel_topic:=/cmd_vel/nav
```

`cmd_vel_topic:=/cmd_vel/nav` is what makes the simulated wheels stay still.
`spawn_hege.launch.py` starts `cmd_vel_relay`, which listens on `/cmd_vel` and
is the only thing feeding the Ackermann controller; publishing on
`/cmd_vel/nav` instead leaves it with nothing to relay while the planner keeps
running. Drop that argument and it drives, exactly as before.

**This does not need Gazebo Harmonic.** Harmonic is only needed for PX4 SITL,
which this is not - there is no PX4 in the simulation dry run at all. The
Classic simulation that already drove a waypoint mission is the right place.

## Giving it one waypoint

For a first autonomous move, **use RViz rather than a waypoint file**. A file
takes latitude and longitude, which means knowing where the rover is to a metre
before you start. RViz lets you click a point a few metres ahead and see the
plan before anything moves:

```bash
ros2 launch hege_bringup hege_real.launch.py rviz:=true
```

Then **2D Goal Pose**, a few metres directly in front, pointing the way the
rover already faces — a goal that needs no turn at all is the right first test.
Watch `/plan` appear in RViz. Nothing moves yet, because PX4 is not armed.

Once you are happy with the plan, arm and go to offboard from RC or
QGroundControl. `/hege/bridge/status` should go to `ACTIVE` and the rover
should start.

When that works, a waypoint file drives a route instead of a single goal:

```bash
ros2 run hege_navigation gps_waypoint_follower.py --ros-args \
  -p waypoints_file:=/path/to/waypoints.yaml
```

The format is in `hege_navigation/config/waypoints_short_run.yaml`. Note that
the legs there are 10 m, sized around a 6 m turning radius — keep that in mind
before shortening them.

## The order to do it in

1. The dry run above, first in simulation and then on the vehicle. It needs
   none of the measurements below and cannot move anything.
2. Set the three parameters above. The bridge will not start otherwise.
3. **Driven wheels lifted.** Launch, arm, give a goal, and watch the wheels
   turn and the steering move. Confirm the steering goes the way the plan
   bends — this is where a sign error shows up, and it is much cheaper to find
   here.
4. Wheels down, cleared area, one goal a few metres straight ahead. RC
   takeover in someone's hands, physical E-stop within reach.
5. A goal that needs a turn.
6. A short waypoint route.
7. Then start tuning against what you measured.

## What to watch, and what it means

`/hege/bridge/status` reports the first failing check, in order of severity:

| State | Meaning |
|---|---|
| `SOFTWARE_STOP` | The latch is set. Clear it with `/hege/bridge/clear_software_stop` after publishing `software_stop=false`. |
| `WAITING_PX4` | No `VehicleStatus` yet. Agent down, or `ROS_DOMAIN_ID` is not 73. |
| `PX4_STALE` | The link dropped. The bridge also stops the heartbeat, handing over to PX4's own failsafe. |
| `NOT_OFFBOARD` | PX4 is not armed, or not in offboard. Expected before you arm. |
| `CMD_TIMEOUT` | Armed and in offboard, but nothing is commanding. Nav2 has no goal, or its output is not reaching `/cmd_vel/nav`. |
| `ACTIVE` | Motion allowed. |

Two failure modes worth naming because they do not look like what they are:

**The rover creeps or stalls just short of the goal.** The bridge turns
anything below `min_moving_speed` (0.05 m/s) into a full stop, and the goal
checker accepts 1.5 m. Both are deliberate. It has arrived.

**The rover drives past a waypoint and circles.** PX4 cannot reverse in
offboard — its offboard velocity branch is forward-only, and the bridge turns a
negative `linear.x` into a stop. A forward-only car that overshoots has no way
back except a full loop. Widen the goal tolerance or lengthen the legs; do not
try to make it reverse.

## What is still guesswork

Everything in `nav2_real_overlay.yaml` is a starting point. The numbers in
`nav2_params.yaml` were measured by driving the simulated rover and reading
what it did; nothing has been measured on the real vehicle at 0.3 m/s. Expect
to change:

- `lookahead_dist`, which at this speed is pinned to `min_lookahead_dist` of
  2.0 m. Too short and a 1.90 m wheelbase oscillates; too long and pure pursuit
  cuts corners.
- `minimum_turning_radius`, still 6.0 m from the simulation. The real
  achievable radius at 0.3 m/s is probably tighter, since tyre slip grows with
  speed.
- `xy_goal_tolerance`, 1.5 m, which means the rover stops that far short.

And four knobs in `dual_ekf_navsat.yaml` are commented out with a note saying
they are exactly what real hardware will demand: outlier rejection for when RTK
drops from Fixed to Float and the fix jumps, `smooth_lagged_data` for a
receiver that needs 100–200 ms to compute a fix, `predict_to_current_time`, and
a non-zero `transform_timeout` for a link that is not a single machine.
Uncomment them when the logs show they are needed rather than pre-emptively —
but know they are there.

## The thing that does not change

The vehicle is over a tonne and has **no lidar, radar or depth camera**. It
cannot detect or avoid a person, an animal or an obstacle, and nothing in this
software will stop for one. Autonomous operation means a cleared, controlled
area with a person watching and an emergency stop in reach. The software stop
is a ROS-level latch, not a substitute.
