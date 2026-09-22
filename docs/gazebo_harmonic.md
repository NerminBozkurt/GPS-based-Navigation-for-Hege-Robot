# The Gazebo Harmonic port of the simulation

The navigation simulation runs on Gazebo **Classic**, through
`spawn_hege.launch.py`. That is the default and it is what every measured
number in the tuning came from.

`spawn_hege_gz.launch.py` is the same simulation ported to Gazebo
**Harmonic**. This document is about that port: why it exists, what changed in
it, and what to install to run it.

## Why it exists at all

Gazebo Classic and Gazebo Harmonic **cannot be installed on the same machine**.
Their Debian packages both ship `/usr/bin/gz` and conflict outright:

    gz-tools2 : Conflicts: gazebo (>= 11.0.0)
                Conflicts: gazebo (<= 11.14.0)

`apt install gz-harmonic` therefore removes `gazebo`, `ros-humble-gazebo-ros`,
`ros-humble-gazebo-plugins`, `ros-humble-gazebo-ros2-control` and everything
else in the Classic ROS stack, without asking twice. It is worth knowing that
this is how it fails, because the symptom afterwards is
`package 'gazebo_ros' not found` and nothing says a simulator was uninstalled.

PX4 v1.16 SITL requires Harmonic. So a machine set up for the PX4 integration
cannot run the Classic simulation, and this port is what it runs instead. A
machine doing navigation work keeps Classic and never needs any of this.

Both launch files are driven from the same `hege.urdf.xacro`, so the vehicle
is described once either way.

## What did NOT change

This is the important part. `hege_localization` and `hege_navigation` were not
touched. The topics the simulator presents to ROS are identical:

| Topic | Type | Produced by |
| --- | --- | --- |
| `/imu/data` | `sensor_msgs/Imu` | gz IMU sensor, via `ros_gz_bridge` |
| `/gps/fix` | `sensor_msgs/NavSatFix` | gz NavSat sensor, via `ros_gz_bridge` |
| `/ackermann_steering_controller/odometry` | `nav_msgs/Odometry` | the same controller, via `gz_ros2_control` |
| `/cmd_vel` | `geometry_msgs/Twist` | `cmd_vel_relay`, unchanged |
| `/clock` | `rosgraph_msgs/Clock` | bridged from Gazebo |

The Ackermann controller, `controllers.yaml`, the dual EKF, `navsat_transform`
and the whole Nav2 configuration are the same files doing the same job. What
changed is underneath them.

## What did change

**The xacro grew a `gz` mode.** `hege.urdf.xacro` now has four `drive` modes:
`planar` and `ros2_control` (Classic), `gz` (Harmonic, ROS-driven) and `px4`
(Harmonic, PX4-driven). The vehicle geometry, masses, inertias and sensor noise
are shared by all four — there is still exactly one description of this robot.
The `ros2_control` joint list is shared between the `ros2_control` and `gz`
modes too; only the hardware plugin name differs, so the two cannot drift apart
in which joints they expose.

**Sensors publish differently.** A Classic sensor carried a ROS plugin that
published straight to a ROS topic. A gz sensor publishes on gz-transport and
`ros_gz_bridge` carries it across, so the `<topic>` in the URDF and the bridge
arguments in `spawn_hege_gz.launch.py` have to agree. The GPS sensor type is now
called `navsat`, not `gps`.

**The world loads its own systems.** Harmonic loads nothing by default.
`hege_field_gz.world` explicitly loads `gz-sim-imu-system` and
`gz-sim-navsat-system`; without them the sensors exist and silently publish
nothing. `hege_field.world` next to it is the Classic original, kept for the
Classic launch.

**`heading_deg` went from 180 to 0.** The Classic world carried a
`heading_deg: 180` that compensated for Classic mirroring the local frame when
converting to spherical coordinates — a rover driving east was reported as
moving west. Its comment said to recheck the value whenever the simulator
changed, and that if the bug were ever fixed the compensation would become the
bug. Harmonic does not have the mirroring, so the Harmonic world uses 0.

**The GPS sensor changed units, and nothing says so.** Gazebo Classic's `<gps>`
sensor takes horizontal position noise in **metres**. Harmonic's `<navsat>`
sensor applies the same field straight to `Latitude().Degree()` and
`Longitude().Degree()` — it is in **degrees**. Carrying the value across
unchanged asked for 0.02 degrees of error, which is 2.2 km, and neither
simulator warns: the fix simply becomes noise.

The xacro now divides by `metres_per_latitude_degree` in the `gz` branch and
leaves the Classic branch alone. This was found by Oğuzhan Enes Işık in
oguzissik/hege_gps_navigation, and `src/hege_description/test/` has regression
tests for it — including one that expands both branches and checks they
describe the same error in metres.

A consequence that follows from the same fact: equal *angular* noise on both
axes is not equal *metric* noise, because one degree of longitude is
cos(latitude) fewer metres than one of latitude. At this site the east error is
about 0.61 of the north one.

**GPS covariance had to be added.** Gazebo publishes the fix with
`position_covariance` all zeros and `COVARIANCE_TYPE_UNKNOWN`.
`navsat_transform` carries that into `/odometry/gps`, and the global EKF uses
it to decide how far to move towards each fix — so it was guessing. The bridge
now publishes to `/gps/fix_raw` and `hege_evaluation`'s `sim_gps_covariance`
republishes it on `/gps/fix` with the covariance the noise settings imply,
anisotropy included. The same package's `gps_noise_evaluator` scores the fix
against ground truth and reports on `/hege/evaluation/gps_noise`, which is how
you find out the GPS is not the error you configured.

Worth checking on the Classic path too: whether `libgazebo_ros_gps_sensor`
fills the covariance is not something this port established. `ros2 topic echo
/gps/fix --once` and look at `position_covariance_type` — if it is 0, the
default simulation has the same gap.

**Ground truth moved.** Classic's `gazebo_ros_state` plugin and `/model_states`
have no Harmonic equivalent. The `gz` branch of the URDF runs
`gz-sim-odometry-publisher-system` instead, bridged to `/hege/ground_truth/odom`.
`localization_monitor.py` listens for both and uses whichever arrives, and its
`gazebo_msgs` import is now optional so it still starts on a machine without
the Classic messages.

**The files, and which simulator each belongs to:**

| Classic | Harmonic |
| --- | --- |
| `launch/spawn_hege.launch.py` | `launch/spawn_hege_gz.launch.py` |
| `worlds/hege_field.world` | `worlds/hege_field_gz.world` |
| `drive:=planar`, `drive:=ros2_control` | `drive:=gz`, `drive:=px4` |

Nothing above the simulator changes between them:

    ros2 launch hege_description spawn_hege.launch.py      # or _gz
    ros2 launch hege_localization localization.launch.py
    ros2 launch hege_navigation navigation.launch.py rviz:=true

## What to install

This section is only for the Harmonic path. For the Classic default,
`hege_description/package.xml` declares what it needs and `rosdep install`
works normally.

`package.xml` deliberately does **not** declare the Harmonic packages, because
rosdep would then uninstall Classic to satisfy them on a Classic machine. So
they have to be installed deliberately, and they come from two different
places.

**`ros_gz` — from apt.** The OSRF repository, which is already configured if
`gz-harmonic` was installed from it, ships Harmonic builds under a suffixed
Debian name. The ROS package names inside are still the usual ones:

    sudo apt install ros-humble-ros-gzharmonic-sim ros-humble-ros-gzharmonic-bridge

    ros2 pkg prefix ros_gz_sim ros_gz_bridge     # both must print a path

**`gz_ros2_control` — from source.** There is a
`ros-humble-gz-ros2-control` on packages.ros.org, and it is the wrong one:

    apt show ros-humble-gz-ros2-control | grep ^Depends
    -> libsdformat12

`libsdformat12` is Gazebo Fortress. Harmonic is `libsdformat14`. The plugin
will not load into `gz-sim8`, so this package has to be built from source
against Harmonic, at the revision `EXTERNAL_DEPS.txt` pins:

    gz_ros2_control: humble c88a5fd

In its own workspace, next to this one:

    cd <workspace>
    mkdir -p gz_ros2_control_ws/src && cd gz_ros2_control_ws/src
    git clone -b humble https://github.com/ros-controls/gz_ros2_control.git
    git -C gz_ros2_control checkout c88a5fd
    cd ..
    source /opt/ros/humble/setup.bash
    export GZ_VERSION=harmonic
    colcon build --symlink-install

`GZ_VERSION=harmonic` is what selects the Harmonic libraries; without it the
build picks Fortress if it is present and fails if it is not. Do **not** run
`rosdep install` in that workspace — it would resolve the Gazebo dependencies
to Fortress, which is the whole problem. The Harmonic development headers it
needs (`libgz-sim8-dev`, `libsdformat14-dev`) come with `gz-harmonic`.

Then source it before this workspace, in every terminal:

    source gz_ros2_control_ws/install/setup.bash
    source install/setup.bash

`spawn_hege_gz.launch.py` takes care of `GZ_SIM_SYSTEM_PLUGIN_PATH` itself, by
looking up the `gz_ros2_control` prefix through ament. That matters because the
failure when it is missing is quiet: Gazebo starts, the world loads, the model
appears, and then every controller spawner sits waiting for a
`/controller_manager` that was never created.

**Checking for damage before any of this.** Given how easily one Gazebo removes
another, simulate first. Nothing is changed by:

    sudo apt install -s <packages> 2>&1 | grep -E '^(Remv|REMOV)'

Any line in that output means apt intends to remove something. Read it before
continuing.

## Checking it works

    ros2 launch hege_description spawn_hege_gz.launch.py

    ros2 control list_controllers      # joint_state_broadcaster,
                                       # ackermann_steering_controller, both active
    ros2 topic hz /imu/data            # about 100 Hz
    ros2 topic echo /gps/fix --once    # latitude near 52.466, longitude near 12.958
    ros2 topic echo /clock --once      # non-zero, and advancing

Then drive it and check the coordinate convention, which is the one thing most
likely to be wrong:

    ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.5}, angular: {z: 0.0}}"

Facing the default yaw of 0 the rover drives EAST, so the **longitude reported
on `/gps/fix` must increase**. If it decreases, Harmonic has the same mirroring
Classic had and `heading_deg` in `hege_field_gz.world` goes back to 180.

## Status

The port has not been run. It was written and checked statically: all four
`drive` modes expand through xacro with no cross-contamination, the `gz` output
carries the gz sensors with the right topics and the preserved fixed joints,
and every file parses. Nothing has been launched, no controller has been
spawned and no rover has moved.

The measured numbers in `nav2_params.yaml` and `dual_ekf_navsat.yaml` — the
turning radii, the lookahead distances, the 0.08 m localization error — all
come from Gazebo Classic runs. Harmonic uses a different physics engine
(DART rather than ODE), so the tyre behaviour and therefore the achievable
turning radius may genuinely differ. Expect to re-measure them here rather
than assuming they carry over. That, and the fact that Classic is where the
work was actually done, is why Classic remains the default.
