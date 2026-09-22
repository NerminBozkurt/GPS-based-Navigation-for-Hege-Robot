# Moving the simulation to Gazebo Harmonic

The ROS-driven simulation used to run on Gazebo Classic. It now runs on Gazebo
Harmonic. This is why, what changed, and what to install.

## Why

Gazebo Classic and Gazebo Harmonic **cannot be installed on the same machine**.
Their Debian packages both ship `/usr/bin/gz` and conflict outright:

    gz-tools2 : Conflicts: gazebo (>= 11.0.0)
                Conflicts: gazebo (<= 11.14.0)

`apt install gz-harmonic` therefore removes `gazebo`, `ros-humble-gazebo-ros`,
`ros-humble-gazebo-plugins`, `ros-humble-gazebo-ros2-control` and everything
else in the Classic ROS stack, without asking twice.

PX4 v1.16 SITL requires Harmonic. So the choice was never "which simulator do
we prefer" but "which one machine can have". Keeping the navigation simulation
on Classic would have meant a machine that could run either the path planning
or the PX4 integration, never both, with a gigabyte of apt churn between them.
Moving this side to Harmonic makes one machine run everything.

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
arguments in `spawn_hege.launch.py` have to agree. The GPS sensor type is now
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

**Ground truth moved.** Classic's `gazebo_ros_state` plugin and `/model_states`
have no Harmonic equivalent. The `gz` branch of the URDF runs
`gz-sim-odometry-publisher-system` instead, bridged to `/ground_truth/odom`.
`localization_monitor.py` listens for both and uses whichever arrives, and its
`gazebo_msgs` import is now optional so it still starts on a machine without
the Classic messages.

**Files that moved:**

| Before | Now |
| --- | --- |
| `launch/spawn_hege.launch.py` (Classic) | `launch/spawn_hege_classic.launch.py` |
| — | `launch/spawn_hege.launch.py` (Harmonic) |
| `worlds/hege_field.world` (Classic) | unchanged, still there |
| — | `worlds/hege_field_gz.world` |

The command you already know is unchanged and now starts Harmonic:

    ros2 launch hege_description spawn_hege.launch.py

## What to install

**Read this before running `rosdep install`.** `hege_description/package.xml`
deliberately declares *neither* Gazebo generation, because rosdep would resolve
the names to the wrong packages and uninstall what is working:

- `ros_gz_sim`, `ros_gz_bridge` and `gz_ros2_control` resolve to the ROS 2
  Humble binaries, which are built against Gazebo **Fortress**. Humble's
  official Gazebo pairing is Fortress, not Harmonic. Installing them next to
  Harmonic pulls in a second, conflicting Gazebo.
- `gazebo_ros`, `gazebo_plugins` and `gazebo_ros2_control` are Classic, and
  conflict with Harmonic as described above.

So the ROS-to-Harmonic packages have to be installed deliberately, and they
come from two different places.

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

`spawn_hege.launch.py` takes care of `GZ_SIM_SYSTEM_PLUGIN_PATH` itself, by
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

    ros2 launch hege_description spawn_hege.launch.py

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
turning radius may genuinely differ. Those numbers should be re-measured on
Harmonic before they are trusted again, which is also why
`spawn_hege_classic.launch.py` is kept rather than deleted.
