# GPS based Navigation for Hege Robot

GPS waypoint navigation for the Hege Ackermann field robot, developed for the
Erasmus BIP Robotics Bootcamp 2026.

The goal is a navigation stack that is developed and validated entirely in
simulation, independent of the target hardware, and then deployed onto the real
robot with GPS as the only localization source.

## Packages

| Package | What it is |
| --- | --- |
| `hege_description` | URDF/Xacro model, Gazebo world, Ackermann controller, `/cmd_vel` relay |
| `hege_localization` | Dual-EKF + `navsat_transform` state estimation |
| `hege_navigation` | Nav2 configuration, Ackermann behaviour trees, GPS waypoint follower |
| `hege_px4_bridge` | `/cmd_vel` → PX4 offboard velocity setpoints, with a safety supervisor |
| `hege_px4_sensors` | PX4 NED/FRD topics → standard ROS `Odometry`, `Imu`, `NavSatFix` |
| `hege_bringup` | `twist_mux` arbitration and the real/SITL launch files for the PX4 stack |
| `hege_px4_sim` | The Hege model and airframe that let PX4 SITL drive our vehicle |

The first three are the simulation stack. `hege_px4_bridge`, `hege_px4_sensors`
and `hege_bringup` are the PX4 side, written by Oğuzhan Enes Işık against the
real Pixhawk and PX4 SITL in
[oguzissik/hege_gps_navigation](https://github.com/oguzissik/hege_gps_navigation)
and brought in from there.

## The two ways to run it

Both are Gazebo Harmonic, on one machine, from one vehicle description.

    ros2 launch hege_description spawn_hege.launch.py
    ros2 launch hege_localization localization.launch.py
    ros2 launch hege_navigation navigation.launch.py rviz:=true

is the navigation simulation: our own Ackermann controller, our own
localization, Nav2 planning and tracking a GPS route.

    make px4_sitl gz_hege_rover                     # in PX4-Autopilot
    ros2 launch hege_bringup hege_sitl.launch.py rviz:=true

is the same navigation and localization stack with PX4 in the middle, driving
the same vehicle through the Pixhawk's own interface.

`hege.urdf.xacro` serves every one of these through its `drive` argument and
stays the single source of the vehicle's geometry.

The catch is that the first block runs Gazebo **Classic** and PX4 SITL requires
Gazebo **Harmonic**, and the two cannot be installed on the same machine —
their packages conflict outright.

So the PX4 side runs in the devcontainer in `.devcontainer/`, which carries
Harmonic, PX4's toolchain and the Micro XRCE-DDS Agent, while the host keeps
Classic and the navigation simulation. Open the repository in VS Code and run
**Dev Containers: Reopen in Container**; `post-create.sh` does the rest, and
`bash .devcontainer/setup-px4.sh` fetches PX4 when you want it.

The container also runs `spawn_hege_gz.launch.py`, the same navigation
simulation ported to Harmonic. `docs/gazebo_harmonic.md` covers that port.

## Docs

`docs/px4_ros2_topics.md` records the ROS 2 interface the Pixhawk actually
exposes on the real robot, and how it lines up with the simulation stack.

`docs/px4_bridge.md` covers the PX4 bridge and the sensor conversion: how a
`Twist` becomes an offboard setpoint, what the safety supervisor does, and the
five things that still have to be settled before Nav2 drives the real rover
through it.

`docs/px4_sitl.md` covers joining the two simulations: how the Gazebo Harmonic
model is generated from the same xacro, the sensor and link names PX4 requires,
how the existing localization and Nav2 launches are reused unchanged, and what
has and has not been verified.

`docs/gazebo_harmonic.md` covers the move from Gazebo Classic to Harmonic: why
the two cannot coexist, what changed in the description and the world, what
deliberately did not change, and what to install — read it before running
`rosdep install`.

## External dependencies

`EXTERNAL_DEPS.txt` pins the revisions the PX4 packages were developed against.
`px4_msgs` and `PX4-Autopilot` are cloned into the workspace rather than
vendored, so both are in `.gitignore`.

## Helper scripts

`scripts/collect_interface_inventory.sh` captures the node, topic, service and
action inventory of a running system; `scripts/record_discovery_bag.sh` records
every discovered topic to an mcap bag. Both write under `data/`, which is
ignored.
