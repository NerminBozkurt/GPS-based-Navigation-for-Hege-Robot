# GPS based Navigation for Hege Robot

GPS waypoint navigation for the Hege Ackermann field robot, developed for the
Erasmus BIP Robotics Bootcamp 2026.

The goal is a navigation stack that is developed and validated entirely in
simulation, independent of the target hardware, and then deployed onto the real
robot with GPS as the only localization source.

## This branch

This is an alternative navigation stack for the same robot, developed
independently by Michal Czaplinski. It diverged early from
[`main`](../../tree/main), which is the maintained line with the fuller
README, and does not merge back into it. Kept here as a reference, not under
further development.

What sets it apart:

- **`nav2_smac_planner/SmacPlannerHybrid`** with `motion_model_for_search:
  DUBIN` as the global planner, instead of `main`'s default Nav2 planner —
  forward-only paths, no reversing.
- **`BaleDetection`**, in `hege_navigation/scripts/gps_waypoint_navigator.py`
  — lays out a hay-bale collection mission from a fixed set of bale positions
  and generates the approach waypoints and RViz/Gazebo markers around them.
- **Its own `hege_px4_bridge`**, built for a real tractor running PX4 1.16,
  with `px4_msgs` vendored under `src/` rather than cloned externally. It is
  a separate implementation from `main`'s `hege_px4_bridge`, and this branch
  has no equivalent of `main`'s `hege_px4_sensors`, `hege_bringup` or
  `hege_evaluation`.
