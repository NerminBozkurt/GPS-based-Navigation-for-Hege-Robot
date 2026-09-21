# GPS based Navigation for Hege Robot

GPS waypoint navigation for the Hege Ackermann field robot, developed for the
Erasmus BIP Robotics Bootcamp 2026.

The goal is a navigation stack that is developed and validated entirely in
simulation, independent of the target hardware, and then deployed onto the real
robot with GPS as the only localization source.

`docs/px4_ros2_topics.md` records the ROS 2 interface the Pixhawk actually
exposes on the real robot, and how it lines up with the simulation stack.
