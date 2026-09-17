# GPS based Navigation for Hege Robot

GPS waypoint navigation for the Hege Ackermann field robot, developed for the
Erasmus BIP Robotics Bootcamp 2026 (TU Berlin / ATB Fieldlab Marquardt).

The goal is a navigation stack that is developed and validated entirely in
simulation, independent of the target hardware, and then deployed onto the real
robot with GPS as the only localization source.

## Architecture

Everything above the flight controller runs as ROS 2 on the robot's onboard
computer; the Pixhawk is treated as a smart motor driver fed by `/cmd_vel`.

    GPS ─┐
    IMU ─┼──> robot_localization (dual EKF) ──> Nav2 ──> /cmd_vel ──> drivetrain
    odom ┘

The same controller and navigation configuration is intended to run unchanged on
the real robot; only the hardware interface beneath `ros2_control` is swapped.

## Packages

| Package | Contents |
| --- | --- |
| `hege_description` | URDF/xacro model, Gazebo world, `ros2_control` hardware interface, Ackermann controller configuration, simulation bringup |
| `hege_localization` | Dual-EKF and `navsat_transform` configuration, localization bringup, live estimate-vs-truth monitor |

## Requirements

ROS 2 Humble, Gazebo Classic 11, and:

    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
    ros-humble-gazebo-ros2-control ros-humble-ackermann-steering-controller
    ros-humble-joint-state-broadcaster ros-humble-robot-localization
    ros-humble-teleop-twist-keyboard

## Build

    colcon build --symlink-install
    source install/setup.bash

## Run

Simulation:

    ros2 launch hege_description spawn_hege.launch.py

Localization:

    ros2 launch hege_localization localization.launch.py

Live comparison of every estimate against Gazebo ground truth:

    ros2 run hege_localization localization_monitor.py

Keyboard driving (hold the key; one keypress sends a single message):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -p speed:=1.5 -p turn:=0.3

`i` forward, `u`/`o` turn while moving, `,` reverse, `k` stop. The rover is
Ackermann-steered, so `j` and `l` alone do nothing - they command rotation at
zero forward speed, which a car-like vehicle cannot do.

## Status

Working: model with measured dimensions, Ackermann drive and steering, GPS/IMU
sensing, dual-EKF fusion tracking ground truth to within about 0.1 m.

Not started: Nav2 bringup, GPS waypoint following, deployment to hardware.
