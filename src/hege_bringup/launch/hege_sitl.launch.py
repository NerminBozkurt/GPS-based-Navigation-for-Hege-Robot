"""The whole stack, with PX4 SITL driving the measured Hege rover.

This is the combination the project is aiming at: our own path planning and
tracking on top of our own vehicle model, with PX4 in between instead of the
Gazebo Ackermann controller.

PX4 SITL is a separate process tree with its own build system, so it starts on
its own first (see docs/px4_sitl.md for the one-time model install):

    cd PX4-Autopilot
    make px4_sitl gz_hege_rover

Then, in a sourced terminal:

    ros2 launch hege_bringup hege_sitl.launch.py
    ros2 launch hege_bringup hege_sitl.launch.py rviz:=true

The chain this builds:

    PX4 SITL + Gazebo Harmonic
        | uXRCE-DDS
        v
    hege_px4_sensors  -> /imu/data, /gps/fix, /px4/odom
        v
    hege_localization  (dual EKF + navsat_transform, unchanged)
        v
    hege_navigation    (Nav2, unchanged apart from nav2_px4_overlay.yaml)
        v
    /cmd_vel/nav -> twist_mux -> /cmd_vel/selected
        v
    hege_px4_bridge -> /fmu/in/trajectory_setpoint

Note what is NOT here: cmd_vel_relay, the Ackermann controller, the
controller_manager and gazebo_ros2_control. In this path PX4 owns the
actuators and Gazebo Harmonic is driven by PX4 over gz-transport, not by ROS.
The Gazebo Classic simulation is untouched and still runs with
`ros2 launch hege_description spawn_hege.launch.py`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('hege_bringup')
    description = get_package_share_directory('hege_description')
    localization = get_package_share_directory('hege_localization')
    navigation = get_package_share_directory('hege_navigation')
    bridge_share = get_package_share_directory('hege_px4_bridge')

    sensors_params = os.path.join(bringup, 'config', 'sensors_hege_nav.yaml')
    mux_params = os.path.join(bringup, 'config', 'twist_mux.yaml')
    nav2_overlay = os.path.join(bringup, 'config', 'nav2_px4_overlay.yaml')
    bridge_params = LaunchConfiguration('bridge_params')

    start_agent = LaunchConfiguration('start_agent')
    agent_port = LaunchConfiguration('agent_port')
    rviz = LaunchConfiguration('rviz')

    # There is no /clock anywhere in this path. PX4 SITL runs against the wall
    # clock and the uXRCE-DDS agent re-bases PX4's microsecond timestamps onto
    # the companion's clock, so use_sim_time must be FALSE everywhere - the
    # opposite of the Gazebo stack, where every node takes Gazebo's /clock.
    # Leaving it true here makes every node block forever waiting for a topic
    # that will never be published.
    use_sim_time = 'false'

    # TF only. Gazebo gets its model from hege_px4_sim, which PX4 spawns; this
    # is the same xacro rendered for the ROS side, so base_footprint ->
    # base_link -> imu_link / gps_link are the measured heights and
    # navsat_transform can read the antenna offset off TF.
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(description, 'urdf', 'hege.urdf.xacro'),
                 ' drive:=px4']),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    # Publishes zeros for the wheel and steer joints. Nothing in the
    # navigation chain needs them - the frames that matter are all fixed
    # joints and go out on /tf_static regardless - but without it
    # robot_state_publisher warns on every cycle and the rover shows up in
    # RViz with no wheels.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', agent_port],
        output='screen',
        condition=IfCondition(start_agent),
    )

    sensors = Node(
        package='hege_px4_sensors', executable='sensors',
        name='hege_px4_sensors',
        parameters=[sensors_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    bridge = Node(
        package='hege_px4_bridge', executable='bridge',
        name='hege_px4_bridge',
        parameters=[bridge_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel/selected')],
        output='screen',
    )

    # Both included unchanged. The arguments below are the entire difference
    # between driving Gazebo and driving PX4.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # No wheel encoders exist on this side of the Pixhawk. The forward
            # velocity comes from PX4's own estimate instead.
            'odom_topic': '/px4/odom',
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # Into twist_mux at the lowest priority, so the step test and
            # teleop can both override Nav2 without stopping it.
            'cmd_vel_topic': '/cmd_vel/nav',
            # Forward-only planner and controller, no BackUp recovery.
            'params_overlay': nav2_overlay,
            'bt_variant': 'px4',
            'rviz': rviz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_agent', default_value='true',
            description='Start MicroXRCEAgent here. Set false if one is '
                        'already running.'),
        DeclareLaunchArgument(
            'agent_port', default_value='8888',
            description='UDP port PX4 SITL connects to (PX4 default 8888).'),
        DeclareLaunchArgument(
            'bridge_params',
            default_value=os.path.join(bridge_share, 'config',
                                       'bridge_hege_sitl.yaml'),
            description='Bridge limits. The default matches the Hege model '
                        'and the 4100_gz_hege_rover airframe. bridge_sim.yaml '
                        "is for PX4's own small demo rover instead."),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz with the navigation view.'),

        robot_state_publisher,
        joint_state_publisher,
        agent,
        sensors,
        twist_mux,
        bridge,
        localization_launch,
        navigation_launch,
    ])
