"""Everything except the part that can move the rover.

    ROS_DOMAIN_ID=73 ros2 launch hege_bringup hege_real_dry_run.launch.py rviz:=true

This is hege_real.launch.py with hege_px4_bridge replaced by hege_px4_bridge's
dry_run node. Give a 2D Goal Pose in RViz and watch the velocities and the
steering angle Nav2 produces, with nothing reaching the vehicle - because the
node that talks to PX4 is not running.

Why a separate file rather than just not arming: not arming is already safe,
and hege_real.launch.py is built around that. But the bridge refuses to start
until px4_yaw_p is set to the tuned RO_YAW_P, which is a measurement you do not
have yet on the first day. That refusal is correct - and it also means
hege_real.launch.py will not come up until the tuning is done. This file lets
the whole navigation stack be tested against the real Pixhawk, the real GPS and
the real compass before that measurement exists, which is the right order: find
out whether the rover knows where it is and plans a sane route, then tune the
gain, then let it drive.

What runs, and what does not:

    MicroXRCEAgent, hege_px4_sensors    yes - reading the Pixhawk
    robot_state_publisher, EKF, Nav2    yes - the whole decision chain
    twist_mux                           yes - so the priorities are exercised
    hege_px4_bridge                     NO  - nothing publishes to /fmu/in/*
    hege_px4_bridge dry_run             yes - reports what the bridge would send

Watch, from a second sourced terminal:

    ros2 topic echo /hege/dry_run/report        what the wheels would be told
    ros2 topic echo /cmd_vel/nav                what Nav2 decided
    ros2 run tf2_ros tf2_echo map base_footprint

The dry-run node reads its limits from the same bridge_real.yaml the bridge
would use, so the numbers it reports are the ones that will apply later. It
does not need px4_yaw_p: that gain only affects HOW the bridge asks PX4 for a
yaw rate, not what yaw rate is asked for.

SAFETY. This launch cannot command PX4, but it is not a substitute for the
physical emergency stop or for RC takeover being in someone's hands. Treat the
vehicle as live whenever it is powered.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
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
    nav2_overlay = os.path.join(bringup, 'config', 'nav2_real_overlay.yaml')

    start_agent = LaunchConfiguration('start_agent')
    agent_port = LaunchConfiguration('agent_port')
    bridge_params = LaunchConfiguration('bridge_params')
    rviz = LaunchConfiguration('rviz')

    # Wall time on this side; there is no /clock behind a Pixhawk.
    use_sim_time = 'false'

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

    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel/selected')],
        output='screen',
    )

    # The bridge's seat, with the output disconnected.
    dry_run = Node(
        package='hege_px4_bridge', executable='dry_run',
        name='hege_dry_run',
        parameters=[{'bridge_params': bridge_params,
                     'use_sim_time': use_sim_time}],
        output='screen',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_topic': '/px4/odom',
            'use_datum': 'false',
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': '/cmd_vel/nav',
            'params_overlay': nav2_overlay,
            'bt_variant': 'px4',
            'rviz': rviz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_agent', default_value='true',
            description='Start MicroXRCEAgent here. false if one already runs.'),
        DeclareLaunchArgument(
            'agent_port', default_value='8888',
            description='UDP port configured in PX4 UXRCE_DDS_PRT.'),
        DeclareLaunchArgument(
            'ros_domain_id', default_value='73',
            description='Must equal PX4 UXRCE_DDS_DOM_ID.'),
        DeclareLaunchArgument(
            'bridge_params',
            default_value=os.path.join(bridge_share, 'config',
                                       'bridge_real.yaml'),
            description='Read for its limits only. px4_yaw_p may still be 0 '
                        'here, unlike in hege_real.launch.py.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open RViz. Defaults true here, because giving a goal '
                        'and watching the result is the entire purpose.'),

        SetEnvironmentVariable('ROS_DOMAIN_ID',
                               LaunchConfiguration('ros_domain_id')),

        robot_state_publisher,
        joint_state_publisher,
        agent,
        sensors,
        twist_mux,
        dry_run,
        localization_launch,
        navigation_launch,
    ])
