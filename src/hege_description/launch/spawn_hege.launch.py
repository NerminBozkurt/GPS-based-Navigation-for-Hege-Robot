"""Bring up Gazebo with the Hege rover spawned in it.

    ros2 launch hege_description spawn_hege.launch.py
    ros2 launch hege_description spawn_hege.launch.py gui:=false   # headless
"""

import os
import re

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('hege_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'hege.urdf.xacro')

    # Processed here rather than via a Command substitution on purpose (and it
    # is what lets the drive mode be a xacro mapping at all).
    # In ros2_control mode, gazebo_ros2_control forwards robot_description to controller_manager as a
    # command-line override, '--param robot_description:=<urdf>', and rcl parses
    # that value as YAML. Two things in a normal xacro output break it:
    #   - the leading <?xml ... ?> declaration
    #   - any ': ' sequence, which YAML reads as the start of a mapping
    # documentElement.toxml() drops the declaration, and stripping XML comments
    # removes the ': ' sequences (they only ever appear in prose comments here).
    # Without both, controller_manager never starts and the controller spawners
    # wait forever on /controller_manager/list_controllers.
    drive = LaunchConfiguration('drive').perform(context)
    # Everything the xacro exposes as an <arg> is forwarded here, so a run can
    # be degraded from the command line without touching the model.
    mappings = {name: LaunchConfiguration(name).perform(context)
                for name in ('drive', 'gps_noise', 'imu_gyro_bias',
                             'imu_accel_bias', 'imu_gyro_noise',
                             'imu_accel_noise')}
    robot_description_xml = xacro.process_file(
        xacro_file, mappings=mappings).documentElement.toxml()
    robot_description_xml = re.sub(r'<!--.*?-->', '', robot_description_xml,
                                   flags=re.DOTALL)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # gzserver carries the ROS init + factory plugins; spawn_entity needs the
    # factory one to exist or the service call below never appears.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'verbose': 'true'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Expands the xacro at launch time, so editing the URDF needs no rebuild
    # (the package is installed with --symlink-install).
    robot_description = ParameterValue(robot_description_xml, value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hege',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # Controllers can only be spawned once gazebo_ros2_control has come up
    # with the model, which happens during spawn_entity - hence the chaining
    # below rather than starting everything at once.
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # The controller listens on ~/reference_unstamped; exposing it as plain
    # /cmd_vel lets Nav2 and teleop_twist_keyboard drive it unmodified.
    cmd_vel_relay = Node(
        package='hege_description',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    nodes = [gzserver, gzclient, robot_state_publisher, spawn_entity]

    # In planar mode the Gazebo plugin takes /cmd_vel straight from teleop, so
    # there is no controller_manager to talk to and nothing to relay.
    if drive == 'ros2_control':
        nodes += [
            RegisterEventHandler(OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster])),
            RegisterEventHandler(OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[ackermann_controller, cmd_vel_relay])),
        ]
    return nodes


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory('hege_description'), 'worlds', 'hege_field.world')
    return LaunchDescription([
        DeclareLaunchArgument(
            'drive', default_value='ros2_control',
            choices=['planar', 'ros2_control'],
            description='planar = teleop-friendly Gazebo plugin; '
                        'ros2_control = real Ackermann controller.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Run the Gazebo client GUI as well as the server.'),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Full path to the .world file to load.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the /clock topic published by Gazebo.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Sensor realism. Defaults match the model's own defaults; override to
        # degrade a run, e.g. gps_noise:=1.5 for a single-point GNSS solution.
        DeclareLaunchArgument(
            'gps_noise', default_value='0.02',
            description='GPS horizontal stddev in metres. 0.02 RTK-fixed, '
                        '~0.4 RTK float, ~2.0 single-point.'),
        DeclareLaunchArgument(
            'imu_gyro_bias', default_value='7.5e-6',
            description='Gyro bias, rad/s. Zero gives an unrealistically good IMU.'),
        DeclareLaunchArgument(
            'imu_accel_bias', default_value='0.05',
            description='Accelerometer bias, m/s^2.'),
        DeclareLaunchArgument(
            'imu_gyro_noise', default_value='2e-4',
            description='Gyro white noise stddev per sample, rad/s.'),
        DeclareLaunchArgument(
            'imu_accel_noise', default_value='1.7e-2',
            description='Accelerometer white noise stddev per sample, m/s^2.'),

        OpaqueFunction(function=launch_setup),
    ])
