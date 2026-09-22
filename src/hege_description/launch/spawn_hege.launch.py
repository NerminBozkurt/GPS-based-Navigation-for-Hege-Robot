"""Bring up Gazebo Harmonic with the Hege rover spawned in it.

    ros2 launch hege_description spawn_hege.launch.py
    ros2 launch hege_description spawn_hege.launch.py gui:=false   # headless

This is the simulation the navigation stack runs against:

    ros2 launch hege_description spawn_hege.launch.py
    ros2 launch hege_localization localization.launch.py
    ros2 launch hege_navigation navigation.launch.py rviz:=true

It used to be Gazebo Classic. spawn_hege_classic.launch.py is that file, kept
unchanged and still working on a machine that has Classic installed - but the
two cannot be installed together, because their Debian packages both ship
/usr/bin/gz and conflict outright. PX4 SITL requires Harmonic, so this side
moved to Harmonic as well rather than making the machine choose. The reasoning
and the migration notes are in docs/gazebo_harmonic.md.

What ROS sees is deliberately unchanged: /imu/data, /gps/fix, the Ackermann
controller's odometry and /cmd_vel all carry the same messages on the same
topics as before, so hege_localization and hege_navigation did not have to
change at all. The difference is that a gz sensor publishes on gz-transport
and ros_gz_bridge carries it across, instead of a Gazebo plugin publishing to
ROS directly.
"""

import os

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('hege_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_share, 'urdf', 'hege.urdf.xacro')

    # Expanded here rather than through a Command substitution because the
    # controller manager needs the same string and the xacro takes a mapping.
    # Unlike the Classic file this needs no comment stripping: gz_ros2_control
    # reads robot_description off the parameter server rather than passing it
    # through a command line that rcl then parses as YAML.
    robot_description_xml = xacro.process_file(
        xacro_file, mappings={'drive': 'gz'}).documentElement.toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')

    # -r starts the world unpaused; -s is server only. Without -r the
    # controllers spawn against a simulator whose clock never advances, and
    # every spawner times out waiting for the controller manager.
    gz_args = '-r -v 3 ' + world if gui else '-r -s -v 3 ' + world

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_xml, value_type=str),
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'hege',
            '-allow_renaming', 'false',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # The seam between the two middlewares. Everything ROS consumes from the
    # simulator crosses here, and the type on each side has to match what the
    # sensor actually publishes - a wrong type is not an error, it is a topic
    # that exists and stays empty.
    #
    # All four are gz -> ROS only ('[') because nothing in the stack commands
    # the simulator through gz-transport; /cmd_vel goes to the Ackermann
    # controller through ros2_control instead.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # Without this every node with use_sim_time waits forever.
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            # Ground truth for localization_monitor.py. Replaces Classic's
            # /model_states. Nothing in the navigation stack may consume it.
            '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Controllers can only be spawned once gz_ros2_control has come up with the
    # model, which happens during the create call - hence the chaining rather
    # than starting everything at once.
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

    return [
        gz_sim,
        robot_state_publisher,
        bridge,
        spawn_entity,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[ackermann_controller, cmd_vel_relay])),
    ]


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory('hege_description'), 'worlds',
        'hege_field_gz.world')
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Run the Gazebo GUI as well as the server.'),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Full path to the Harmonic world file to load. '
                        'hege_field.world next to it is the Classic original '
                        'and will not load here.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the /clock topic bridged from Gazebo.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        OpaqueFunction(function=launch_setup),
    ])
