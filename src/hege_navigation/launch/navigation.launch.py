"""Nav2 bringup for Hege.

Add rviz:=true to see what Nav2 is doing - the planned route, the local costmap,
the fused pose estimate and the waypoints - next to Gazebo's view of the world.

Runs on top of the simulation and the localization stack:

    ros2 launch hege_description spawn_hege.launch.py
    ros2 launch hege_localization localization.launch.py
    ros2 launch hege_navigation navigation.launch.py

There is no map server and no AMCL. This is GPS navigation across open ground:
the map frame comes from robot_localization's global EKF, and both costmaps are
rolling windows with no static layer.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Started in this order and managed as one group by the lifecycle manager.
NODES = [
    ('controller_server', 'nav2_controller', 'controller_server'),
    ('smoother_server', 'nav2_smoother', 'smoother_server'),
    ('planner_server', 'nav2_planner', 'planner_server'),
    ('behavior_server', 'nav2_behaviors', 'behavior_server'),
    ('bt_navigator', 'nav2_bt_navigator', 'bt_navigator'),
    ('waypoint_follower', 'nav2_waypoint_follower', 'waypoint_follower'),
    ('velocity_smoother', 'nav2_velocity_smoother', 'velocity_smoother'),
]


def generate_launch_description():
    share = get_package_share_directory('hege_navigation')
    params = os.path.join(share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(share, 'rviz', 'navigation.rviz')

    # Absolute paths, resolved here. bt_navigator needs a real path and a
    # parameter file cannot expand $(find-pkg-share ...) on its own.
    bt_trees = {
        'default_nav_to_pose_bt_xml': os.path.join(
            share, 'behavior_trees', 'navigate_to_pose_ackermann.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(
            share, 'behavior_trees', 'navigate_through_poses_ackermann.xml'),
    }

    use_sim_time = LaunchConfiguration('use_sim_time')

    nodes = []
    for name, package, executable in NODES:
        remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
        # The smoother sits between Nav2 and the rover, and BOTH ends of it have
        # to be wired or the chain is silently broken:
        #
        #   controller_server  cmd_vel        -> cmd_vel_nav
        #   velocity_smoother  cmd_vel        <- cmd_vel_nav   (its input)
        #   velocity_smoother  cmd_vel_smoothed -> cmd_vel     (its output)
        #
        # /cmd_vel is what cmd_vel_relay listens to. Remapping only the
        # controller leaves the smoother subscribed to a topic nobody
        # publishes, so Nav2 plans happily and the rover never moves.
        if name == 'controller_server':
            remaps.append(('cmd_vel', 'cmd_vel_nav'))
        elif name == 'velocity_smoother':
            remaps += [('cmd_vel', 'cmd_vel_nav'),
                       ('cmd_vel_smoothed', 'cmd_vel')]
        extra = dict(bt_trees) if name == 'bt_navigator' else {}
        extra['use_sim_time'] = use_sim_time
        nodes.append(Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[params, extra],
            remappings=remaps,
        ))

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [name for name, _, _ in NODES],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the /clock topic published by Gazebo.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also open RViz showing the plan, costmaps, pose '
                        'estimate and waypoints.'),
    ] + nodes + [lifecycle_manager, rviz])
