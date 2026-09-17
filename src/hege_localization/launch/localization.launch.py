"""Dual-EKF + navsat localization for Hege.

Run alongside the simulation:

    ros2 launch hege_description spawn_hege.launch.py
    ros2 launch hege_localization localization.launch.py

Publishes:
    /odometry/filtered      local  EKF, odom -> base_footprint  (smooth)
    /odometry/filtered_map  global EKF, map  -> odom            (drift-free)
    /odometry/gps           navsat_transform, GPS as a metric map-frame pose
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('hege_localization'),
        'config', 'dual_ekf_navsat.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Local filter. Owns odom -> base_footprint. No GPS on purpose.
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    # Global filter. Owns map -> odom. GPS lands here so its jumps stay off
    # the odom edge.
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered_map')],
    )

    # Feeds ekf_global. Note it consumes the GLOBAL filter's output, not the
    # local one - it needs the map-frame estimate to place each fix.
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/filtered_map'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the /clock topic published by Gazebo.'),
        ekf_local,
        ekf_global,
        navsat_transform,
    ])
