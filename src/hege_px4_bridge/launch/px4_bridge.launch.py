"""Launch file for hege_px4_bridge."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hege_px4_bridge')
    default_params_file = os.path.join(pkg_dir, 'config', 'px4_bridge_params.yaml')

    domain_id_arg = DeclareLaunchArgument(
        'domain_id',
        default_value='73',
        description='ROS_DOMAIN_ID matching MicroXRCE-DDS agent on PX4'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (false for physical robot)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to bridge configuration parameters'
    )

    set_domain_env = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=LaunchConfiguration('domain_id')
    )

    bridge_node = Node(
        package='hege_px4_bridge',
        executable='px4_bridge_node',
        name='hege_px4_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return LaunchDescription([
        domain_id_arg,
        use_sim_time_arg,
        params_file_arg,
        set_domain_env,
        bridge_node,
    ])
