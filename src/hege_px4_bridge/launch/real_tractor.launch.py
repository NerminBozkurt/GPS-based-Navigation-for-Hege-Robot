"""Complete bringup launch file for the physical Hege tractor with PX4 and Nav2.

Usage:
    ros2 launch hege_px4_bridge real_tractor.launch.py
"""

import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    desc_pkg = get_package_share_directory('hege_description')
    bridge_pkg = get_package_share_directory('hege_px4_bridge')
    nav_pkg = get_package_share_directory('hege_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # Load and process URDF for robot_state_publisher (no Gazebo)
    xacro_file = os.path.join(desc_pkg, 'urdf', 'hege.urdf.xacro')
    robot_description_xml = xacro.process_file(
        xacro_file, mappings={'drive': 'ros2_control'}
    ).documentElement.toxml()
    robot_description_xml = re.sub(r'<!--.*?-->', '', robot_description_xml, flags=re.DOTALL)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_xml, value_type=str),
            'use_sim_time': False
        }],
    )

    # PX4 Bridge Node
    bridge_params = os.path.join(bridge_pkg, 'config', 'px4_bridge_params.yaml')
    px4_bridge_node = Node(
        package='hege_px4_bridge',
        executable='px4_bridge_node',
        name='hege_px4_bridge',
        output='screen',
        parameters=[
            bridge_params,
            {'use_sim_time': False}
        ],
    )

    # Nav2 Navigation stack (planning, control, recovery, behavior tree)
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_nav2'))
    )

    return [
        robot_state_publisher,
        px4_bridge_node,
        nav2_bringup_cmd,
    ]


def generate_launch_description():
    domain_id_arg = DeclareLaunchArgument(
        'domain_id',
        default_value='73',
        description='ROS_DOMAIN_ID matching MicroXRCE-DDS agent on PX4'
    )

    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='Whether to launch Nav2 stack alongside the bridge'
    )

    set_domain = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=LaunchConfiguration('domain_id')
    )

    return LaunchDescription([
        domain_id_arg,
        launch_nav2_arg,
        set_domain,
        OpaqueFunction(function=launch_setup),
    ])
