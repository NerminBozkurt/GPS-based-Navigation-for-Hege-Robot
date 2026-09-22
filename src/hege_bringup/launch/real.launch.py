"""
Real rover bring-up: MicroXRCEAgent over Pixhawk Ethernet + bridge + sensors.

    ROS_DOMAIN_ID=73 ros2 launch hege_bringup real.launch.py

The established HEGE topology keeps TELEM2 for MAVLink/mavlink-router/QGC and
uses Ethernet UDP port 8888 for uXRCE-DDS. Never reassign TELEM2 to DDS.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_agent = LaunchConfiguration("start_agent")
    agent_port = LaunchConfiguration("agent_port")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    bridge_params = LaunchConfiguration("bridge_params")

    sensors_params = PathJoinSubstitution([FindPackageShare("hege_px4_sensors"), "config", "sensors.yaml"])
    mux_params = PathJoinSubstitution([FindPackageShare("hege_bringup"), "config", "twist_mux.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("start_agent", default_value="true"),
        DeclareLaunchArgument("agent_port", default_value="8888", description="UDP port configured in PX4 UXRCE_DDS_PRT"),
        DeclareLaunchArgument("ros_domain_id", default_value="73", description="must equal PX4 UXRCE_DDS_DOM_ID"),
        DeclareLaunchArgument("bridge_params", default_value=PathJoinSubstitution([FindPackageShare("hege_px4_bridge"), "config", "bridge_real.yaml"])),
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),

        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
            output="screen",
            condition=IfCondition(start_agent),
        ),

        Node(package="twist_mux", executable="twist_mux", name="twist_mux",
             parameters=[mux_params], remappings=[("cmd_vel_out", "/cmd_vel/selected")],
             output="screen"),

        Node(package="hege_px4_bridge", executable="bridge", name="hege_px4_bridge",
             parameters=[bridge_params], output="screen"),

        Node(package="hege_px4_sensors", executable="sensors", name="hege_px4_sensors",
             parameters=[sensors_params], output="screen"),
    ])
