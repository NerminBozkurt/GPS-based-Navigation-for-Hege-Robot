"""
Simulation bring-up:  MicroXRCEAgent (UDP)  +  hege_px4_bridge  +  hege_px4_sensors

PX4 SITL + Gazebo must be started separately (it is a different process tree):

    cd PX4-Autopilot            # pinned to v1.16.1
    make px4_sitl gz_rover_ackermann

Then:
    ros2 launch hege_bringup sim.launch.py
    ros2 launch hege_bringup sim.launch.py step_test:=true      # scripted slow drive
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
    step_test = LaunchConfiguration("step_test")
    ros_domain_id = LaunchConfiguration("ros_domain_id")

    bridge_params = PathJoinSubstitution([FindPackageShare("hege_px4_bridge"), "config", "bridge_sim.yaml"])
    sensors_params = PathJoinSubstitution([FindPackageShare("hege_px4_sensors"), "config", "sensors.yaml"])
    mux_params = PathJoinSubstitution([FindPackageShare("hege_bringup"), "config", "twist_mux.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("start_agent", default_value="true",
                              description="start MicroXRCEAgent udp4 here"),
        DeclareLaunchArgument("agent_port", default_value="8888",
                              description="UDP port PX4 SITL connects to (PX4 default 8888)"),
        DeclareLaunchArgument("step_test", default_value="false",
                              description="also run the scripted /cmd_vel step test"),
        DeclareLaunchArgument("ros_domain_id", default_value="0",
                              description="must equal PX4 SITL UXRCE_DDS_DOM_ID"),

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

        Node(package="hege_px4_bridge", executable="step_test", name="hege_step_test",
             parameters=[{
                 "confirm_motion_test": True,
                 # The PX4 SITL rover has L=0.321 m. omega=0.20 rad/s gives
                 # about 12 degrees steering, which is easy to see in Gazebo.
                 "phases": [
                     "3.0,0.0,0.0",
                     "5.0,0.3,0.0",
                     "5.0,0.3,0.2",
                     "5.0,0.3,-0.2",
                     "3.0,0.0,0.0",
                 ],
             }],
             output="screen", condition=IfCondition(step_test)),
    ])
