"""The whole stack on the REAL Hege: Nav2 driving the rover through PX4.

    ROS_DOMAIN_ID=73 ros2 launch hege_bringup hege_real.launch.py

real.launch.py next to this file starts the agent, twist_mux, the bridge and
the sensor conversion - the plumbing. It deliberately does not start
localization or Nav2, so with it alone the rover only moves when a human sends
/cmd_vel. This file adds the layer that decides where to go.

READ THIS BEFORE RUNNING IT
---------------------------
This is the file that makes a 1300 kg machine with no obstacle sensors drive
itself. It cannot see a person, an animal or a post, and nothing in it will
stop for one.

Three things gate it, and two of them are enforced in code:

  * hege_px4_bridge refuses to start while px4_yaw_p is 0.0, which is what
    bridge_real.yaml ships. Read the tuned RO_YAW_P off QGroundControl and put
    it there first - a wrong value turns the rover at the wrong rate and
    nothing reports an error.

  * bridge_real.yaml keeps allow_remote_vehicle_commands false, so nothing
    here can arm the vehicle or change its mode. Arming stays with RC and
    QGroundControl, deliberately. Launching this file does NOT make the rover
    move; it makes it ready to move once a human arms it.

  * The third is procedure, not code: driven wheels lifted for the first run,
    a cleared area after that, RC takeover in someone's hands and the physical
    emergency stop within reach. twist_mux gives /cmd_vel/teleop priority 100
    against Nav2's 10, so a teleop command overrides the navigator instantly -
    but that is a software layer, not a substitute for the E-stop.

docs/real_vehicle.md has the ordered procedure and what to measure.

What this builds:

    Pixhawk ---(uXRCE-DDS, domain 73)--- hege_px4_sensors
                                              |  /imu/data, /gps/fix, /px4/odom
                                              v
                                    hege_localization  (dual EKF + navsat)
                                              v
                                    hege_navigation    (Nav2)
                                              v
                              /cmd_vel/nav -> twist_mux -> /cmd_vel/selected
                                              v
                                    hege_px4_bridge -> /fmu/in/trajectory_setpoint
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

    # Same file the SITL bring-up uses: it only renames topics and frames, and
    # the conversion behind it does not care which side of the link is real.
    sensors_params = os.path.join(bringup, 'config', 'sensors_hege_nav.yaml')
    mux_params = os.path.join(bringup, 'config', 'twist_mux.yaml')
    nav2_overlay = os.path.join(bringup, 'config', 'nav2_real_overlay.yaml')

    start_agent = LaunchConfiguration('start_agent')
    agent_port = LaunchConfiguration('agent_port')
    bridge_params = LaunchConfiguration('bridge_params')
    rviz = LaunchConfiguration('rviz')

    # There is no /clock on this side. The uXRCE-DDS agent re-bases PX4's
    # microsecond timestamps onto the companion's clock, so every node runs on
    # wall time. Setting this true makes every one of them block forever
    # waiting for a topic nobody publishes.
    use_sim_time = 'false'

    # TF only - nothing here simulates anything. This is what gives
    # navsat_transform the measured base_link -> gps_link offset so it can take
    # the 1.30 m antenna height out of the fix, and what gives Nav2 a
    # base_footprint to plan for.
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

    # Zeros for the wheel and steer joints. The frames that matter -
    # base_footprint, base_link, imu_link, gps_link - are all fixed joints and
    # go out on /tf_static regardless; this exists so robot_state_publisher
    # stops warning and so the rover has wheels in RViz.
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

    bridge = Node(
        package='hege_px4_bridge', executable='bridge',
        name='hege_px4_bridge',
        parameters=[bridge_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # There are no wheel encoders behind a Pixhawk. The forward
            # velocity comes from PX4's own estimate instead; only vx is taken
            # from it, because its position is built from the same GPS that
            # navsat_transform already feeds in.
            'odom_topic': '/px4/odom',
            # The map origin comes from the first fix, so no survey is needed -
            # but wait for a good fix before launching, or the whole map
            # anchors on a bad one.
            'use_datum': 'false',
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # Lowest priority into twist_mux, so teleop outranks the navigator.
            'cmd_vel_topic': '/cmd_vel/nav',
            # 0.3 m/s, forward-only planner and controller.
            'params_overlay': nav2_overlay,
            # Trees without the BackUp recovery the bridge cannot execute.
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
            description='Must equal PX4 UXRCE_DDS_DOM_ID. With the default 0 '
                        'the /fmu topics never appear and it looks like the '
                        'bridge is down.'),
        DeclareLaunchArgument(
            'bridge_params',
            default_value=os.path.join(bridge_share, 'config',
                                       'bridge_real.yaml'),
            description='Bridge limits. The default caps the vehicle at '
                        '0.3 m/s and refuses to start until px4_yaw_p is set '
                        'to the tuned RO_YAW_P.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz with the navigation view.'),

        SetEnvironmentVariable('ROS_DOMAIN_ID',
                               LaunchConfiguration('ros_domain_id')),

        robot_state_publisher,
        joint_state_publisher,
        agent,
        sensors,
        twist_mux,
        bridge,
        localization_launch,
        navigation_launch,
    ])
