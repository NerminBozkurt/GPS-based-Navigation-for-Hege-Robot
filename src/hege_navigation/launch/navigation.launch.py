import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav_pkg_dir = get_package_share_directory('hege_navigation')

    params_file = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Odpalamy "navigation_launch.py" bezpośrednio (ZAMIAST bringup_launch.py).
    # Dlaczego? Ponieważ standardowy bringup odpala serwer map i AMCL,
    # których my NIE MAMY i NIE CHCEMY (bo jesteśmy na pustym polu, nawigujemy przez GPS).
    # navigation_launch.py uruchamia samo "mięso": planner, controller, bt_navigator.
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': params_file}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true, wall time if false'
        ),
        nav2_cmd
    ])
