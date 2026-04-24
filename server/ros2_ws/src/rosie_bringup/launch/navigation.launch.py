"""
navigation.launch.py — Nav2 stack for autonomous navigation.

Requires a map (from SLAM) and the rosie_bringup base stack running.

Usage:
  ros2 launch rosie_bringup navigation.launch.py map:=/path/to/map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # Use our patched navigation_launch_base.py which omits smoother_server
    # (smoother_server causes lifecycle bringup failure in Nav2 Jazzy 1.3.x on ARM64)
    rosie_launch_dir = os.path.join(pkg_dir, 'launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/ros2_ws/maps/home.yaml',
            description='Full path to the map yaml file',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosie_launch_dir, 'navigation_launch_base.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),
    ])
