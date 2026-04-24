"""
slam.launch.py — SLAM Toolbox online async mapping.

Usage:
  ros2 launch rosie_bringup slam.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    slam_node = LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            parameters=[slam_params],
            output='screen',
        )

    # Auto-configure and activate the lifecycle node
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: True,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: True,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    return LaunchDescription([
        slam_node,
        configure_event,
        activate_event,
    ])
