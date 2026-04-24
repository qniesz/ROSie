"""
foxglove.launch.py — Foxglove WebSocket bridge for visualization.

Opens a WebSocket on port 8765 for Foxglove Studio to connect to.

Usage:
  ros2 launch rosie_bringup foxglove.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'send_buffer_limit': 10000000,
                'qos_overrides./map.durability': 'transient_local',
                'qos_overrides./map.reliability': 'reliable',
                'qos_overrides./map.depth': 1,
            }],
            output='screen',
        ),
    ])
