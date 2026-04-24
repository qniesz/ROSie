"""
rosie_bringup.launch.py — Core ROSie ROS 2 bringup (nav container).

Starts two nodes that must always be running in the nav container:
  1. robot_state_publisher — URDF → static TF (base_link → laser, etc.)
  2. foxglove_bridge   — WebSocket visualization on port 8765

NOTE: rosie_driver runs in its own container (rosie-driver) and is NOT
started here. All three used to be in a single container; the driver was
split out so it can be rebuilt independently without touching Nav2/SLAM.

SLAM Toolbox and Nav2 are started/stopped separately by rosie_server.py
(the map pipeline supervisor) so they can be toggled at runtime.

Usage (called by rosie_server.py):
  ros2 launch rosie_bringup rosie_bringup.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_file = os.path.join(pkg_dir, 'urdf', 'rosie.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # ── 1. Robot state publisher (URDF → static TF) ──────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ── 2. Foxglove WebSocket bridge ─────────────────────────────────
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
