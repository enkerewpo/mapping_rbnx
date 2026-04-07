"""ROS2 launch file for FAST-LIVO2 in localization mode (against pre-built map).

Usage:
  ros2 launch mapping_rbnx slam_localization.launch.py map_file:=/maps/map.pcd
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, 'config')
    default_config = os.path.join(config_dir, 'fast_livo2_default.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Path to FAST-LIVO2 config YAML',
        ),
        DeclareLaunchArgument(
            'map_file', default_value='/maps/map.pcd',
            description='Path to pre-built PCD map for relocalization',
        ),

        # ── FAST-LIVO2 node (localization mode) ──────────────────────────
        Node(
            package='fast_livo2',
            executable='fast_livo2_node',
            name='fast_livo2',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'relocalization_enable': True},
                {'map_file': LaunchConfiguration('map_file')},
            ],
            remappings=[
                ('/Odometry', '/robonix/slam/odom'),
                ('/cloud_registered', '/robonix/slam/cloud_registered'),
                ('/path', '/robonix/slam/path'),
            ],
        ),

        # ── Livox ROS2 driver ────────────────────────────────────────────
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_ros_driver2',
            output='screen',
            parameters=[{
                'xfer_format': 1,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
            }],
        ),
    ])
