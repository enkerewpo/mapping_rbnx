"""ROS2 launch file for FASTLIO2 in mapping mode (LIO + PGO with loop closure).

Launches:
  - fastlio2/lio_node: LiDAR-Inertial odometry (ESKF + ikd-Tree)
  - pgo/pgo_node: Pose graph optimization with loop closure detection
  - livox_ros_driver2: Livox LiDAR driver

Usage:
  # Native (Jetson Orin):
  ros2 launch mapping_rbnx slam_mapping.launch.py
  ros2 launch mapping_rbnx slam_mapping.launch.py lio_config:=/path/to/lio.yaml

  # Docker (x86):
  ros2 launch /ws/launch/slam_mapping.launch.py
"""
import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, 'config')

    # Default config paths — use installed package configs if available,
    # otherwise fall back to local config directory.
    default_lio_config = os.path.join(config_dir, 'fastlio2_default.yaml')
    default_pgo_config = os.path.join(config_dir, 'fastlio2_default.yaml')

    # Try to use upstream package configs when installed via colcon
    try:
        ctx = LaunchContext()
        default_lio_config = PathJoinSubstitution(
            [FindPackageShare('fastlio2'), 'config', 'lio.yaml']
        ).perform(ctx)
        default_pgo_config = PathJoinSubstitution(
            [FindPackageShare('pgo'), 'config', 'pgo.yaml']
        ).perform(ctx)
    except Exception:
        pass

    return LaunchDescription([
        DeclareLaunchArgument(
            'lio_config', default_value=default_lio_config,
            description='Path to fastlio2 LIO config YAML',
        ),
        DeclareLaunchArgument(
            'pgo_config', default_value=default_pgo_config,
            description='Path to PGO config YAML',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch RViz2 visualization',
        ),

        # ── fastlio2 LIO node ───────────────────────────────────────────
        Node(
            package='fastlio2',
            namespace='fastlio2',
            executable='lio_node',
            name='lio_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('lio_config'),
            }],
        ),

        # ── PGO node (loop closure + pose graph optimization) ───────────
        Node(
            package='pgo',
            namespace='pgo',
            executable='pgo_node',
            name='pgo_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('pgo_config'),
            }],
        ),

        # NOTE: Livox driver is launched separately by lidar_rbnx package.
        # It publishes /livox/lidar (CustomMsg) and /livox/imu (Imu).

        # ── 3D → 2D projection (for Nav2 costmap) ──────────────────────
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/fastlio2/world_cloud'),
                ('scan', '/robonix/map/scan_2d'),
            ],
            parameters=[{
                'min_height': 0.1,
                'max_height': 2.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00436,  # ~0.25 deg
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 30.0,
                'target_frame': 'base_link',
                'inf_epsilon': 1.0,
                'use_inf': True,
            }],
        ),
    ])
