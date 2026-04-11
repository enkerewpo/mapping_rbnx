"""ROS2 launch file for FASTLIO2 in localization mode (LIO + ICP relocalization).

Launches:
  - fastlio2/lio_node: LiDAR-Inertial odometry
  - localizer/localizer_node: Two-stage coarse-to-fine ICP relocalization
  - livox_ros_driver2: Livox LiDAR driver

Relocalization is triggered via ROS2 service /localizer/relocalize with
a PCD map path and initial pose estimate.

Usage:
  # Native (Jetson Orin):
  ros2 launch mapping_rbnx slam_localization.launch.py

  # Then trigger relocalization:
  ros2 service call /localizer/relocalize interface/srv/Relocalize \
    '{pcd_path: "/maps/map.pcd", x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}'
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

    default_lio_config = os.path.join(config_dir, 'fastlio2_default.yaml')
    default_localizer_config = os.path.join(config_dir, 'fastlio2_default.yaml')

    try:
        ctx = LaunchContext()
        default_lio_config = PathJoinSubstitution(
            [FindPackageShare('fastlio2'), 'config', 'lio.yaml']
        ).perform(ctx)
        default_localizer_config = PathJoinSubstitution(
            [FindPackageShare('localizer'), 'config', 'localizer.yaml']
        ).perform(ctx)
    except Exception:
        pass

    return LaunchDescription([
        DeclareLaunchArgument(
            'lio_config', default_value=default_lio_config,
            description='Path to fastlio2 LIO config YAML',
        ),
        DeclareLaunchArgument(
            'localizer_config', default_value=default_localizer_config,
            description='Path to localizer config YAML',
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

        # ── Localizer node (ICP relocalization) ─────────────────────────
        Node(
            package='localizer',
            namespace='localizer',
            executable='localizer_node',
            name='localizer_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('localizer_config'),
            }],
        ),

        # NOTE: Livox driver is launched separately by lidar_rbnx package.

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
                'angle_increment': 0.00436,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 30.0,
                'target_frame': 'base_link',
                'inf_epsilon': 1.0,
                'use_inf': True,
            }],
        ),
    ])
