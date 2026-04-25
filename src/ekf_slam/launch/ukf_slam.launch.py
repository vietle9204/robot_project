from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
            get_package_share_directory("ekf_slam"), "config", "ukf_slam.yaml"
        )

ukf_slam_config_arg = DeclareLaunchArgument(
    'ukf_slam_config',
    default_value=config
)

ukf_slam_config = LaunchConfiguration('ukf_slam_config')


def generate_launch_description():
    return LaunchDescription([
        ukf_slam_config_arg,

        # ===== EKF SLAM Nodes =====
        Node(
            package='ekf_slam',
            executable='map_draw',
            name='map_draw',
            output='screen',
        ),

        Node(
            package='ekf_slam',
            executable='map_to_tf',
            name='map_to_tf',
            output='screen',
        ),

        Node(
            package='ekf_slam',
            executable='ukf_slam',
            name='ukf_slam',
            output='screen',
            parameters=[ukf_slam_config]
        ),

    ])
