from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
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
        ),

    ])
