from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

config_file_path = os.path.join(
            get_package_share_directory("odom_to_tf"), "config", "odom_to_tf.yaml"
        )

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_tf = LaunchConfiguration('publish_tf')
    use_LPF = LaunchConfiguration('use_LPF')

    return LaunchDescription([

        # ===== Declare arguments =====
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'publish_tf',
            default_value='true'
        ),

         DeclareLaunchArgument(
            'use_LPF',
            default_value='false'
        ),

        # ===== EKF SLAM Nodes =====
        Node(
            package='ekf_slam',
            executable='map_draw',
            name='map_draw',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='ekf_slam',
            executable='map_to_tf',
            name='map_to_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='ekf_slam',
            executable='ekf_slam',
            name='ukf_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # ===== UKF State Estimation =====
        Node(
            package='my_robot_kinematic',
            executable='state_estimate_UKF2',
            name='state_estimate_UKF',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # ===== Odom to TF =====
        Node(
            package='odom_to_tf',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
            parameters=[config_file_path,
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(publish_tf)
        ),

        # ===== IMU Filter =====
        Node(
            package='my_robot_kinematic',
            executable='imu_filter',
            name='imu_filter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_LPF)
        ),

        # ===== Scan to PointCloud =====
        Node(
            package='my_robot_kinematic',
            executable='scanToCloud',
            name='scanToCloud',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

    ])
