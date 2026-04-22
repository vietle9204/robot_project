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

config = os.path.join(
            get_package_share_directory("my_robot_kinematic"), "config", "ukf_st.yaml"
        )

ukf_config_arg = DeclareLaunchArgument(
    'ukf_config',
    default_value=config
)

ukf_config = LaunchConfiguration('ukf_config')

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_tf = LaunchConfiguration('publish_tf')
    use_LPF = LaunchConfiguration('use_LPF')


    return LaunchDescription([
        ukf_config_arg,

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

        # ===== UKF State Estimation =====
        Node(
            package='my_robot_kinematic',
            executable='state_estimate_UKF2',
            name='state_estimate_UKF',
            output='screen',
            parameters=[ukf_config,
                {'use_sim_time': use_sim_time}]
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
