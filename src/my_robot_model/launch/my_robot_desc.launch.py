from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    urdf_pkg_path = FindPackageShare('my_robot_model')
    default_model = 'robot.urdf.xacro'
    
    rviz_config = PathJoinSubstitution([
        urdf_pkg_path,  
        'rviz',
        'config.rviz'  
    ])
        
    robot_desc = Command(['xacro ', PathJoinSubstitution([urdf_pkg_path, 'description', LaunchConfiguration('robot_model')])])

    config_file_path = os.path.join(
                get_package_share_directory("odom_to_tf"), "config", "odom_to_tf.yaml"
            )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value=default_model,
            description='urdf'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Whether to publish tf or not'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_desc
            }],
        ),

        # ====== joint state =====
        Node(
            package='my_robot_kinematic',
            executable='jointState_from_vel_encoder',
            name='jointState_from_vel_encoder',
            output='screen',
        ),

        # ====== imu filter =====
        Node(
            package='my_robot_kinematic',
            executable='imu_filter',
            name='imu_filter',
            output='screen',
            prefix='xterm -e',
        ),

        # ====== odometry =====

        TimerAction(
            period=3.0,  # đợi 4 giây cho robot_state_publisher publish
            actions=[
            
            # Node(
            #     package='my_robot_kinematic',
            #     executable='odom_from_vel_encoder',
            #     name='odom_from_vel_encoder',
            #     parameters=[{'imu_topic': 'imu/filtered'}],
            #     output='screen',
            # ),

            # Node(
            #     package='my_robot_kinematic',
            #     executable='odometry_ekf',
            #     name='odometry_kf',
            #     output='screen',
            #     parameters=[{'imu_topic': 'imu/filtered'}]
            # ),

            Node(
                package='my_robot_kinematic',
                executable='state_estimate',
                name='state_estimate',
                output='screen',
                parameters=[{'imu_topic': 'imu/filtered'}]
            ),
        ]),
        
        # RViz
        TimerAction(
            period=5.0,  # đợi 4 giây cho robot_state_publisher publish
            actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]),

        Node(
            package="odom_to_tf",
            executable="odom_to_tf",
            name="odom_to_tf",
            output="screen",
            parameters=[config_file_path],
            remappings=[],
            condition=IfCondition(LaunchConfiguration('publish_tf'))
        )

    ])
