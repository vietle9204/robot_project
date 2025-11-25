#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    pkg_path = get_package_share_directory('my_robot_model')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    world_path = os.path.join(pkg_path, 'worlds', 'my_world.sdf')
    rviz_config = os.path.join(pkg_path, 'rviz', 'config.rviz')
    ekf_config = os.path.join(pkg_path, 'config', 'ekf.yaml')

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    wheel_odom_pub = TimerAction(
        period=7.0,
        actions=[
            Node(
            package='my_robot_model',
            executable='wheel_odom_publisher.py',
            name='wheel_odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # RViz
    rviz = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp,
        wheel_odom_pub,
        rviz,
    ])

