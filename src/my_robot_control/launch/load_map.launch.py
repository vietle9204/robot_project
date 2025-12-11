#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import time

def generate_launch_description():

    map_yaml = "/home/vietle9204/robot_ws/src/maps/my_map.yaml"

    # 1. Launch map_server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}],
    )

    # 2. Launch AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
    )

    # 3. Launch lifecycle_manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # 4. Publish initial pose (example, delayed 5s)
    pub_pose_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            "'{header: {frame_id: \"map\"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'",
            '--once'
        ],
        shell=True
    )
    pub_pose_timer = TimerAction(period=10.0, actions=[pub_pose_cmd])

    # 5. Call load_map service (delayed 6s)
    call_load_map = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/map_server/load_map', 'nav2_msgs/srv/LoadMap',
            f'{{map_url: "{map_yaml}"}}'
        ],
        shell=True
    )
    call_load_map_timer = TimerAction(period=10.0, actions=[call_load_map])

    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        pub_pose_timer,
        # call_load_map_timer
    ])
