from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_tools',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            remappings=[
                ('/imu/data_raw', '/imu/data'),               # input raw
                ('/imu/data', '/imu/data_filtered')      # output filtered
            ],
            parameters=[{
                'publish_tf': False,
                'frequency': 100.0,
                'beta': 0.1,
                'world_frame': 'base_link'
            }]
        )
    ])
