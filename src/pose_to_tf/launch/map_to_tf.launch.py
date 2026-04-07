import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pose_to_tf'),
        'params',
        'map_to_tf.yaml' 
    )

    return LaunchDescription([
        Node(
            package='pose_to_tf',
            executable='pose_to_tf',
            name='pose_to_tf',
            parameters=[config],
            output='screen'
        )
    ])