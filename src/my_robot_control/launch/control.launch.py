from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():

    default_map = os.path.join(
                get_package_share_directory("my_robot_control"), "map", "my_map.yaml"
            )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='path to file map .yaml'
        ),

        Node(
            package='my_robot_control',
            executable='A_star',
            name='A_star',
            output='screen',
            prefix='xterm -e',
            parameters=[{'map_file': LaunchConfiguration('map_file')}],
        ),

        Node(
            package='my_robot_control',
            executable='my_robot_nav',
            name='my_robot_nav',
            output='screen',
            prefix='xterm -e',
        ),

        Node(
            package='my_robot_control',
            executable='vfh_alg_test',
            name='VFH_alg_test',
            output='screen',
            parameters=[{'goal_topic': 'goal_tmp'}],
            prefix='xterm -e',
        ),
    ])
