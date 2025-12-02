import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Đường dẫn config cho imu_complementary_filter
    imu_config = os.path.join(
        get_package_share_directory('imu_complementary_filter'),
        'config',
        'filter_config.yaml'
    )

    # Node imu_complementary_filter
    imu_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[imu_config, {'publish_tf': False}],
        remappings=[
                ('/imu/data_raw', '/imu/data'),               # input raw
                ('/imu/data', '/imu/data_filtered')      # output filtered
            ],
    )

    # Node ekf
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[[
            PathJoinSubstitution([
                FindPackageShare('my_robot_localization'),  # đổi thành package của bạn
                'config',
                'ekf.yaml'
            ])
        ]]
    )

    # Gộp lại
    ld = LaunchDescription()
    # ld.add_action(imu_node)
    ld.add_action(ekf_node)

    return ld
