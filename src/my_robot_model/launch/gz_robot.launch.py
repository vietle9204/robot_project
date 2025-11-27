#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    pkg_path = get_package_share_directory('my_robot_model')
    # xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    # robot_description = xacro.process_file(xacro_file).toxml()
    world_path = os.path.join(pkg_path, 'worlds', 'my_world.sdf')
    # rviz_config = os.path.join(pkg_path, 'rviz', 'config.rviz')
    # ekf_config = os.path.join(pkg_path, 'config', 'ekf.yaml')

    # Robot description publisher
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_path, 'launch', 'my_robot_desc.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 'robot_model': 'robot.urdf.xacro'
        }.items()
    )

    # --- Launch Gazebo ---
    gz_process = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'sim', world_path],
                output='screen'
            )
        ]
    )

    # spawn robot in Gazebo
    spawn_robot = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                    '-topic', 'robot_description',
                    '-name', 'my_robot',
                    '-x', '-5.0',
                    '-y', '-5.0'],
                output='screen'
            )
        ]
    )

    # bridge between ROS2 and Gazebo
    node_gz_bridge = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',

                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

                    '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

                    # '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

                    '/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist',

                    '/world/simple_room/model/my_robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'

                ],
                output='screen',
                remappings=[#('/model/my_robot/odometry', '/odometry/data'),
                            ('/world/simple_room/model/my_robot/joint_state', '/joint_states_raw'),
                            ('/imu', '/imu/data')
                ]
            ),
            Node(
                package='my_robot_kinematic',
                executable='gz_vel_encoder_fake',
                name='gz_vel_encoder_fake',
                output='screen',
            )
        ]
    )



    return LaunchDescription([
        declare_use_sim_time,
        robot_desc_launch,
        gz_process,
        spawn_robot,
        node_gz_bridge
    ])

