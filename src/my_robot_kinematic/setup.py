from setuptools import find_packages, setup

package_name = 'my_robot_kinematic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vietle9204',
    maintainer_email='vietle9204@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometry_publisher = my_robot_kinematic.odometry_publisher:main',
            'gz_vel_encoder_fake = my_robot_kinematic.gz_vel_encoder_fake:main',
            'odom_from_vel_encoder = my_robot_kinematic.odom_from_vel_encoder:main',
            'jointState_from_vel_encoder = my_robot_kinematic.jointState_from_vel_encoder:main',
            'odometry_kf = my_robot_kinematic.odometry_kf:main',
            'odom_kf_xy_from_imu = my_robot_kinematic.odom_kf_xy_from_imu:main',
            'odometry_ekf = my_robot_kinematic.odometry_ekf:main',
            'imu_filter = my_robot_kinematic.imu_filter:main',
        ],
    },
)
