from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
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
            'vfh_alg_test = my_robot_control.vfh_alg_test:main',
            'A_start = my_robot_control.A_star_implementation:main',
            'my_robot_nav = my_robot_control.my_robot_nav:main'
        ],
    },
)
