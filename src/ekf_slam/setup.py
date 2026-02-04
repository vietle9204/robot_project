from setuptools import find_packages, setup

package_name = 'ekf_slam'

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
            'ekf_slam = ekf_slam.ekf_slam:main',
            'map_to_tf = ekf_slam.map_to_tf:main',
            'map_draw = ekf_slam.map_draw:main',
        ],
    },
)
