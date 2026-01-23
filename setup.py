from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # INSTALL LAUNCH FILES
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # INSTALL URDF FILES
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),

        # (Optional later)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pg',
    maintainer_email='pg@todo.todo',
    description='ROS2 bringup for differential drive robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_only_node = robot_bringup.odom_only_node:main',
            'motor_node = robot_bringup.motor_node:main',
            'imu_node = robot_bringup.imu_node:main',
            'serial_node = robot_bringup.serial_node:main',
            'odom_node = robot_bringup.odom_node:main',
            # If you use unified serial node later:
            # 'serial_node = robot_bringup.serial_node:main',
        ],
    },
)
