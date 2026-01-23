from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    nav2_dir = get_package_share_directory('nav2_bringup')

    nav2_launch = os.path.join(
        nav2_dir,
        'launch',
        'bringup_launch.py'
    )

    nav2_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'nav2.yaml'
    )

    map_file = '/home/pg/ros2_ws/src/robot_bringup/maps/my_map3.yaml'

    

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'false',
                'params_file': nav2_params
            }.items()
        )

    ])
