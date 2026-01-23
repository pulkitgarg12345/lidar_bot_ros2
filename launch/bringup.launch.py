from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_bringup')

    # ---------------- Robot description ----------------
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # ---------------- EKF config ----------------
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    # ---------------- Nodes ----------------
    serial_node = Node(
        package='robot_bringup',
        executable='serial_node',
        name='serial_node',
        output='screen'
    )

    motor_node = Node(
        package='robot_bringup',
        executable='motor_node',
        name='motor_node',
        output='screen'
    )

    odom_node = Node(
        package='robot_bringup',
        executable='odom_node',
        name='odom_node',
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'laser'
        }]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }]
    )

    return LaunchDescription([
        serial_node,
        motor_node,
        odom_node,
        ekf_node,
        lidar_node,
        robot_state_publisher
    ])
