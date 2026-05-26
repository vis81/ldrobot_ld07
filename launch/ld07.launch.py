import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ldrobot_ld07')
    params_file = os.path.join(pkg_dir, 'params', 'ld07.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port device path for the LD07 sensor',
        ),
        Node(
            package='ldrobot_ld07',
            executable='ldrobot_ld07_node',
            name='ldrobot_ld07_node',
            output='screen',
            parameters=[
                params_file,
                {'comm.serial_port': LaunchConfiguration('serial_port')},
            ],
        ),
    ])
