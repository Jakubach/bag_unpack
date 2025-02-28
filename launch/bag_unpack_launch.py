import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('bag_unpack'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=config_file_path,
            description='Path to the config file with parameters'
        ),
        Node(
            package='bag_unpack',
            executable='bag_unpack',
            name='bag_unpack',
            output='screen',
            parameters=[config_file_path], 
        ),
    ])
