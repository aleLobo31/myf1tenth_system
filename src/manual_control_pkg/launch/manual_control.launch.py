from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('manual_control_pkg'),
        'config',
        'manual_control.yaml'
    )

    return LaunchDescription([
        Node(
            package='manual_control_pkg',
            executable='manual_control_node',
            name='manual_control_node',
            parameters=[config],
        ),
    ])
