import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    safety_config = os.path.join(
        get_package_share_directory('safety_pkg'),
        'config',
        'safety_params.yaml'
    )

    safety_la = DeclareLaunchArgument(
        'safety_config',
        default_value=safety_config
    )

    safety_node = Node(
        package="safety_pkg",
        executable="safety_node",
        name="safety_node",
        parameters=[LaunchConfiguration('safety_config')]
    )

    ld = LaunchDescription([safety_la])

    ld.add_action(safety_node)

    return ld