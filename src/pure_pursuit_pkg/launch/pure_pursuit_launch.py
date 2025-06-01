import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pure_pursuit_config = os.path.join(
        get_package_share_directory('pure_pursuit_pkg'),
        'config',
        'pure_pursuit_params.yaml'
    )

    pure_pursuit_la = DeclareLaunchArgument(
        'pure_pursuit_config',
        default_value=pure_pursuit_config
    )

    pure_pursuit_node = Node(
        package="pure_pursuit_pkg",
        executable="pure_pursuit_node",
        name="pure_pursuit_node",
        parameters=[LaunchConfiguration('pure_pursuit_config')]
    )

    ld = LaunchDescription([pure_pursuit_la])

    ld.add_action(pure_pursuit_node)

    return ld