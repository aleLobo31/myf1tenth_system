from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    waypoint_generator_config = os.path.join(
        get_package_share_directory('waypoint_generator_pkg'),
        'config',
        'waypoint_visualizer_params.yaml'
    )

    waypoint_visualizer_la = DeclareLaunchArgument(
        'waypoint_generator_config',
        default_value=waypoint_generator_config
    )

    ld = LaunchDescription([waypoint_visualizer_la])

    waypoint_visualizer_node = Node(
        package="waypoint_generator_pkg",
        executable="waypoint_visualizer_node",
        name="waypoint_visualizer_node",
        parameters=[LaunchConfiguration('waypoint_generator_config')]
    )

    ld.add_action(waypoint_visualizer_node)

    return ld
