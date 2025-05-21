import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    waypoint_generator_config = os.path.join(
        get_package_share_directory('waypoint_generator'),
        'config',
        'waypoint_generator_params.yaml'
    )

    waypoint_generator_la = DeclareLaunchArgument(
        'waypoint_generator_params',
        default_value=waypoint_generator_config
    )

    ld = LaunchDescription([waypoint_generator_la])

    waypoint_generator_node = Node(
        package="waypoint_generator",
        executable="waypoint_generator_node",
        name="waypoint_generator_node",
        parameters=[LaunchConfiguration('waypoint_generator_config')]
    )



    ld.add_action(waypoint_generator_node)

    return ld