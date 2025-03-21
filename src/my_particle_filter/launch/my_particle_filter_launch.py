import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    my_particle_filter_config = os.path.join(
        get_package_share_directory('my_particle_filter'),
        'config',
        'my_particle_filter_params.yaml'
    )
    my_particle_filter_la = DeclareLaunchArgument(
        'my_particle_filter_config',
        default_value=my_particle_filter_config
    )

    my_particle_filter_node = Node(
        package="my_particle_filter",
        executable="pf_node",
        name="pf_node",
        parameters=[LaunchConfiguration('my_particle_filter_config')]
    )

    ld = LaunchDescription([my_particle_filter_la])

    ld.add_action(my_particle_filter_node)

    return ld