from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz2_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mapa.rviz'
    )

    # Declare launch arguments
    rviz_la = DeclareLaunchArgument(
        'rviz2_config',
        default_value=rviz2_config,
        description='Descriptions for rviz configs'
    )

    ld = LaunchDescription([rviz_la])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz2_config')],
    )

    # finalize
    ld.add_action(rviz2_node)

    return ld
