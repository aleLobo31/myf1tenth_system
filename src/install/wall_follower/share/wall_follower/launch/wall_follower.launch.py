from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wall_follower'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_follower',
            name='wall_follower',
            parameters=[config]
        )
    ]) 