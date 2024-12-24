from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_pkg',
            executable='safety_node',
            name='safety_node',
            output='screen',
            # You can add parameters here if needed
            # parameters=[{'param_name': param_value}]
        )
    ]) 