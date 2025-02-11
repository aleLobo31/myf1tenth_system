# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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

    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    lidar_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'lidar.yaml'
    )
    imu_config = os.path.join(
         get_package_share_directory('f1tenth_stack'),
         'config',
         'imu.yaml'
    )
    manual_control_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'manual_control.yaml'
    )
    safety_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'safety.yaml'
    )
    reactive_follower_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'reactive_follower.yaml'
    )

    # Declare launch arguments
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs'
    )
    lidar_la = DeclareLaunchArgument(
        'lidar_config',
        default_value=lidar_config,
        description='Descriptions for lidar configs'
    )
    imu_la = DeclareLaunchArgument(
        'imu_config',
        default_value=imu_config,
        description='Descriptions for imu configs'
    )
    manual_control_la = DeclareLaunchArgument(
        'manual_control_config',
        default_value=manual_control_config,
        description='Descriptions for manual_control configs'
    )
    safety_la = DeclareLaunchArgument(
        'safety_config',
         default_value=safety_config,
         description='Descriptions for safety configs'
    )
    reactive_follower_la = DeclareLaunchArgument(
        'reactive_follower_config',
         default_value=reactive_follower_config,
         description='Descriptions for reactive_follower configs'
    )

    ld = LaunchDescription([vesc_la, lidar_la, imu_la, manual_control_la, safety_la, reactive_follower_la])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    manual_control_node = Node(
        package='manual_control_pkg',
        executable='manual_control_node',
        name='manual_control_node',
        output='screen',
        parameters=[LaunchConfiguration('manual_control_config')]
        )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
        )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
        )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
        )
    ldlidar_stl_ros2 = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        parameters=[LaunchConfiguration('lidar_config')]
        )
    razor_imu_ros2 = Node(
        package='razor_imu_ros2',
        executable='razor_imu_ros2_exe',
        name='IMU_ARTEMIS',
        parameters=[LaunchConfiguration('imu_config')]
        )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0', '0', '0', '-1.5708', '0', '0', 'base_link', 'laser']
        )
    safety_node = Node(
        package='safety_pkg',
        executable='safety_node',
        name='safety_node',
        parameters=[LaunchConfiguration('safety_config')]
        )
    reactive_follower_node = Node(
        package='reactive_follower_pkg',
        executable='reactive_follower_node',
        name='reactive_follower_node',
        parameters=[LaunchConfiguration('reactive_follower_config')]
        )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(manual_control_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(ldlidar_stl_ros2)
    #ld.add_action(razor_imu_ros2)
    ld.add_action(static_tf_node)
    ld.add_action(safety_node)
    ld.add_action(reactive_follower_node)

    return ld
