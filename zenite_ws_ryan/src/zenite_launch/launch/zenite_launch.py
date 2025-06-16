from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_control',
            executable='camera_control_node',
            name='camera_control',
            output='screen',
            prefix='xterm -e',
        ),

        Node(
            package='interface_node',
            executable='interface_node',
            name='interface_node',
            output='screen',
            prefix='xterm -e',  # Abre em terminal separado
        ),
    ])
