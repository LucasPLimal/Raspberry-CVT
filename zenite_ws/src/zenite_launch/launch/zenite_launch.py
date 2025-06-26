from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='acquisition_node',
            executable='acquisition_node',
            name='acquisition_node',
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
