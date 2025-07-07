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

        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen',
            prefix='xterm -hold -e',
            arguments=['5.0', '5.0'],  # se quiser mudar o destino
        ),

        Node(
            package='transmission_node',
            executable='transmission_node',
            name='transmission_node',
            output='screen',
            prefix='xterm -e',
        ),

    ])
