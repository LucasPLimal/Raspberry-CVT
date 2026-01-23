from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='acquisition_node',
            executable='acquisition_node',
            name='acquisition_node',
            output='screen',
            prefix='xterm -hold -e'
        ),

        Node(
            package='interface_node',
            executable='interface_node',
            name='interface_node',
            output='screen',
            prefix='xterm -hold -e'
        ),

        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen',
            prefix='xterm -hold -e'
        ),

        Node(
            package='transmission_node',
            executable='transmission_node',
            name='transmission_node',
            output='screen',
            prefix='xterm -hold -e'
        ),

        Node(
            package='localization_node',
            executable='localization_node',
            name='localization_node',
            output='screen',
            prefix='xterm -hold -e'
        ),
    ])
