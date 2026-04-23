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
            package='calibration_node',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            prefix='xterm -hold -e'
        ),
    ])
