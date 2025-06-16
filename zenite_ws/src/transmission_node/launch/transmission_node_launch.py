from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='transmission_node',
            executable='transmission_subscriber_node',
            name='transmission_node',
            output='screen'
        )
    ])
