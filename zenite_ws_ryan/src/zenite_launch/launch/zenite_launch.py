from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    interface_node = Node(
        package='interface_node',
        executable='interface_node',
        name='interface_node',
        output='screen',
        prefix='xterm -e',
    )

    camera_control = Node(
        package='camera_control',
        executable='camera_control_node',
        name='camera_node',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([
        interface_node,
        camera_control,

        # Se o interface_node morrer, encerra tudo
        RegisterEventHandler(
            OnProcessExit(
                target_action=interface_node,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        )
    ])