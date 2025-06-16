from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): 
    return LaunchDescription([
        # Nó de aquisição (webcam) - Terminal 1
        #Node(
            #package='acquisition_node',
            #node',
            #name='acquisition_node,
            #output='screen',
            #prefix='xterm -e'  # Abre em terminal separado
        #),
        
        # Nó de interface - Terminal 3
        Node(
            package='interface_node',
            executable='interface_node',
            name='interface_node',
            output='screen',
            prefix='xterm -e',  # Abre em terminal separado
        ),

        Node(
            package='camera_control',
            executable='camera_control_node',
            name='usb_cam',
            output='screen',
            prefix='xterm -e',
        ),
    ])
