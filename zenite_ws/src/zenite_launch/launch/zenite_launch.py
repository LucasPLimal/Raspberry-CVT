from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('zenite_launch'),
        'config',
        'usb_cam.yaml'
    )
    
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
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[config_path],
            prefix='xterm -e',
        ),
    ])
