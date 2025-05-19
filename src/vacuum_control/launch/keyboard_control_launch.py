from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vacuum_control',
            executable='keyboard_controller_node',
            name='keyboard_controller_node',
            output='screen',
            emulate_tty=True
        )
    ])
