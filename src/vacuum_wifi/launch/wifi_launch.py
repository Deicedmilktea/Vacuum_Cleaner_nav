from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vacuum_wifi',
            executable='wifi_node',
            name='wifi_node',
            output='screen',
            parameters=[{
                'tcp_server_ip': '0.0.0.0',
                'tcp_server_port': 8080,
                'map_send_interval': 1.0
            }]
        )
    ])
