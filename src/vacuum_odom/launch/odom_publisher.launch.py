from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vacuum_odom',
            executable='odom_publisher_node', # This matches the entry point in setup.py
            name='odom_publisher_node',      # Node name in the ROS graph
            output='screen'
            # Add parameters here if needed, e.g.:
            # parameters=[{'param_name': 'value'}]
        )
    ])
