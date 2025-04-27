import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'vacuum_slam'
    
    # Get the urdf file path
    urdf_file_name = 'vacuum.urdf'
    urdf_file_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', urdf_file_name)

    # Read the URDF file
    with open(urdf_file_path, 'r') as file:
        robot_description = file.read()

    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 100.0,
        }]
    )

    # Return launch description
    return LaunchDescription([
        robot_state_publisher_node
    ])
