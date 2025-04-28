import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'vacuum_slam'
    share_dir = get_package_share_directory(pkg_name)

    # Configuration file path
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Declare the launch argument for the parameters file
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(share_dir, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the SLAM Toolbox parameters file')

    # Start SLAM Toolbox node (async mode)
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node', # Corrected executable name
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        # Remappings can be added here if needed, e.g.,
        # remappings=[('/scan', '/my_scan_topic')]
    )

    return LaunchDescription([
        declare_slam_params_file_cmd,
        start_async_slam_toolbox_node
    ])
