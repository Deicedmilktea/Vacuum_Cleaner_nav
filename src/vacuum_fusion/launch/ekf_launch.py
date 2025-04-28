import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'vacuum_fusion'
    share_dir = get_package_share_directory(pkg_name)

    # Construct the full path to the EKF config file
    ekf_config_path = os.path.join(share_dir, 'config', 'ekf.yaml')

    # Start robot localization using an EKF
    start_robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('/odometry/filtered', '/odom')] # Optional: Remap output topic if needed by other nodes like Nav2
                                                     # Default output is /odometry/filtered
    )

    return LaunchDescription([
        start_robot_localization_node
    ])
