import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Goal receiver node
    goal_receiver_node = Node(
        package='vacuum_nav',
        executable='goal_receiver_node',
        name='goal_receiver_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(goal_receiver_node)

    return ld
