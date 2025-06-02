from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    vacuum_slam_dir = get_package_share_directory('vacuum_slam')
    vacuum_nav_dir = get_package_share_directory('vacuum_nav')
    
    # Include SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(vacuum_slam_dir, 'launch', 'slam_toolbox_launch.py')
        ])
    )
    
    # Nav2 params file
    nav2_params_path = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    
    # Include Nav2 bringup launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )
    
    # Launch our navigation node
    navigation_node = Node(
        package='vacuum_nav',
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )
    
    return LaunchDescription([
        slam_launch,
        nav2_launch,
        navigation_node
    ])
