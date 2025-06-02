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
    vacuum_nav_dir = get_package_share_directory('vacuum_nav')
    
    # Nav2 params file
    nav2_params_path = os.path.join(vacuum_nav_dir, 'config', 'nav2_params.yaml')
    
    # Behavior tree path
    bt_xml_path = os.path.join(vacuum_nav_dir, 'behavior_trees')
    
    # Include Nav2 bringup launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': 'false',
            'autostart': 'true',
            'bt_xml_file': os.path.join(bt_xml_path, 'navigate.xml')
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
        nav2_launch,
        navigation_node
    ])
