import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    vacuum_nav_dir = get_package_share_directory('vacuum_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(vacuum_nav_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load. Leave empty for SLAM mode')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    # Nav2 bringup launch
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'map': map_yaml_file
        }.items())

    # Goal receiver node (接收rviz2目标点)
    goal_receiver_node = Node(
        package='vacuum_nav',
        executable='goal_receiver_node',
        name='goal_receiver_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Velocity converter node (转换速度指令格式)
    velocity_converter_node = Node(
        package='vacuum_nav',
        executable='velocity_converter_node',
        name='velocity_converter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_threshold': 0.05,
            'angular_threshold': 0.1,
            'max_linear_vel': 0.26,
            'max_angular_vel': 1.0,
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch navigation nodes
    ld.add_action(bringup_cmd)
    ld.add_action(goal_receiver_node)
    ld.add_action(velocity_converter_node)

    return ld
