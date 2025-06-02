import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Package share directories
    pkg_ldlidar = get_package_share_directory('ldlidar_sl_ros2')
    pkg_vacuum_describe = get_package_share_directory('vacuum_describe')
    pkg_vacuum_interface = get_package_share_directory('vacuum_interface')
    pkg_vacuum_fusion = get_package_share_directory('vacuum_fusion')
    pkg_vacuum_nav = get_package_share_directory('vacuum_nav')
    pkg_vacuum_slam = get_package_share_directory('vacuum_slam')
    pkg_vacuum_wifi = get_package_share_directory('vacuum_wifi')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_params_file = LaunchConfiguration('nav_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_nav_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(pkg_vacuum_nav, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_vacuum_slam, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the SLAM Toolbox parameters file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load. Leave empty for SLAM mode (default)')

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

    # --- Hardware and System Nodes ---

    # 1. Lidar Driver
    lidar_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ldlidar, 'launch', 'ld14.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. STM32 Interface
    stm32_interface_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vacuum_interface, 'launch', 'stm32_interface_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Robot Description (TF)
    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vacuum_describe, 'launch', 'robot_description_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. EKF Fusion
    ekf_fusion_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vacuum_fusion, 'launch', 'ekf_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. WiFi Communication
    wifi_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vacuum_wifi, 'launch', 'wifi_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- SLAM and Navigation ---

    # 6. SLAM Toolbox (async mode for real-time mapping)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    # 7. Nav2 Navigation Stack
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav_params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'map': map_yaml_file
        }.items())

    # 8. Goal Receiver Node (接收rviz2目标点)
    goal_receiver_node = Node(
        package='vacuum_nav',
        executable='goal_receiver_node',
        name='goal_receiver_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 9. Velocity Converter Node (转换速度指令格式)
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

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav_params_file_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add hardware and system nodes
    ld.add_action(lidar_driver_cmd)
    ld.add_action(stm32_interface_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(ekf_fusion_cmd)
    ld.add_action(wifi_cmd)

    # Add SLAM and navigation
    ld.add_action(slam_toolbox_node)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(goal_receiver_node)
    ld.add_action(velocity_converter_node)

    return ld
