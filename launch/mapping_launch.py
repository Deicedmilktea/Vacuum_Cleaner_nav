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
    pkg_vacuum_interface = get_package_share_directory('vacuum_interface') # Needed for launch file path
    pkg_vacuum_fusion = get_package_share_directory('vacuum_fusion')
    pkg_vacuum_slam = get_package_share_directory('vacuum_slam')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file',
                                           default=os.path.join(pkg_vacuum_slam, 'config', 'slam_toolbox_params.yaml'))

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_vacuum_slam, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the SLAM Toolbox parameters file')

    # Arguments for serial_port and baud_rate removed, handled in included launch file

    # --- Nodes and Included Launch Files ---

    # 1. Lidar Driver
    lidar_launch_path = os.path.join(pkg_ldlidar, 'launch', 'ld14.launch.py') # Adjust if using ld14p or other
    lidar_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items() # Pass use_sim_time if needed by lidar launch
    )

    # 2. STM32 Interface Launch File
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

    # 5. SLAM Toolbox
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vacuum_slam, 'launch', 'slam_toolbox_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)

    # Add nodes and includes
    ld.add_action(lidar_driver_cmd)
    ld.add_action(stm32_interface_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(ekf_fusion_cmd)
    ld.add_action(slam_toolbox_cmd)

    return ld
