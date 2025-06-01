import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare Launch Arguments first to define their defaults
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',  # Defaulting STM32 interface to /dev/ttyUSB1
        description='Serial port for STM32 connection')

    declare_baud_rate_cmd = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for STM32 connection')

    # Launch configurations that will pick up the defaults from DeclareLaunchArgument
    # if not overridden when this launch file is included.
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    serial_port_config = LaunchConfiguration('serial_port')
    baud_rate_config = LaunchConfiguration('baud_rate')

    # STM32 Interface Node
    stm32_interface_node = Node(
        package='vacuum_interface',
        executable='stm32_interface_node',
        name='stm32_interface_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port_config,  # Uses /dev/ttyUSB1 by default now
            'baud_rate': baud_rate_config,
            'use_sim_time': use_sim_time_config,
            'simulate': False  # Explicitly set to False to ensure serial mode
            }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_serial_port_cmd,
        declare_baud_rate_cmd,
        stm32_interface_node
    ])
