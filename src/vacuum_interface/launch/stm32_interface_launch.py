import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')

    # Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for STM32 connection')

    declare_baud_rate_cmd = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for STM32 connection')

    # STM32 Interface Node
    stm32_interface_node = Node(
        package='vacuum_interface',
        executable='stm32_interface_node',
        name='stm32_interface_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'use_sim_time': use_sim_time
            }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_serial_port_cmd,
        declare_baud_rate_cmd,
        stm32_interface_node
    ])
