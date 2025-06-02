#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    linear_threshold_arg = DeclareLaunchArgument(
        'linear_threshold',
        default_value='0.05',
        description='Linear velocity threshold for conversion'
    )
    
    angular_threshold_arg = DeclareLaunchArgument(
        'angular_threshold',
        default_value='0.1',
        description='Angular velocity threshold for conversion'
    )
    
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.26',
        description='Maximum linear velocity'
    )
    
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.0',
        description='Maximum angular velocity'
    )

    # 速度转换节点
    velocity_converter_node = Node(
        package='vacuum_nav',
        executable='velocity_converter_node',
        name='velocity_converter_node',
        output='screen',
        parameters=[{
            'linear_threshold': LaunchConfiguration('linear_threshold'),
            'angular_threshold': LaunchConfiguration('angular_threshold'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
        }],
        remappings=[
            # 可以在这里添加话题重映射，如果需要的话
        ]
    )

    return LaunchDescription([
        linear_threshold_arg,
        angular_threshold_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        velocity_converter_node,
    ])
