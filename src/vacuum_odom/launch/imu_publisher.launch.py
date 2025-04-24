#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_simulator = LaunchConfiguration('use_simulator', default='true')
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    pub_rate = LaunchConfiguration('pub_rate', default='50.0')
    
    # 创建启动参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='true',
        description='使用IMU模拟器数据')
    
    declare_imu_frame_cmd = DeclareLaunchArgument(
        'imu_frame',
        default_value='imu_link',
        description='IMU坐标系名称')
    
    declare_pub_rate_cmd = DeclareLaunchArgument(
        'pub_rate',
        default_value='50.0',
        description='IMU数据发布频率(Hz)')
    
    # IMU发布节点
    imu_publisher_node = Node(
        package='vacuum_odom',
        executable='imu_publisher_node.py',
        name='imu_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_simulator': use_simulator,
            'imu_frame': imu_frame,
            'pub_rate': pub_rate
        }]
    )
    
    # IMU坐标系与base_link之间的静态变换
    imu_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_link',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    # 返回启动描述
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_simulator_cmd,
        declare_imu_frame_cmd,
        declare_pub_rate_cmd,
        imu_publisher_node,
        imu_to_base_link_node
    ])