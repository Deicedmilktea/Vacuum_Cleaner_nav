# 真空吸尘器导航系统 - SLAM启动文件
# 功能：启动雷达、里程计和SLAM节点

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取各功能包的共享目录路径
    pkg_ldlidar = get_package_share_directory('ldlidar_sl_ros2')  # 雷达驱动包
    pkg_vacuum_odom = get_package_share_directory('vacuum_odom')  # 里程计包
    pkg_vacuum_slam = get_package_share_directory('vacuum_slam')  # SLAM包

    # 1. 包含雷达启动文件 (根据实际雷达型号选择ld14或ld14p)
    lidar_launch_file = os.path.join(pkg_ldlidar, 'launch', 'ld14.launch.py')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file)
    )

    # 2. 包含里程计启动文件
    odom_launch_file = os.path.join(pkg_vacuum_odom, 'launch', 'odom_publisher.launch.py')
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odom_launch_file)
    )

    # 3. SLAM Toolbox 异步建图节点配置
    slam_toolbox_node = Node(
        package='slam_toolbox',  # 使用slam_toolbox功能包
        executable='async_slam_toolbox_node',  # 异步SLAM节点
        name='slam_toolbox',  # 节点名称
        output='screen',  # 输出到屏幕
        parameters=[
            os.path.join(pkg_vacuum_slam, 'config', 'mapper_params_online_async.yaml'),  # SLAM参数配置文件
            {'use_sim_time': False}  # 是否使用仿真时间，实际硬件设为False
        ],
        # 可选的topic重映射，如里程计使用不同的tf话题时
        # remappings=[
        #     ('/tf', '/odom_tf')
        # ]
    )

    # 4. (可选) RViz2可视化节点配置
    rviz_config_file = os.path.join(pkg_vacuum_slam, 'rviz', 'slam_config.rviz')  # RViz配置文件路径
    rviz_node = Node(
        package='rviz2',  # RViz2功能包
        executable='rviz2',  # RViz可执行文件
        name='rviz2',  # 节点名称
        arguments=['-d', rviz_config_file],  # 指定配置文件
        output='screen',  # 输出到屏幕
        parameters=[{'use_sim_time': False}]  # 与SLAM节点时间设置保持一致
    )

    # 返回包含所有节点的启动描述
    return LaunchDescription([
        lidar_launch,      # 雷达节点
        odom_launch,       # 里程计节点  
        slam_toolbox_node,  # SLAM节点
        # rviz_node        # 取消注释以自动启动RViz
    ])
