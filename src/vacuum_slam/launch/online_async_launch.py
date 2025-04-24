# 真空吸尘器导航系统 - SLAM启动文件
# 功能：启动雷达、里程计、IMU、EKF融合和SLAM节点

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取各功能包的共享目录路径
    pkg_ldlidar = get_package_share_directory('ldlidar_sl_ros2')  # 雷达驱动包
    pkg_vacuum_odom = get_package_share_directory('vacuum_odom')  # 里程计包
    pkg_vacuum_slam = get_package_share_directory('vacuum_slam')  # SLAM包
    pkg_vacuum_bluetooth = get_package_share_directory('vacuum_bluetooth') # 与手机蓝牙通信的包

    # 使用仿真时间参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 参数声明
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间 (true/false)'
    )

    # 声明里程计类型参数
    declare_odom_type = DeclareLaunchArgument(
        'odom_type',
        default_value='rf2o',  # 现在默认使用RF2O
        description='里程计类型: simple(简易实现), enhanced(增强实现) 或 rf2o(RF2O库)'
    )
    odom_type = LaunchConfiguration('odom_type')

    # 1. 包含雷达启动文件 (根据实际雷达型号选择ld14或ld14p)
    lidar_launch_file = os.path.join(pkg_ldlidar, 'launch', 'ld14.launch.py')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file)
    )

    # 2A. RF2O 激光里程计节点
    rf2o_laser_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,                # 不发布TF，由robot_localization负责
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',         # 不从其他话题获取初始位置
            'freq': 20.0                        # 发布频率
        }]
    )
    
    # 2B. 自定义激光里程计节点 (根据odom_type参数决定是否启动)
    custom_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vacuum_odom, 'launch', 'odom_publisher.launch.py')),
        launch_arguments={
            'odom_topic': 'odom_laser',
            'scan_topic': 'scan',
            'odom_type': odom_type
        }.items()
    )
    
    # 3. 包含IMU启动文件 (发布 /imu/data 消息)
    imu_launch_file = os.path.join(pkg_vacuum_odom, 'launch', 'imu_publisher.launch.py')
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. 添加 robot_localization EKF 节点 - 融合多个传感器数据
    ekf_config_file = os.path.join(pkg_vacuum_slam, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # EKF的输入为雷达里程计和IMU
            ('/odom0', '/odom_rf2o'),  # 使用RF2O产生的里程计
            ('/imu0', '/imu/data'),
            # EKF的输出重映射为标准的/odom话题
            ('/odometry/filtered', '/odom')
        ]
    )

    # 5. SLAM Toolbox 异步建图节点配置
    slam_toolbox_node = Node(
        package='slam_toolbox',  # 使用slam_toolbox功能包
        executable='async_slam_toolbox_node',  # 异步SLAM节点
        name='slam_toolbox',  # 节点名称
        output='screen',  # 输出到屏幕
        parameters=[
            os.path.join(pkg_vacuum_slam, 'config', 'mapper_params_online_async.yaml'),  # SLAM参数配置文件
            {'use_sim_time': use_sim_time} # 使用统一的 use_sim_time 参数
        ],
        # SLAM Toolbox 将通过 TF 监听 /odom -> base_link 的变换，无需直接重映射 /odom 话题
    )

    # 6. (可选) RViz2可视化节点配置
    rviz_config_file = os.path.join(pkg_vacuum_slam, 'rviz', 'slam_config.rviz')  # RViz配置文件路径
    rviz_node = Node(
        package='rviz2',  # RViz2功能包
        executable='rviz2',  # RViz可执行文件
        name='rviz2',  # 节点名称
        arguments=['-d', rviz_config_file],  # 指定配置文件
        output='screen',  # 输出到屏幕
        parameters=[{'use_sim_time': use_sim_time}] # 与SLAM节点时间设置保持一致
    )

    # 7. Bluetooth Node
    bluetooth_comm_node = Node(
        package='vacuum_bluetooth', # 蓝牙通信包
        executable='bluetooth_node', # 蓝牙通信节点
        name='bluetooth_communicator',
        output='screen'
    )

    # 返回包含所有节点的启动描述
    return LaunchDescription([
        # 参数声明
        declare_use_sim_time,
        declare_odom_type,
        
        # 节点启动
        lidar_launch,           # 雷达节点 (发布 /scan 消息)
        rf2o_laser_odometry_node,  # RF2O激光里程计节点 (发布 /odom_rf2o 消息)
        # custom_odom_launch 不再使用，改为RF2O
        imu_launch,             # IMU节点 (发布 /imu/data 消息)
        ekf_node,               # EKF 融合节点 (融合里程计和IMU数据，发布 /odom 和 TF odom->base_link)
        slam_toolbox_node,      # SLAM节点 (使用 EKF 提供的 /odom 和 TF，以及激光扫描构建地图)
        bluetooth_comm_node,    # 蓝牙通信节点
        # rviz_node               # 可视化节点
    ])
