from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明参数
    odom_topic = LaunchConfiguration('odom_topic', default='odom_laser')
    scan_topic = LaunchConfiguration('scan_topic', default='scan')
    pub_rate = LaunchConfiguration('pub_rate', default='10.0')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    base_frame = LaunchConfiguration('base_frame', default='base_link')
    odom_type = LaunchConfiguration('odom_type', default='simple')  # 添加odom_type参数
    
    # 参数声明
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='odom_laser',
        description='输出的里程计话题名称')
        
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='激光雷达数据话题')
        
    declare_pub_rate = DeclareLaunchArgument(
        'pub_rate',
        default_value='10.0',
        description='发布频率 (Hz)')
        
    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='里程计坐标系名称')
        
    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='机器人基准坐标系名称')
        
    declare_odom_type = DeclareLaunchArgument(
        'odom_type',
        default_value='simple',
        description='里程计类型: simple(简易实现) 或 rf2o(专业实现)')
    
    return LaunchDescription([
        # 声明参数
        declare_odom_topic,
        declare_scan_topic,
        declare_pub_rate,
        declare_odom_frame,
        declare_base_frame,
        declare_odom_type,
        
        # 里程计节点
        Node(
            package='vacuum_odom',
            executable='odom_publisher_node', 
            name='laser_odom_publisher',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'scan_topic': scan_topic,
                'pub_rate': pub_rate,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'odom_type': odom_type
            }]
        )
    ])
