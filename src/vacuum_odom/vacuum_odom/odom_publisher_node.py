#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion
import math
import numpy as np
from builtin_interfaces.msg import Time

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w, x, y, z)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

# 定义激光里程计抽象基类
class BaseLaserOdometry:
    """雷达里程计基类，定义接口方法"""
    def __init__(self, node):
        self.node = node
        # 每个子类应当实现这些属性
        self.linear_vel_x = 0.0
        self.linear_vel_y = 0.0
        self.angular_vel = 0.0

    def process_scan(self, scan_msg):
        """处理激光扫描数据，子类需要重写此方法"""
        raise NotImplementedError("子类必须实现process_scan方法")

    def get_velocity(self):
        """获取当前速度估计"""
        return self.linear_vel_x, self.linear_vel_y, self.angular_vel

# 简化版激光里程计实现
class SimpleLaserOdometry(BaseLaserOdometry):
    """简化的激光里程计实现，使用帧间点云匹配"""
    def __init__(self, node):
        super().__init__(node)
        self.prev_scan = None
        self.node.get_logger().info("初始化简化版激光里程计")

    def process_scan(self, scan_msg):
        if self.prev_scan is None:
            self.prev_scan = scan_msg
            return False
        
        # 计算两次扫描之间的时间差
        delta_t = (scan_msg.header.stamp.sec - self.prev_scan.header.stamp.sec) + \
                  (scan_msg.header.stamp.nanosec - self.prev_scan.header.stamp.nanosec) / 1e9
        
        if delta_t <= 0.0:
            return False
            
        # 获取有效的点
        prev_ranges = np.array(self.prev_scan.ranges)
        curr_ranges = np.array(scan_msg.ranges)
        
        # 过滤无效点
        min_range = max(self.prev_scan.range_min, scan_msg.range_min)
        max_range = min(self.prev_scan.range_max, scan_msg.range_max)
        
        valid_prev = (prev_ranges > min_range) & (prev_ranges < max_range)
        valid_curr = (curr_ranges > min_range) & (curr_ranges < max_range)
        valid_both = valid_prev & valid_curr
        
        # 如果没有足够的有效点，则跳过
        if np.sum(valid_both) < 10:
            self.node.get_logger().warning("激光点不足 (< 10)，跳过此帧")
            self.prev_scan = scan_msg
            return False
            
        # 使用简单的距离变化评估运动
        angle_increment = scan_msg.angle_increment
        angles = np.arange(len(curr_ranges)) * angle_increment + scan_msg.angle_min
        
        # 转换为笛卡尔坐标
        x_prev = prev_ranges[valid_both] * np.cos(angles[valid_both])
        y_prev = prev_ranges[valid_both] * np.sin(angles[valid_both])
        
        x_curr = curr_ranges[valid_both] * np.cos(angles[valid_both])
        y_curr = curr_ranges[valid_both] * np.sin(angles[valid_both])
        
        # 计算平均位移
        dx = np.mean(x_curr - x_prev)
        dy = np.mean(y_curr - y_prev)
        
        # 估计转向角
        dtheta = 0.0
        if np.sum(valid_both) > 20:
            try:
                # 使用叉积和点积的比值估计旋转角
                angle_diff = np.arctan2(np.sum((x_prev * y_curr - y_prev * x_curr)), 
                                       np.sum((x_prev * x_curr + y_prev * y_curr)))
                dtheta = angle_diff
            except:
                self.node.get_logger().warning("角度估算失败")
                dtheta = 0.0
        
        # 更新速度估计
        self.linear_vel_x = dx / delta_t
        self.linear_vel_y = dy / delta_t
        self.angular_vel = dtheta / delta_t
        
        # 保存当前扫描
        self.prev_scan = scan_msg
        return True

# 增强型激光里程计（RF2O风格）
class EnhancedLaserOdometry(BaseLaserOdometry):
    """增强的激光里程计实现，模拟RF2O算法"""
    def __init__(self, node):
        super().__init__(node)
        self.prev_scan = None
        self.prev_points = None
        self.node.get_logger().info("初始化增强版激光里程计 (RF2O风格)")
        self.window_size = 5  # 滑动窗口大小
        self.motion_history = []  # 运动历史

    def preprocess_scan(self, scan_msg):
        """预处理扫描数据，滤波并转换为笛卡尔坐标系中的点"""
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(len(ranges)) * scan_msg.angle_increment + scan_msg.angle_min
        
        # 过滤无效点
        valid = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
        
        if np.sum(valid) < 30:  # 需要足够的点
            return None
            
        # 转换为笛卡尔坐标
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        
        return np.column_stack((x, y))

    def estimate_motion_icp(self, prev_points, curr_points):
        """使用简化的ICP算法估计运动"""
        # 计算点云质心
        prev_centroid = np.mean(prev_points, axis=0)
        curr_centroid = np.mean(curr_points, axis=0)
        
        # 去中心化
        prev_centered = prev_points - prev_centroid
        curr_centered = curr_points - curr_centroid
        
        # 计算旋转矩阵的最佳估计
        H = np.dot(prev_centered.T, curr_centered)
        U, _, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # 确保是有效的旋转矩阵 (det(R) = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # 计算平移向量
        t = curr_centroid - np.dot(prev_centroid, R.T)
        
        # 计算旋转角度
        theta = np.arctan2(R[1, 0], R[0, 0])
        
        return t[0], t[1], theta
        
    def process_scan(self, scan_msg):
        curr_points = self.preprocess_scan(scan_msg)
        if curr_points is None:
            return False
            
        if self.prev_points is None:
            self.prev_points = curr_points
            self.prev_scan = scan_msg
            return False
            
        # 计算时间差
        delta_t = (scan_msg.header.stamp.sec - self.prev_scan.header.stamp.sec) + \
                  (scan_msg.header.stamp.nanosec - self.prev_scan.header.stamp.nanosec) / 1e9
        
        if delta_t <= 0.0:
            return False
            
        try:
            # 估计运动
            dx, dy, dtheta = self.estimate_motion_icp(self.prev_points, curr_points)
            
            # 更新速度估计
            self.linear_vel_x = dx / delta_t
            self.linear_vel_y = dy / delta_t
            self.angular_vel = dtheta / delta_t
            
            # 将运动估计添加到历史记录
            self.motion_history.append((self.linear_vel_x, self.linear_vel_y, self.angular_vel))
            if len(self.motion_history) > self.window_size:
                self.motion_history.pop(0)
                
            # 平滑运动估计，减少噪声影响
            if len(self.motion_history) > 1:
                smoothed = np.mean(self.motion_history, axis=0)
                self.linear_vel_x = smoothed[0]
                self.linear_vel_y = smoothed[1]
                self.angular_vel = smoothed[2]
                
            # 更新前一帧数据
            self.prev_points = curr_points
            self.prev_scan = scan_msg
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"估计运动时错误: {e}")
            self.prev_points = curr_points
            self.prev_scan = scan_msg
            return False

class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher_node')

        # 参数声明
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pub_rate', 10.0) # 发布频率 (Hz)
        self.declare_parameter('scan_topic', 'scan') # 激光雷达数据话题
        self.declare_parameter('odom_topic', 'odom_laser') # 输出的里程计话题名称
        self.declare_parameter('odom_type', 'simple') # 里程计类型: simple 或 rf2o

        # 获取参数
        self.odom_frame_ = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame_ = self.get_parameter('base_frame').get_parameter_value().string_value
        pub_rate = self.get_parameter('pub_rate').get_parameter_value().double_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        odom_type = self.get_parameter('odom_type').get_parameter_value().string_value

        # 初始化所选类型的里程计
        if odom_type == 'rf2o':
            self.get_logger().info("使用增强型激光里程计 (RF2O风格)")
            self.laser_odom = EnhancedLaserOdometry(self)
        else:
            self.get_logger().info("使用简化版激光里程计")
            self.laser_odom = SimpleLaserOdometry(self)

        # 创建 Odometry 发布者
        self.odom_pub_ = self.create_publisher(Odometry, odom_topic, 10)
        
        # 创建雷达数据订阅者
        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        
        # 节点状态变量
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.last_time_ = self.get_clock().now()

        # 创建定时器，周期性调用 publish_odom 方法
        self.timer_ = self.create_timer(1.0 / pub_rate, self.publish_odom)

        self.get_logger().info(f"激光里程计启动完成。发布话题: {odom_topic}, 频率: {pub_rate} Hz")
        self.get_logger().info(f"订阅雷达数据话题: {scan_topic}")

    def scan_callback(self, msg):
        """处理雷达数据，调用里程计算法"""
        self.laser_odom.process_scan(msg)

    def publish_odom(self):
        """发布里程计数据"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time_).nanoseconds / 1e9
        
        if dt <= 0.0:
            return

        # 获取里程计算法估计的速度
        vx, vy, vth = self.laser_odom.get_velocity()

        # 更新位姿估计
        self.x_ += (vx * math.cos(self.theta_) - 
                   vy * math.sin(self.theta_)) * dt
        self.y_ += (vx * math.sin(self.theta_) + 
                   vy * math.cos(self.theta_)) * dt
        self.theta_ += vth * dt

        # 准备 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_
        odom_msg.child_frame_id = self.base_frame_

        # 设置位置
        odom_msg.pose.pose.position.x = self.x_
        odom_msg.pose.pose.position.y = self.y_
        odom_msg.pose.pose.position.z = 0.0  # 假设是 2D 运动
        odom_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self.theta_)

        # 设置速度
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vth

        # 设置协方差 - 这对于robot_localization融合很重要
        # 位置协方差
        odom_msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        
        # 速度协方差
        odom_msg.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        # 发布 Odometry 消息
        self.odom_pub_.publish(odom_msg)

        # 更新上次时间
        self.last_time_ = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()