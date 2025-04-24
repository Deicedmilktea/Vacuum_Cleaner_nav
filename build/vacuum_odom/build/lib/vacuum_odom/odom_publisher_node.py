#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
# from tf2_ros import TransformBroadcaster #不再需要广播TF
import math
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

class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher_node')

        # 参数 (可以后续通过 launch 文件或命令行修改)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pub_rate', 10.0) # 发布频率 (Hz)

        self.odom_frame_ = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame_ = self.get_parameter('base_frame').get_parameter_value().string_value
        pub_rate = self.get_parameter('pub_rate').get_parameter_value().double_value

        # 创建 Odometry 发布者
        self.odom_pub_ = self.create_publisher(Odometry, '/odom', 10)
        # 不再创建 TF 广播者
        # self.tf_broadcaster_ = TransformBroadcaster(self)

        # 节点状态变量 (用于模拟运动)
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        # 模拟速度 (这些值最终需要从 STM32 获取)
        self.linear_vel_ = 0.01  # 米/秒 (假设向前)
        self.angular_vel_ = 0.01  # 弧度/秒 (假设轻微左转)

        self.last_time_ = self.get_clock().now()

        # 创建定时器，周期性调用 publish_odom 方法
        self.timer_ = self.create_timer(1.0 / pub_rate, self.publish_odom)

        self.get_logger().info(f"Odometry publisher started. Publishing Odom message to /odom at {pub_rate} Hz.")

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time_).nanoseconds / 1e9 # 计算时间差，单位：秒

        # --- 更新机器人位姿 (简单的欧拉积分，实际应使用 STM32 数据) ---
        # 这里用固定的速度来模拟运动，你需要用 STM32 传来的实际速度替换
        delta_x = (self.linear_vel_ * math.cos(self.theta_)) * dt
        delta_y = (self.linear_vel_ * math.sin(self.theta_)) * dt
        delta_theta = self.angular_vel_ * dt

        self.x_ += delta_x
        self.y_ += delta_y
        self.theta_ += delta_theta
        # --- 结束模拟更新 ---

        # 准备 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_
        odom_msg.child_frame_id = self.base_frame_

        # 设置位置
        odom_msg.pose.pose.position.x = self.x_
        odom_msg.pose.pose.position.y = self.y_
        odom_msg.pose.pose.position.z = 0.0 # 假设是 2D 运动
        odom_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self.theta_)

        # 设置速度 ( twist )
        odom_msg.twist.twist.linear.x = self.linear_vel_ # 实际应是从STM32读取的线速度
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel_ # 实际应是从STM32读取的角速度

        # 发布 Odometry 消息
        self.odom_pub_.publish(odom_msg)

        # 不再准备和广播 TF 变换
        # # 准备 TF 变换消息 (odom -> base_link)
        # t = TransformStamped()
        # t.header.stamp = current_time.to_msg()
        # t.header.frame_id = self.odom_frame_
        # t.child_frame_id = self.base_frame_
        #
        # # 设置 TF 的平移
        # t.transform.translation.x = self.x_
        # t.transform.translation.y = self.y_
        # t.transform.translation.z = 0.0
        #
        # # 设置 TF 的旋转 (与 odom 消息中的 pose 一致)
        # t.transform.rotation = odom_msg.pose.pose.orientation # 直接使用上面计算好的四元数
        #
        # # 广播 TF 变换
        # self.tf_broadcaster_.sendTransform(t)

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