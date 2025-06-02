#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class VelocityConverterNode(Node):
    def __init__(self):
        super().__init__('velocity_converter_node')
        
        # 声明参数
        self.declare_parameter('linear_threshold', 0.02)  # 线速度阈值
        self.declare_parameter('angular_threshold', 0.1)  # 角速度阈值
        self.declare_parameter('max_linear_vel', 0.26)    # 最大线速度
        self.declare_parameter('max_angular_vel', 1.0)    # 最大角速度
        
        # 获取参数
        self.linear_threshold = self.get_parameter('linear_threshold').get_parameter_value().double_value
        self.angular_threshold = self.get_parameter('angular_threshold').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        
        # 订阅导航系统发布的/cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # 发布处理后的速度指令
        self.cmd_vel_processed_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_processed',
            10)
        
        self.get_logger().info("Velocity Converter Node initialized")
        self.get_logger().info(f"Linear threshold: {self.linear_threshold}")
        self.get_logger().info(f"Angular threshold: {self.angular_threshold}")
        self.get_logger().info(f"Max linear velocity: {self.max_linear_vel}")
        self.get_logger().info(f"Max angular velocity: {self.max_angular_vel}")

    def cmd_vel_callback(self, msg):
        """
        处理导航系统发布的速度指令，转换为适合两轮差速机器人的格式
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 创建输出消息
        output_msg = Twist()
        
        # 速度转换逻辑
        converted_linear, converted_angular = self.convert_velocity(linear_x, angular_z)
        
        output_msg.linear.x = converted_linear
        output_msg.linear.y = 0.0
        output_msg.linear.z = 0.0
        output_msg.angular.x = 0.0
        output_msg.angular.y = 0.0
        output_msg.angular.z = converted_angular
        
        # 发布转换后的速度指令
        self.cmd_vel_processed_publisher.publish(output_msg)
        
        # 记录转换信息
        if abs(linear_x) > 0.001 or abs(angular_z) > 0.001:
            self.get_logger().info(
                f"Input: ({linear_x:.3f}, {angular_z:.3f}) -> "
                f"Output: ({converted_linear:.3f}, {converted_angular:.3f})"
            )

    def convert_velocity(self, linear_x, angular_z):
        """
        将连续的速度值转换为离散的控制指令
        返回格式：(线速度, 角速度)
        - (1, 0) - 前进
        - (-1, 0) - 后退  
        - (0, 1) - 左转
        - (0, -1) - 右转
        - (0, 0) - 停止
        """
        
        # 如果速度都很小，则停止
        if abs(linear_x) < self.linear_threshold and abs(angular_z) < self.angular_threshold:
            return 0.0, 0.0
        
        # 优先处理角速度（转向）
        if abs(angular_z) > self.angular_threshold:
            if abs(angular_z) > abs(linear_x):  # 角速度占主导
                if angular_z > 0:
                    return 0.0, 1.0  # 左转
                else:
                    return 0.0, -1.0  # 右转
        
        # 处理线速度（前进/后退）
        if abs(linear_x) > self.linear_threshold:
            if linear_x > 0:
                return 1.0, 0.0  # 纯前进
            else:
                return -1.0, 0.0  # 纯后退
        
        # 默认情况（理论上不应该到达这里）
        return 0.0, 0.0

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VelocityConverterNode()
        node.get_logger().info("Velocity Converter Node started, spinning...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
    except Exception as e:
        if 'node' in locals():
            node.get_logger().error(f"Unhandled exception: {e}", exc_info=True)
        else:
            print(f"Failed during node creation: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Velocity Converter Node shutdown complete.")

if __name__ == '__main__':
    main()
