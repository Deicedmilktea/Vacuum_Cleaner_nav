#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import math
import numpy as np
from builtin_interfaces.msg import Time
import serial

def quaternion_from_euler(roll, pitch, yaw):
    """
    将欧拉角转换为四元数
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    
    return q

class IMUPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')

        # 参数 (可通过launch文件或命令行修改)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('pub_rate', 50.0) # 发布频率 (Hz)
        self.declare_parameter('use_simulator', True)  # 是否使用模拟器数据

        self.imu_frame_ = self.get_parameter('imu_frame').get_parameter_value().string_value
        pub_rate = self.get_parameter('pub_rate').get_parameter_value().double_value
        self.use_simulator_ = self.get_parameter('use_simulator').get_parameter_value().bool_value
        
        # 创建IMU数据发布者
        self.imu_pub_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # 创建定时器，周期性调用publish_imu方法
        self.timer_ = self.create_timer(1.0 / pub_rate, self.publish_imu)
        
        # IMU仿真相关变量
        self.yaw_ = 0.0
        self.yaw_rate_ = 0.01  # 弧度/秒
        self.linear_accel_x_ = 0.0
        self.linear_accel_y_ = 0.0
        self.linear_accel_noise_ = 0.05  # 加速度噪声
        self.angular_vel_noise_ = 0.01   # 角速度噪声
        
        # IMU误差相关系数
        self.orientation_covariance_ = [0.01, 0, 0,  # 方向协方差
                                      0, 0.01, 0,
                                      0, 0, 0.01]
        self.angular_velocity_covariance_ = [0.01, 0, 0,  # 角速度协方差
                                           0, 0.01, 0,
                                           0, 0, 0.01]
        self.linear_acceleration_covariance_ = [0.04, 0, 0,  # 线性加速度协方差
                                              0, 0.04, 0,
                                              0, 0, 0.04]
        
        self.get_logger().info(f"IMU publisher started. Publishing to /imu/data at {pub_rate} Hz")
        
    def get_simulated_imu_data(self):
        """
        生成模拟的IMU数据
        在真实项目中，这部分应当从实际的IMU传感器读取
        """
        # 更新航向
        self.yaw_ += self.yaw_rate_ * 0.02  # 假设间隔20ms
        if self.yaw_ > 2*math.pi:
            self.yaw_ -= 2*math.pi
            
        # 模拟机器人线性加速度 (加上一些随机噪声)
        self.linear_accel_x_ = 0.2 + np.random.normal(0, self.linear_accel_noise_)
        self.linear_accel_y_ = 0.0 + np.random.normal(0, self.linear_accel_noise_)
        
        # 模拟角速度 (加上一些随机噪声)
        angular_vel_z = self.yaw_rate_ + np.random.normal(0, self.angular_vel_noise_)
        
        return {
            'orientation': quaternion_from_euler(0.0, 0.0, self.yaw_),
            'angular_velocity': Vector3(x=0.0, y=0.0, z=angular_vel_z),
            'linear_acceleration': Vector3(
                x=self.linear_accel_x_,
                y=self.linear_accel_y_,
                z=9.81  # 重力加速度
            )
        }
        
    def read_real_imu_data(self):
        """
        从实际IMU传感器读取数据
        这里通过串口从STM32获取IMU数据
        假设STM32通过串口发送格式为: roll,pitch,yaw,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n
        单位分别为: 弧度, 弧度, 弧度, rad/s, rad/s, rad/s, m/s^2, m/s^2, m/s^2
        """

        # 串口参数根据实际情况修改
        SERIAL_PORT = '/dev/ttyUSB1'
        BAUDRATE = 115200

        try:
            if not hasattr(self, 'ser'):
                self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                raise ValueError("No data received from serial")
            parts = line.split(',')
            if len(parts) != 9:
                raise ValueError("Invalid IMU data format")
            roll, pitch, yaw = map(float, parts[0:3])
            gyro_x, gyro_y, gyro_z = map(float, parts[3:6])
            accel_x, accel_y, accel_z = map(float, parts[6:9])

            return {
                'orientation': quaternion_from_euler(roll, pitch, yaw),
                'angular_velocity': Vector3(x=gyro_x, y=gyro_y, z=gyro_z),
                'linear_acceleration': Vector3(x=accel_x, y=accel_y, z=accel_z)
            }
        except Exception as e:
            self.get_logger().warn(f"Failed to read IMU from serial: {e}")
            # 返回上一次数据或默认数据
            return {
                'orientation': quaternion_from_euler(0.0, 0.0, 0.0),
                'angular_velocity': Vector3(x=0.0, y=0.0, z=0.0),
                'linear_acceleration': Vector3(x=0.0, y=0.0, z=9.81)
            }
        
    def publish_imu(self):
        """
        发布IMU数据
        """
        # 根据设置选择模拟数据或实际数据
        if self.use_simulator_:
            imu_data = self.get_simulated_imu_data()
        else:
            imu_data = self.read_real_imu_data()
            
        # 创建并填充IMU消息
        imu_msg = Imu()
        
        # 设置头部信息
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_
        
        # 设置方向(四元数)
        imu_msg.orientation = imu_data['orientation']
        imu_msg.orientation_covariance = self.orientation_covariance_
        
        # 设置角速度
        imu_msg.angular_velocity = imu_data['angular_velocity']
        imu_msg.angular_velocity_covariance = self.angular_velocity_covariance_
        
        # 设置线性加速度
        imu_msg.linear_acceleration = imu_data['linear_acceleration']
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance_
        
        # 发布IMU消息
        self.imu_pub_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()