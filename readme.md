# 🤖智能吸尘器机器人导航系统

本项目是一个基于ROS2的完整智能扫地机器人系统，集成了SLAM建图、自主导航、远程控制和WiFi通信功能。

## 🚀 系统概述

本项目是一个完整的扫地机器人解决方案，运行在树莓派上，具备以下核心功能：

- **实时SLAM建图**：使用SLAM Toolbox进行边建图边导航
- **自主导航**：基于Nav2导航栈的路径规划和避障
- **多种控制方式**：支持rviz2目标点、键盘控制、WiFi远程遥控
- **传感器融合**：EKF融合IMU和轮式里程计数据
- **硬件接口**：STM32微控制器通信、激光雷达驱动
- **远程监控**：WiFi地图数据传输和远程控制

## 📁 项目结构

```
Vacuum_Cleaner_nav/
├── src/
│   ├── ldlidar_sl_ros2/           # 激光雷达驱动包
│   ├── vacuum_control/            # 键盘控制包
│   ├── vacuum_describe/           # 机器人描述和URDF
│   ├── vacuum_fusion/             # EKF传感器融合
│   ├── vacuum_interface/          # STM32硬件接口
│   ├── vacuum_nav/                # 导航和SLAM核心包
│   ├── vacuum_slam/               # SLAM配置和地图导出
│   └── vacuum_wifi/               # WiFi通信包
├── launch/                        # 全局启动文件
├── build/                         # 编译输出目录
├── install/                       # 安装目录
└── readme.md                      # 本文档
```

## 🏗️ 系统架构

### 硬件层
- **激光雷达**：LD14 360度激光雷达，提供环境感知
- **STM32微控制器**：处理电机控制、IMU数据采集和轮式里程计
- **树莓派4B**：主控制器，运行ROS2系统
- **WiFi模块**：无线通信和远程控制

### 软件架构
![系统架构图](/doc/system_architecture.png)

## 🚀 快速开始

### 环境要求
- **操作系统**：Ubuntu 22.04 LTS
- **ROS版本**：ROS2 Humble
- **硬件**：树莓派4B（推荐4GB+）
- **Python版本**：3.10+

### 依赖安装

```bash
# 安装ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装导航相关包
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-localization

# 安装其他依赖
sudo apt install python3-tf-transformations
sudo apt install python3-serial
```

### 编译项目

```bash
# 克隆项目
git clone https://github.com/Deicedmilktea/Vacuum_Cleaner_nav.git
cd Vacuum_Cleaner_nav

# 编译所有包
colcon build

# 设置环境变量
source install/setup.bash
```

### 启动系统

#### 方式1：完整系统启动（推荐）
```bash
# 启动完整的SLAM+导航系统
ros2 launch vacuum_nav vacuum_slam_nav_launch.py
```

#### 方式2：分步启动
```bash
# 1. 启动建图和传感器驱动
ros2 launch launch/mapping_launch.py

# 2. 启动导航
ros2 launch vacuum_nav navigation_launch.py
```

### 远程控制设置

#### 启动rviz2可视化
```bash
# 在另一台设备上启动rviz2
ros2 run rviz2 rviz2
```

**rviz2配置**：
1. 设置Fixed Frame为"map"
2. 添加显示项：
   - Map (topic: `/map`)
   - LaserScan (topic: `/scan`)
   - TF
   - Path (topic: `/plan`)
   - Costmap (topics: `/local_costmap/costmap`, `/global_costmap/costmap`)
   - goal_pose 使用`2D Pose Estimate`工具设置目标点

#### 键盘控制
```bash
# 启动键盘控制节点
ros2 launch vacuum_control keyboard_control_launch.py
```

#### WiFi远程控制
```bash
# WiFi通信已集成在主启动文件中
# 客户端连接示例（Python）
import socket

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(('树莓派IP', 8080))

# 发送速度命令
client.send(b'v=1,0')   # 前进
client.send(b'v=-1,0')  # 后退
client.send(b'v=0,1')   # 左转
client.send(b'v=0,-1')  # 右转
client.send(b'v=0,0')   # 停止
```

## 📊 话题和服务

### 主要话题

| 话题名称 | 消息类型 | 方向 | 描述 |
|---------|---------|------|------|
| `/scan` | sensor_msgs/LaserScan | 发布 | 激光雷达数据 |
| `/imu/data` | sensor_msgs/Imu | 发布 | IMU数据 |
| `/wheel_odom` | nav_msgs/Odometry | 发布 | 轮式里程计 |
| `/odom` | nav_msgs/Odometry | 发布 | EKF融合里程计 |
| `/map` | nav_msgs/OccupancyGrid | 发布 | SLAM地图 |
| `/tf` | tf2_msgs/TFMessage | 发布 | 坐标系变换 |
| `/cmd_vel` | geometry_msgs/Twist | 订阅 | Nav2速度命令 |
| `/cmd_vel_processed` | geometry_msgs/Twist | 发布 | 处理后速度命令 |
| `/goal_pose` | geometry_msgs/PoseStamped | 订阅 | rviz2目标点 |
| `/plan` | nav_msgs/Path | 发布 | 路径规划结果 |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | 发布 | 局部代价地图 |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | 发布 | 全局代价地图 |

### 坐标系变换

```
          [map]
            |
          [odom]
            |
        [base_link]
         /       \
 [imu_link]   [lidar_link]
```

## 🔮 扩展功能

### 已规划功能
- [ ] 自动充电桩对接
- [ ] 多房间清扫规划
- [ ] 语音控制接口
- [ ] 移动端APP控制
- [ ] 云端地图同步
- [ ] 多机器人协作

### 开发建议
- 使用行为树进行任务规划
- 集成深度学习目标检测
- 添加语义地图功能
- 实现动态避障算法
- 支持多传感器数据融合

## 🤝 致谢

感谢以下开源项目的支持：
- [ROS2](https://ros.org/)
- [Nav2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
