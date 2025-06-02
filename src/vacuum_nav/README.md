# Vacuum Navigation Package

这个包实现了吸尘器机器人的边建图边导航功能，使用SLAM Toolbox进行建图，Nav2进行导航。

## 功能特性

- 使用SLAM Toolbox进行实时建图
- 使用Nav2进行路径规划和导航
- 支持从rviz2接收目标点
- 边建图边导航功能
- 适配树莓派运行环境

## 包结构

```
vacuum_nav/
├── config/
│   └── nav2_params.yaml          # Nav2参数配置
├── launch/
│   ├── navigation_launch.py      # 纯导航launch文件
│   ├── slam_navigation_launch.py # SLAM+导航launch文件（复杂版本）
│   ├── simple_slam_nav_launch.py # SLAM+导航launch文件（简单版本，推荐）
│   ├── vacuum_slam_nav_launch.py # 基础系统launch文件
│   ├── complete_system_launch.py # 完整系统launch文件（推荐）
│   └── goal_receiver_launch.py   # 目标点接收节点launch文件
├── vacuum_nav/
│   └── goal_receiver_node.py     # 目标点接收节点
└── README.md
```

## 依赖包

- nav2_bringup
- slam_toolbox
- nav2_msgs
- geometry_msgs
- tf2_ros
- tf2_geometry_msgs

## 使用方法

### 1. 编译包

```bash
cd /path/to/your/workspace
colcon build --packages-select vacuum_nav
source install/setup.bash
```

### 2. 启动完整的边建图边导航系统

在树莓派上运行（推荐使用完整系统）：

```bash
ros2 launch vacuum_nav complete_system_launch.py
```

或者使用基础版本（不包含目标点接收节点）：

```bash
ros2 launch vacuum_nav vacuum_slam_nav_launch.py
```

这个命令会启动：
- 激光雷达驱动
- STM32接口
- 机器人描述
- EKF传感器融合
- SLAM建图
- Nav2导航栈
- 目标点接收节点（complete_system_launch.py）

### 3. 在另一台设备上启动rviz2

```bash
ros2 run rviz2 rviz2
```

在rviz2中：
1. 添加显示项：
   - Map (topic: /map)
   - LaserScan (topic: /scan)
   - RobotModel
   - TF
   - Path (topic: /plan)
   - Local Costmap (topic: /local_costmap/costmap)
   - Global Costmap (topic: /global_costmap/costmap)

2. 设置Fixed Frame为"map"

3. 使用"2D Pose Estimate"工具设置机器人初始位置（如果需要）

4. 使用"2D Nav Goal"工具发布导航目标点

### 4. 单独启动目标点接收节点

如果需要单独运行目标点接收功能：

```bash
ros2 launch vacuum_nav goal_receiver_launch.py
```

### 5. 仅启动SLAM+导航（不包含硬件驱动）

如果硬件驱动已经在其他地方启动，可以单独启动SLAM和导航：

```bash
ros2 launch vacuum_nav simple_slam_nav_launch.py
```

## 配置说明

### Nav2参数配置 (config/nav2_params.yaml)

主要配置项：
- `robot_radius`: 机器人半径 (0.22m)
- `max_vel_x`: 最大线速度 (0.26 m/s)
- `max_vel_theta`: 最大角速度 (1.0 rad/s)
- `inflation_radius`: 膨胀半径 (0.55m)

可以根据实际机器人参数调整这些值。

## 话题说明

### 订阅的话题
- `/scan`: 激光雷达数据
- `/odometry/filtered`: EKF融合后的里程计
- `/goal_pose`: 来自rviz2的目标点

### 发布的话题
- `/map`: SLAM生成的地图
- `/plan`: 全局路径规划
- `/cmd_vel`: 速度控制命令

## 启动选项

### 完整系统启动 (推荐)
```bash
ros2 launch vacuum_nav complete_system_launch.py
```
包含所有功能，包括目标点接收。

### 基础系统启动
```bash
ros2 launch vacuum_nav vacuum_slam_nav_launch.py
```
不包含目标点接收节点，需要手动启动。

### 仅SLAM+导航功能 (推荐用于调试)
```bash
ros2 launch vacuum_nav simple_slam_nav_launch.py
```
仅启动SLAM和导航，不包含硬件驱动。适合在硬件驱动已经运行的情况下使用。

## 故障排除

### 1. 导航不工作
- 检查所有传感器数据是否正常发布
- 确认TF树是否完整
- 检查Nav2参数配置是否正确

### 2. 建图质量差
- 调整SLAM Toolbox参数
- 检查激光雷达数据质量
- 确认里程计数据准确性

### 3. 路径规划失败
- 检查costmap配置
- 调整机器人半径和膨胀半径
- 确认目标点在可达区域内

### 4. 目标点接收失败
- 检查goal_receiver_node是否正常运行
- 确认rviz2发布的topic名称正确
- 检查网络连接是否稳定

### 5. Launch文件启动失败
如果遇到"missing required argument 'map'"错误：
- 使用`simple_slam_nav_launch.py`而不是`slam_navigation_launch.py`
- 确保使用的是正确的Nav2版本

## 注意事项

1. 确保所有依赖包已正确安装
2. 在树莓派上运行时注意性能限制
3. 建议在良好的WiFi环境下使用
4. 定期保存地图以备后续使用
5. 使用complete_system_launch.py获得最佳体验
6. 如果遇到launch问题，优先使用simple_slam_nav_launch.py

## 扩展功能

可以考虑添加的功能：
- 自动保存地图
- 多点导航
- 动态避障优化
- 电池监控
- 清扫路径规划

## 版本说明

- v1.0: 初始版本，包含基本的SLAM和导航功能
- v1.1: 修复了Nav2 launch文件兼容性问题，添加了简化版本的SLAM导航launch文件
