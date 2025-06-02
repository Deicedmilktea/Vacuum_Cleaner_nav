# Vacuum Complete System Launch 使用说明

这个文件说明如何使用 `vacuum_slam_nav_launch.py` 启动完整的吸尘器机器人系统。

## 功能说明

`vacuum_slam_nav_launch.py` 是一个完整的系统启动文件，包含：

### 硬件驱动：
- 激光雷达驱动 (ldlidar_sl_ros2)
- STM32接口 (vacuum_interface)
- 机器人描述和TF (vacuum_describe)
- EKF传感器融合 (vacuum_fusion)

### SLAM和导航：
- SLAM Toolbox (实时建图)
- Nav2导航栈 (路径规划和导航)
- 目标点接收节点 (从rviz2接收目标点)

## 使用方法

### 1. 编译包
```bash
colcon build --packages-select vacuum_nav
source install/setup.bash
```

### 2. 启动完整系统

#### 基本启动（SLAM模式，推荐）：
```bash
ros2 launch vacuum_nav vacuum_slam_nav_launch.py
```

#### 使用预加载地图进行导航：
```bash
ros2 launch vacuum_nav vacuum_slam_nav_launch.py map:=/path/to/your/map.yaml
```

#### 自定义参数启动：
```bash
ros2 launch vacuum_nav vacuum_slam_nav_launch.py \
    nav_params_file:=/path/to/custom/nav2_params.yaml \
    slam_params_file:=/path/to/custom/slam_params.yaml \
    use_sim_time:=false
```

### 3. 在另一台设备上启动rviz2
```bash
ros2 run rviz2 rviz2
```

在rviz2中添加以下显示项：
- Map (topic: /map)
- LaserScan (topic: /scan)
- RobotModel
- TF
- Path (topic: /plan)
- Local Costmap (topic: /local_costmap/costmap)
- Global Costmap (topic: /global_costmap/costmap)
- Pose Array (topic: /particlecloud) - 如果使用AMCL

设置Fixed Frame为"map"，然后使用"2D Nav Goal"工具发布导航目标点。

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `nav_params_file` | `nav2_params.yaml` | Nav2参数文件路径 |
| `slam_params_file` | `slam_toolbox_params.yaml` | SLAM参数文件路径 |
| `map` | `""` | 地图文件路径，留空表示SLAM模式 |
| `autostart` | `true` | 是否自动启动导航栈 |
| `use_composition` | `True` | 是否使用组合节点 |
| `use_respawn` | `False` | 节点崩溃时是否重启 |

## 系统架构

```
vacuum_slam_nav_launch.py
├── 硬件驱动层
│   ├── ldlidar_sl_ros2 (激光雷达)
│   ├── vacuum_interface (STM32通信)
│   ├── vacuum_describe (机器人描述)
│   └── vacuum_fusion (EKF融合)
├── 感知和建图层
│   └── slam_toolbox (SLAM建图)
├── 导航层
│   └── nav2_bringup (导航栈)
└── 交互层
    └── goal_receiver_node (目标点接收)
```

## 话题说明

### 主要输入话题：
- `/scan`: 激光雷达数据
- `/wheel_odom`: 轮式里程计
- `/imu`: IMU数据
- `/goal_pose`: 来自rviz2的目标点

### 主要输出话题：
- `/map`: SLAM生成的地图
- `/cmd_vel`: 速度控制命令
- `/plan`: 全局路径规划
- `/local_plan`: 局部路径规划
- `/odometry/filtered`: EKF融合后的里程计

## 启动顺序

系统按以下顺序启动各个组件：
1. 激光雷达驱动
2. STM32接口
3. 机器人描述和TF
4. EKF传感器融合
5. SLAM Toolbox
6. Nav2导航栈
7. 目标点接收节点

## 故障排除

### 1. 系统启动失败
- 检查所有依赖包是否已安装
- 确认硬件连接正常（激光雷达、STM32）
- 检查串口权限和设备路径

### 2. SLAM建图质量差
- 检查激光雷达数据质量
- 调整SLAM参数文件
- 确认机器人运动平稳

### 3. 导航不工作
- 检查TF树是否完整
- 确认地图数据正常
- 检查Nav2参数配置

### 4. 目标点无法到达
- 检查目标点是否在可达区域
- 调整机器人半径和膨胀半径
- 检查costmap配置

### 5. 网络连接问题
- 确认树莓派和rviz2设备在同一网络
- 检查ROS_DOMAIN_ID设置
- 测试话题通信

## 使用场景

### 场景1：首次使用（建图模式）
```bash
# 启动完整系统，开始建图
ros2 launch vacuum_nav vacuum_slam_nav_launch.py

# 在rviz2中观察建图过程，使用2D Nav Goal发送目标点
# 机器人会边建图边导航到目标点
```

### 场景2：使用已有地图
```bash
# 使用之前保存的地图进行导航
ros2 launch vacuum_nav vacuum_slam_nav_launch.py map:=/home/user/maps/my_map.yaml
```

### 场景3：调试模式
```bash
# 使用自定义参数进行调试
ros2 launch vacuum_nav vacuum_slam_nav_launch.py \
    nav_params_file:=/home/user/debug_nav2_params.yaml \
    autostart:=false
```

## 性能优化建议

### 树莓派优化：
1. 使用高速SD卡（Class 10或更高）
2. 确保充足的电源供应
3. 适当降低SLAM和导航频率
4. 关闭不必要的系统服务

### 网络优化：
1. 使用5GHz WiFi频段
2. 减少网络延迟
3. 优化ROS2 DDS配置

## 扩展功能

可以通过修改launch文件添加：
- 自动保存地图功能
- 多点导航任务
- 电池监控
- 清扫路径规划
- 远程监控界面

## 注意事项

1. 确保所有硬件正常连接
2. 在树莓派上运行时注意性能限制
3. 建议在良好的WiFi环境下使用
4. 定期保存重要的地图数据
5. 注意机器人的安全运行范围
6. 定期检查和维护硬件设备

## 版本信息

- 支持ROS2 Humble
- 兼容Nav2和SLAM Toolbox最新版本
- 适配树莓派4B及以上型号
