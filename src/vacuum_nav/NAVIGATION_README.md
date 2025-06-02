# Vacuum Navigation Launch 使用说明

这个文件专门说明如何使用 `navigation_launch.py` 进行纯导航功能。

## 功能说明

`navigation_launch.py` 提供纯导航功能，包括：
- Nav2导航栈
- 目标点接收节点（从rviz2接收目标点）
- 支持预加载地图或SLAM模式

## 使用方法

### 1. 编译包
```bash
colcon build --packages-select vacuum_nav
source install/setup.bash
```

### 2. 启动导航功能

#### 使用预加载地图进行导航：
```bash
ros2 launch vacuum_nav navigation_launch.py map:=/path/to/your/map.yaml
```

#### 在SLAM模式下进行导航（边建图边导航）：
```bash
ros2 launch vacuum_nav navigation_launch.py map:=""
```

#### 其他可选参数：
```bash
ros2 launch vacuum_nav navigation_launch.py \
    map:=/path/to/your/map.yaml \
    use_sim_time:=false \
    params_file:=/path/to/custom/nav2_params.yaml \
    autostart:=true
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

然后使用"2D Nav Goal"工具发布导航目标点。

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map` | `""` | 地图文件路径，留空表示SLAM模式 |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `params_file` | `nav2_params.yaml` | Nav2参数文件路径 |
| `autostart` | `true` | 是否自动启动导航栈 |
| `use_composition` | `True` | 是否使用组合节点 |
| `use_respawn` | `False` | 节点崩溃时是否重启 |

## 前置条件

在启动导航之前，确保以下节点已经运行：
- 激光雷达驱动（发布 `/scan` 话题）
- 里程计节点（发布 `/odom` 或 `/odometry/filtered` 话题）
- TF变换（base_link, odom, map等坐标系）
- 如果使用SLAM模式，需要SLAM节点运行

## 话题说明

### 订阅的话题：
- `/scan`: 激光雷达数据
- `/odom` 或 `/odometry/filtered`: 里程计数据
- `/goal_pose`: 来自rviz2的目标点

### 发布的话题：
- `/cmd_vel`: 速度控制命令
- `/plan`: 全局路径规划
- `/local_plan`: 局部路径规划

## 故障排除

### 1. 导航不工作
- 检查 `/scan` 和 `/odom` 话题是否正常发布
- 确认TF树完整（base_link -> odom -> map）
- 检查Nav2参数配置

### 2. 目标点无法到达
- 检查目标点是否在地图可达区域内
- 调整机器人半径和膨胀半径参数
- 检查costmap配置

### 3. 路径规划失败
- 确认地图数据正确
- 检查机器人当前位置是否准确
- 调整规划器参数

## 示例使用场景

### 场景1：使用已保存的地图进行导航
```bash
# 假设您已经有了保存的地图文件
ros2 launch vacuum_nav navigation_launch.py map:=/home/user/maps/my_map.yaml
```

### 场景2：在SLAM模式下导航（需要SLAM节点已运行）
```bash
# SLAM节点在其他地方启动，这里只启动导航
ros2 launch vacuum_nav navigation_launch.py map:=""
```

### 场景3：自定义参数文件
```bash
ros2 launch vacuum_nav navigation_launch.py \
    map:=/home/user/maps/my_map.yaml \
    params_file:=/home/user/config/custom_nav2_params.yaml
```

## 注意事项

1. 确保所有依赖的传感器和里程计节点已经启动
2. 在树莓派上运行时注意性能限制
3. 建议在良好的WiFi环境下使用rviz2
4. 定期检查TF变换是否正常
5. 根据实际机器人调整Nav2参数
