# 速度转换系统说明

## 概述

本系统为两轮差速扫地机器人实现了速度控制转换功能，将Nav2导航系统输出的连续速度指令转换为适合机器人控制的离散指令格式。

## 系统架构

```
Nav2 Controller -> /cmd_vel -> Velocity Converter -> /cmd_vel_processed -> STM32 Interface -> 机器人硬件
```

### 组件说明

1. **Nav2 Controller**: 导航系统的控制器，发布连续的速度指令到 `/cmd_vel`
2. **Velocity Converter Node**: 速度转换节点，订阅 `/cmd_vel`，转换后发布到 `/cmd_vel_processed`
3. **STM32 Interface Node**: 硬件接口节点，订阅 `/cmd_vel_processed`，发送给STM32

## 速度指令格式

### 输入格式 (从Nav2)
- 连续的线速度和角速度值
- 范围: 线速度 [-0.26, 0.26] m/s, 角速度 [-1.0, 1.0] rad/s

### 输出格式 (发送给STM32)
离散的控制指令：
- `(1.0, 0.0)` - 前进
- `(-1.0, 0.0)` - 后退
- `(0.0, 1.0)` - 左转
- `(0.0, -1.0)` - 右转
- `(0.8, 0.3)` - 前进+左转
- `(0.8, -0.3)` - 前进+右转
- `(-0.8, 0.3)` - 后退+左转
- `(-0.8, -0.3)` - 后退+右转
- `(0.0, 0.0)` - 停止

## 转换逻辑

### 阈值参数
- `linear_threshold`: 0.05 - 线速度阈值
- `angular_threshold`: 0.1 - 角速度阈值

### 转换规则
1. **停止条件**: 当线速度和角速度都小于各自阈值时，输出停止指令
2. **优先级**: 角速度占主导时优先处理转向
3. **组合动作**: 前进/后退时可以同时进行轻微转向

## 使用方法

### 1. 单独启动速度转换节点
```bash
ros2 run vacuum_nav velocity_converter_node
```

### 2. 使用launch文件启动
```bash
# 启动速度转换节点
ros2 launch vacuum_nav velocity_converter_launch.py

# 启动完整导航系统（包含速度转换）
ros2 launch vacuum_nav navigation_launch.py
```

### 3. 参数配置
可以通过参数调整转换行为：
```bash
ros2 run vacuum_nav velocity_converter_node --ros-args \
  -p linear_threshold:=0.08 \
  -p angular_threshold:=0.15 \
  -p max_linear_vel:=0.3 \
  -p max_angular_vel:=1.2
```

## 配置参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `linear_threshold` | 0.05 | 线速度阈值，低于此值视为停止 |
| `angular_threshold` | 0.1 | 角速度阈值，低于此值视为不转向 |
| `max_linear_vel` | 0.26 | 最大线速度限制 |
| `max_angular_vel` | 1.0 | 最大角速度限制 |

## 话题接口

### 订阅话题
- `/cmd_vel` (geometry_msgs/Twist): 来自Nav2的速度指令

### 发布话题
- `/cmd_vel_processed` (geometry_msgs/Twist): 转换后的速度指令

## 调试和监控

### 查看话题数据
```bash
# 查看原始速度指令
ros2 topic echo /cmd_vel

# 查看转换后的速度指令
ros2 topic echo /cmd_vel_processed
```

### 查看节点日志
```bash
ros2 node info /velocity_converter_node
```

## 系统集成

### 修改的文件
1. `src/vacuum_interface/vacuum_interface/stm32_interface_node.py` - 修改订阅话题为 `/cmd_vel_processed`
2. `src/vacuum_nav/vacuum_nav/velocity_converter_node.py` - 新增速度转换节点
3. `src/vacuum_nav/launch/navigation_launch.py` - 集成速度转换节点

### 启动顺序
1. 启动硬件接口: `ros2 launch vacuum_interface stm32_interface_launch.py`
2. 启动导航系统: `ros2 launch vacuum_nav navigation_launch.py`
3. 系统会自动启动速度转换节点

## 故障排除

### 常见问题
1. **机器人不动**: 检查 `/cmd_vel_processed` 话题是否有数据
2. **转换异常**: 检查阈值参数设置是否合理
3. **响应迟缓**: 调整转换逻辑中的阈值参数

### 日志分析
速度转换节点会输出转换信息：
```
Input: (0.150, 0.200) -> Output: (0.800, 0.300)
```

这表示输入线速度0.15m/s，角速度0.2rad/s，转换为前进+左转指令。
