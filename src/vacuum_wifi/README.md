# Vacuum WiFi Package

这个包提供了通过WiFi进行TCP通信的功能，用于远程控制吸尘器机器人。

## 功能特性

1. **速度命令接收**: 接收格式为 `v=linear,angular` 的速度命令
2. **Twist消息发布**: 将接收到的速度命令转换为ROS2 Twist消息并发布到 `/cmd_vel_processed` 话题
3. **地图数据传输**: 定期订阅 `/map` 话题并将地图数据发送到连接的客户端

## 消息格式

### 速度命令格式
```
v=linear_velocity,angular_velocity
```

示例:
- `v=1.0,0.0` - 前进，线速度1.0 m/s，角速度0.0 rad/s
- `v=0.0,1.0` - 左转，线速度0.0 m/s，角速度1.0 rad/s
- `v=-1.0,0.0` - 后退，线速度-1.0 m/s，角速度0.0 rad/s
- `v=0.0,-1.0` - 右转，线速度0.0 m/s，角速度-1.0 rad/s
- `v=0.0,0.0` - 停止

### 地图数据格式
地图数据以JSON格式发送，前缀为 `MAP:`：
```json
{
  "type": "map",
  "width": 地图宽度,
  "height": 地图高度,
  "resolution": 地图分辨率,
  "origin": {
    "x": 原点x坐标,
    "y": 原点y坐标,
    "z": 原点z坐标
  },
  "data": [地图数据数组]
}
```

## 参数配置

- `tcp_server_ip`: TCP服务器IP地址 (默认: '0.0.0.0')
- `tcp_server_port`: TCP服务器端口 (默认: 8080)
- `map_send_interval`: 地图发送间隔，单位秒 (默认: 5.0)

## 使用方法

### 1. 构建包
```bash
cd /path/to/your/workspace
colcon build --packages-select vacuum_wifi
source install/setup.bash
```

### 2. 启动节点
```bash
ros2 launch vacuum_wifi wifi_launch.py
```

### 3. 自定义参数启动
```bash
ros2 launch vacuum_wifi wifi_launch.py tcp_server_port:=9090 map_send_interval:=3.0
```

### 4. 测试连接
使用提供的测试脚本：
```bash
python3 test_wifi_client.py
```

## 话题

### 发布的话题
- `/cmd_vel_processed` (geometry_msgs/Twist): 处理后的速度命令

### 订阅的话题
- `/map` (nav_msgs/OccupancyGrid): 地图数据

## 网络通信

节点作为TCP服务器运行，监听指定端口上的连接。客户端可以连接到服务器并发送速度命令，同时接收地图数据。

### 连接流程
1. 客户端连接到服务器
2. 客户端发送速度命令
3. 服务器解析命令并发布Twist消息
4. 服务器发送确认消息 "OK"
5. 服务器定期发送地图数据（如果有客户端连接且地图可用）

## 依赖项

- rclpy
- geometry_msgs
- nav_msgs
- socket (Python标准库)
- threading (Python标准库)
- re (Python标准库)
- json (Python标准库)

## 注意事项

1. 确保防火墙允许指定端口的连接
2. 一次只能有一个客户端连接
3. 地图数据较大时可能需要调整网络缓冲区大小
4. 建议在局域网环境下使用以获得最佳性能
