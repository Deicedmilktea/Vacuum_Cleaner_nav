# ROS 2 SLAM 系统框架 (树莓派 + LD14 + IMU + 轮式里程计 + slam_toolbox)

这是一个基于文本的框架图，描述了系统中的主要节点、它们交互的关键话题以及TF坐标变换树。**此版本已更新，包含轮式里程计数据，并通过 `robot_localization` 进行融合。**

---

## 1. 传感器与底层接口 (Sensors & Low-Level Interface)

* **节点:** Lidar 驱动 (例如: `/ldlidar_node`)
    * **发布:** `/scan` (类型: `sensor_msgs/msg/LaserScan`) - 提供激光扫描数据。

* **节点:** **STM32 串口接口节点 (例如: `/stm32_interface_node`)** <-- **关键自定义节点**
    * **功能:**
        * 通过串口 (例如`/dev/ttyUSBx`) 读取来自STM32的数据。
        * **解析**自定义的串行通信协议，区分IMU和轮式里程计数据。
    * **发布:** `/imu/data` (类型: `sensor_msgs/msg/Imu`)
        * **填充:** 根据从STM32接收的数据填充姿态、角速度、线加速度。
        * **注意:** **必须在此节点中为IMU消息设置合理、反映真实传感器特性的协方差矩阵。**
    * **发布:** `/wheel_odom` (类型: `nav_msgs/msg/Odometry`)
        * **填充:** 根据从STM32接收的数据填充位姿（X, Y, Yaw）和速度。
        * **注意:** **必须在此节点中为轮式里程计消息设置合理、反映运动模型和传感器精度的协方差矩阵（例如，Roll/Pitch通常不确定性极大或为0）。**
    * **依赖:** 需要串口通信库 (如 `pyserial` for Python, Boost.Asio/serial for C++) 和对串行协议的了解。

---

## 2. TF坐标系设置 (TF Setup)

* **节点:** `/joint_state_publisher`
    * **发布:** `/joint_states` (类型: `sensor_msgs/msg/JointState`)

* **节点:** `/robot_state_publisher`
    * **订阅:** `/joint_states`
    * **使用:** 参数 `/robot_description` (从URDF文件加载)
    * **发布:** `/tf` 和 `/tf_static` (类型: `tf2_msgs/msg/TFMessage`)
        * **关键变换:** `base_link` --> `lidar_link`
        * **关键变换:** `base_link` --> `imu_link`
        * (以及URDF中定义的其他静态变换)

---

## 3. 传感器融合 (Sensor Fusion - robot_localization)

* **节点:** EKF 融合节点 (例如: `/ekf_filter_node` 来自 `robot_localization`)
    * **订阅:** `/imu/data` **(来自IMU驱动)**
    * **订阅:** `/wheel_odom` **(来自轮式里程计驱动)**
    * **订阅:** `/tf`, `/tf_static` (隐式使用，需要知道IMU相对于`base_link`的位置)
    * **发布:** `/odometry/filtered` (类型: `nav_msgs/msg/Odometry`) - 发布**融合轮式里程计和IMU后**的里程计信息。
    * **发布:** `/tf`
        * **关键变换:** `odom` --> `base_link` (机器人相对于里程计坐标系的位姿估计，**现在是轮式里程计和IMU融合的结果，精度更高，漂移更小**)。

    * **配置注意 (`ekf.yaml`):**
        * 需要配置 `odom0` 指向 `/wheel_odom` 话题，并设置 `odom0_config` 来选择使用轮式里程计的哪些数据（通常是 X, Y 位置, Yaw 方向, X, Y 线速度, Yaw 角速度）。**必须将 Roll 和 Pitch 相关项设为 `false`。**
        * 需要配置 `imu0` 指向 `/imu/data` 话题，并设置 `imu0_config` 来选择使用IMU的哪些数据（通常是 Roll, Pitch, Yaw 方向, Roll, Pitch, Yaw 角速度）。**可以将线速度相关项设为 `false`，因为轮式里程计通常更准。**
        * `odom0_differential` 或 `odom0_relative` 参数可能需要根据轮式里程计特性设置。
        * **仔细调整各传感器输入的协方差矩阵对融合效果至关重要。**

---

## 4. SLAM 建图与定位 (SLAM - slam_toolbox)

* **节点:** `/slam_toolbox` (例如: `async_slam_toolbox_node`)
    * **订阅:** `/scan` (激光数据)
    * **订阅:** `/tf`, `/tf_static` (获取由 `robot_localization` 发布的、**更精确的** `odom` --> `base_link` 变换，以及 `base_link` --> `lidar_link` 等)
    * **(可选订阅):** `/imu/data` (现在有了融合后的 `odom`，通常**不再需要**让 `slam_toolbox` 直接订阅原始IMU数据，可以将`use_imu_data`设为`false`)
    * **发布:** `/map` (类型: `nav_msgs/msg/OccupancyGrid`)
    * **发布:** `/map_metadata` (类型: `nav_msgs/msg/MapMetaData`)
    * **发布:** `/particle_cloud` (类型: `geometry_msgs/msg/PoseArray`)
    * **发布:** `/tf`
        * **关键变换:** `map` --> `odom` (SLAM对**融合后**的里程计漂移的修正)
    * **提供服务:** `/slam_toolbox/save_map` 等。

    * **效果提升:** 由于输入的 `odom` -> `base_link` 精度提高，SLAM建图和定位的精度、鲁棒性都会有显著改善。

---

## 5. 可视化 (Visualization)

* **节点:** `/rviz2`
    * **订阅:** (根据用户添加的显示项决定)
        * `/map`, `/scan`, `/imu/data`
        * `/odometry/filtered` (**显示融合后的轨迹**)
        * **(可选):** `/wheel_odom` (显示原始轮式里程计轨迹，用于对比)
        * `/particle_cloud`, `/tf`, `/tf_static`, `/robot_description`

```
此处由于树莓派的算力不足以可视化，采用另一台设备在同一局域网下订阅话题后进入Rviz进行可视化。
```

---

## 6. TF坐标树结构 (TF Tree Structure)

TF树的**结构保持不变**，但 `odom` -> `base_link` 变换的发布者和质量发生了变化：

```text
map (世界固定坐标系, 由 SLAM 发布)
 |
 +--> odom (里程计坐标系, 相对于 map 的变换由 SLAM 发布)
      |
      +--> base_link (机器人基座坐标系, 相对于 odom 的变换现在由 robot_localization (融合轮速+IMU) 发布)
           |
           +--> lidar_link (激光雷达坐标系, 相对于 base_link 的变换由 robot_state_publisher 发布)
           |
           +--> imu_link (IMU坐标系, 相对于 base_link 的变换由 robot_state_publisher 发布)
           |
           +--> [其他机器人部件坐标系, 如轮子等, 由 robot_state_publisher 发布]