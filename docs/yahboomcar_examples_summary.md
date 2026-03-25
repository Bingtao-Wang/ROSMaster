# Yahboomcar ROS2 例程总结

> 源码路径: `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/`
> 支持车型: X1 (紧凑麦克纳姆轮), X3 (麦克纳姆轮), R2 (阿克曼转向)

---

## 一、硬件驱动层

### 1. yahboomcar_bringup — 底盘驱动与硬件接口
- **功能**: 电机驱动、IMU 数据发布、RGB LED 控制、蜂鸣器、速度标定工具、巡逻模式
- **技术**: ROS2 rclpy, Rosmaster_Lib (亚博硬件库), threading
- **关键文件**:
  - `Mcnamu_driver_X3.py` / `Mcnamu_driver_x1.py` — 麦克纳姆轮驱动
  - `Ackman_driver_R2.py` — 阿克曼转向驱动
  - `calibrate_linear_*.py` / `calibrate_angular_*.py` — 线速度/角速度标定

### 2. yahboomcar_base_node — 里程计与坐标变换
- **功能**: 根据速度指令计算里程计, 广播 odom→base_footprint 的 TF 变换
- **技术**: ROS2 rclcpp (C++), tf2, nav_msgs/Odometry
- **关键文件**: `base_node_x1.cpp`, `base_node_X3.cpp`, `base_node_R2.cpp`

---

## 二、传感器与感知层

### 3. yahboomcar_astra — 颜色追踪 (RGB-D 相机)
- **功能**: 基于 HSV 的颜色目标检测与追踪, 发布目标位置驱动机器人跟随, 附 HSV 校准工具
- **技术**: OpenCV, cv_bridge, numpy, PID 控制
- **关键文件**: `colorTracker.py`, `colorHSV.py`, `astra_common.py`

### 4. yahboomcar_KCFTracker — KCF 目标追踪
- **功能**: 使用 KCF (核相关滤波) 算法实时追踪用户选定的 ROI, 结合深度图计算距离, PID 控制跟随
- **技术**: OpenCV 4.10+, C++ rclcpp, FHOG 特征提取, cv_bridge, image_transport
- **关键文件**: `KCF_Tracker.cpp`, `kcftracker.cpp`, `fhog.cpp`, `PID.cpp`

### 5. yahboomcar_mediapipe — AI 人体/手势/人脸检测
- **功能**: 集成 Google MediaPipe, 实现手部检测、姿态估计、面部关键点识别, 发布关键点数组
- **技术**: MediaPipe, OpenCV, numpy, 自定义 PointArray 消息
- **关键文件**: `01_HandDetector.py`, `02_PoseDetector.py`

### 6. yahboomcar_laser — 激光雷达避障与跟随
- **功能**: 分析 2D 激光扫描数据, 分区检测障碍物 (前/左/右), 提供三种模式: 跟踪、避障、预警
- **技术**: ROS2 rclpy, sensor_msgs/LaserScan, numpy, PID 控制
- **关键文件**: `laser_Tracker_a1_X3.py`, `laser_Avoidance_a1_X3.py`, `laser_Warning_a1_X3.py`

### 7. laserscan_to_point_pulisher — 激光扫描转点路径
- **功能**: 将 LaserScan 消息转换为 Path 消息 (含各点位姿), 用于可视化或下游处理
- **技术**: ROS2 rclpy, tf2_ros, 三角函数坐标变换
- **关键文件**: `laserscan_to_point_publish.py`

### 8. robot_pose_publisher_ros2 — 机器人位姿发布
- **功能**: 从 TF 树中提取 map→base_link 变换, 以 50ms 周期发布机器人位姿
- **技术**: ROS2 rclcpp (C++), tf2_ros
- **关键文件**: `robot_pose_publisher.cpp`

---

## 三、视觉处理层

### 9. yahboomcar_visual — 视觉工具集
- **功能**: 图像发布/缩放、激光数据转鸟瞰图、Astra 深度/彩色图处理、简易 AR 叠加、二维码处理
- **技术**: OpenCV, cv_bridge, Astra SDK
- **关键文件**: `pub_image.py`, `laser_to_image.py`, `simple_AR.py`, `astra_rgb_image.py`, `astra_depth_image.py`

### 10. yahboomcar_point — 关键点转点云
- **功能**: 将 MediaPipe 检测到的 2D 关键点转换为 PointCloud2 点云数据用于 3D 可视化
- **技术**: C++ rclcpp, PCL (点云库), VTK, 自定义 PointArray 消息
- **关键文件**: `pub_point.cpp`, `mediapipe.cpp`

---

## 四、SLAM 与导航层

### 11. yahboomcar_slam — 3D 点云建图
- **功能**: 基于 RGB-D 相机的点云 SLAM, 使用 ICP 配准、点云滤波, 构建 OctoMap 三维地图
- **技术**: C++ rclcpp, PCL, Eigen, OpenCV, OctoMap, message_filters
- **关键文件**: `point_cloud.cpp`, `point_cloud.h`

### 12. yahboomcar_nav — 导航与建图启动配置
- **功能**: 提供多种 SLAM 算法的 launch 配置: GMapping、RTABMap、Cartographer; 支持多种雷达型号
- **技术**: ROS2 Navigation Stack, gmapping, rtabmap, cartographer, TEB 局部规划器
- **关键文件**: `map_gmapping_launch.py`, `rtabmap_sync_launch.py`, `cartographer_launch.py`

---

## 五、控制层

### 13. yahboomcar_ctrl — 遥控操作
- **功能**: 手柄遥控和键盘遥控, 映射输入为 Twist 速度指令, 支持档位切换、LED 控制、导航取消
- **技术**: ROS2 rclpy, sensor_msgs/Joy, geometry_msgs/Twist
- **关键文件**: `yahboom_joy_X3.py`, `yahboom_joy_R2.py`, `yahboom_keyboard.py`

### 14. yahboomcar_linefollow — 巡线
- **功能**: 基于摄像头颜色检测实现自动巡线, HSV 提取线条颜色, PID 控制转向
- **技术**: OpenCV, ROS2 rclpy, PID 控制
- **关键文件**: `follow_line_4ROS_R2.py`, `follow_common.py`

### 15. yahboomcar_voice_ctrl — 语音控制
- **功能**: 在巡线和颜色追踪基础上增加语音指令控制, 支持语音启停和模式切换
- **技术**: Speech_Lib (语音识别库), OpenCV, ROS2 rclpy
- **关键文件**: `Voice_Ctrl_follow_line_a1_R2.py`, `Voice_Ctrl_colorTracker.py`

### 16. yahboomcar_multi — 多机协同
- **功能**: 多机器人编队控制 (队列/巡逻), TF 坐标监听实现多机定位
- **技术**: ROS2 rclpy, tf2, PyKDL (运动学库)
- **关键文件**: `queue.py`, `listenline.py`

---

## 六、模型与可视化

### 17. yahboomcar_description — X3/R2 机器人模型
- **功能**: X3 和 R2 的 URDF 机器人描述文件, 定义连杆、关节、碰撞体、惯性参数
- **技术**: URDF, Xacro, STL 网格文件
- **关键文件**: `yahboomcar_X3.urdf`, `yahboomcar_R2.urdf`

### 18. yahboomcar_description_x1 — X1 机器人模型
- **功能**: X1 机器人 URDF 描述文件
- **技术**: URDF, Xacro, STL 网格文件
- **关键文件**: `yahboomcar_X1.urdf`

### 19. yahboomcar_rviz — RViz 可视化配置
- **功能**: 提供建图、导航、激光扫描等场景的 RViz 配置文件和启动脚本
- **技术**: RViz2, joint_state_publisher
- **关键文件**: `mapping.rviz`, `nav.rviz`, `navigation.rviz`, `laser_scan.rviz`

---

## 七、消息与服务接口

### 20. yahboomcar_msgs — 自定义消息
- **功能**: 定义系统专用消息类型
- **消息类型**:
  - `Target.msg` — 目标检测数据
  - `PointArray.msg` — 点数组
  - `ImageMsg.msg` — 自定义图像数据
  - `Position.msg` — 角度和距离

### 21. yahboom_web_savmap_interfaces — Web 地图保存服务接口
- **功能**: 定义 Web 端保存地图的 ROS2 服务接口
- **服务**: `WebSaveMap.srv` (请求: mapname, 返回结果)

### 22. yahboom_app_save_map — App 地图保存
- **功能**: 调用 nav2_map_server 将 SLAM 地图保存到文件系统, 元数据存入 SQLite 数据库
- **技术**: ROS2 rclpy, sqlite3, subprocess, nav2_map_server
- **关键文件**: `yahboom_app_save_map.py`, `yahboom_app_save_map_client.py`

---

## 系统架构总览

```
┌─────────────────────────────────────────────────────┐
│                   应用层                              │
│  巡线 │ 语音控制 │ 多机协同 │ 遥控 │ 地图保存        │
├─────────────────────────────────────────────────────┤
│                 导航与建图                            │
│  GMapping │ RTABMap │ Cartographer │ Nav2            │
├─────────────────────────────────────────────────────┤
│                   感知层                              │
│  颜色追踪 │ KCF追踪 │ MediaPipe │ 激光避障/跟随     │
├─────────────────────────────────────────────────────┤
│               可视化 & 工具                           │
│  RViz │ 视觉工具 │ 点云转换 │ 位姿发布              │
├─────────────────────────────────────────────────────┤
│                 硬件驱动层                            │
│  底盘驱动 (bringup) │ 里程计 (base_node)            │
├─────────────────────────────────────────────────────┤
│              消息 & 接口定义                          │
│  yahboomcar_msgs │ web_savmap_interfaces             │
└─────────────────────────────────────────────────────┘
```

## 核心技术栈

| 类别 | 技术 |
|------|------|
| 框架 | ROS2 (rclpy / rclcpp) |
| 视觉 | OpenCV, MediaPipe, cv_bridge |
| 点云 | PCL, OctoMap, Eigen |
| 建图 | GMapping, RTABMap, Cartographer |
| 导航 | Nav2, TEB Local Planner |
| 硬件 | Rosmaster_Lib, Astra SDK |
| 模型 | URDF, Xacro |
| 控制 | PID, 手柄 (Joy), 键盘, 语音 (Speech_Lib) |
| 语言 | Python (主), C++ (性能关键模块) |
