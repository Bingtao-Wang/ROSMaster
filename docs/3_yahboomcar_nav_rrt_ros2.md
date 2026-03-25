# yahboomcar_nav_rrt_ros2

ROS2 自主探索导航包，基于 RRT（Rapidly-exploring Random Tree）算法实现未知环境的自动探索与建图。

## 功能特性

- **SLAM 建图**：集成 SLAM Toolbox 实现实时建图
- **自主导航**：基于 Nav2 的路径规划与避障
- **多点导航**：支持设置多个目标点顺序导航
- **地图保存**：提供地图保存服务
- **激光雷达支持**：兼容多种激光雷达型号（A1/S2）

## 包结构

```
yahboomcar_nav_rrt_ros2/
├── src/                    # C++ 源码
│   ├── save_map.cpp       # 地图保存服务节点
│   └── rrt_save_map.cpp   # RRT 地图保存节点
├── scripts/               # Python 脚本
│   ├── scan_dilute.py     # 激光扫描稀释（S2 雷达）
│   ├── send_mark.py       # 多点导航节点
│   └── spech_send_mark.py # 语音控制多点导航
├── launch/                # 启动文件
│   ├── rrt_exploration.launch.py  # 主启动文件
│   └── library/           # 子启动文件
│       ├── slam_toolbox.launch.py
│       └── nav2.launch.py
├── param/                 # 参数配置
├── rviz/                  # RViz 配置
└── maps/                  # 地图文件
```

## 依赖项

- ROS2 (Humble/Foxy)
- slam_toolbox
- nav2_bringup
- nav2_msgs
- geometry_msgs
- sensor_msgs
- visualization_msgs

## 编译

```bash
cd /home/jetson/wbt_ws/WBT_ROS2_WS
colcon build --packages-select yahboomcar_nav_rrt
source install/setup.bash
```

## 使用方法

### 1. 启动 RRT 探索

```bash
# 设置激光雷达类型（可选：a1 或 s2）
export RPLIDAR_TYPE=a1

# 启动探索节点
ros2 launch yahboomcar_nav_rrt rrt_exploration.launch.py
```

### 2. 多点导航

启动多点导航节点后，在 RViz 中：
1. 使用 "Publish Point" 工具点击地图设置目标点
2. 机器人将按顺序访问所有目标点
3. 发布 `/initialpose` 可清除所有目标点

```bash
ros2 run yahboomcar_nav_rrt send_mark.py
```

### 3. 保存地图

```bash
# 方法 1：调用服务
ros2 service call /save_map std_srvs/srv/Trigger

# 方法 2：直接保存
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

## 核心节点

### save_map
- **类型**：C++ 节点
- **功能**：提供 `/save_map` 服务保存当前地图
- **服务类型**：`std_srvs/srv/Trigger`

### multipoint_navigation (send_mark.py)
- **类型**：Python 节点
- **功能**：多点导航控制
- **订阅话题**：
  - `/clicked_point` (PointStamped)：接收目标点
  - `/initialpose` (PoseWithCovarianceStamped)：清除目标点
  - `/JoyState` (Bool)：手柄控制状态
- **发布话题**：
  - `/path_point` (MarkerArray)：可视化路径点
  - `/goal_pose` (PoseStamped)：导航目标
- **Action 客户端**：`navigate_to_pose` (Nav2)

### scan_dilute.py
- **类型**：Python 节点
- **功能**：稀释 S2 激光雷达扫描数据以提高性能
- **条件**：仅在 `RPLIDAR_TYPE=s2` 时启动

## 参数配置

Nav2 参数文件位于：`param/common/nav2_params.yaml`

关键参数：
- `base_frame`: `base_footprint`
- `odom_frame`: `odom`
- `map_frame`: `map`
- `scan_topic`: `scan` 或 `scan_dilute`

## 环境变量

- `RPLIDAR_TYPE`：激光雷达型号（`a1` 或 `s2`，默认 `a1`）
- `ROBOT_TYPE`：机器人型号（默认 `X3`）

## 故障排除

### 激光雷达数据过密
如使用 S2 雷达，设置 `RPLIDAR_TYPE=s2` 启用扫描稀释节点。

### 导航失败
检查：
1. TF 树是否完整（`ros2 run tf2_tools view_frames`）
2. 激光雷达数据是否正常（`ros2 topic echo /scan`）
3. Nav2 参数配置是否匹配机器人

### 地图保存失败
确保有写入权限，或手动指定路径：
```bash
ros2 run nav2_map_server map_saver_cli -f /path/to/map
```

## 相关文档

- [Nav2 官方文档](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [RRT 算法原理](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)
