# WBT ROS2 Workspace - Development Guidelines

## 重要：必须使用ROS2

**所有开发和实验必须使用ROS2 Humble**
- ❌ 不使用ROS1
- ✅ 使用ROS2原生包（slam_gmapping, slam_toolbox, cartographer, Nav2等）

## Workspace Structure

```
/home/jetson/wbt_ws/WBT_ROS2_WS/
├── docs/                    # Documentation files
├── yahboomcar_packages/     # Modified ROS2 packages (editable)
├── yahboomcar_ws/          # Source code (READ-ONLY, symlink)
└── .gitignore
```

## Important Rules

### 0. ROS2 Launch 管理规则（重要！）
**每次启动新的ROS2 launch文件前，必须先停止旧的进程，避免节点重复**

**推荐方式：使用启动脚本（自动清理+验证）**
```bash
cd /home/jetson/wbt_ws/WBT_ROS2_WS
./start_rrt.sh
```

**手动方式：**
```bash
# 1. 停止所有旧进程
pkill -9 -f "rrt_full.launch.py"
pkill -9 -f "display_map_launch.py"
killall -9 bt_navigator controller_server planner_server global_rrt_detector local_rrt_detector sllidar_node rviz2 2>/dev/null
sleep 3

# 2. 验证清理（必须！）
ros2 node list | grep -E "bt_navigator|controller_server|planner_server|global_rrt|local_rrt|sllidar|rviz" | wc -l
# 输出必须为0，否则不要启动

# 3. 启动新的
ros2 launch yahboomcar_nav_rrt rrt_full.launch.py
```

**原因**：重复的节点会导致话题订阅混乱，系统严重混乱（可能出现7个Nav2节点）。

### 1. Source Code Protection
- **`yahboomcar_ws/`** is a symlink to `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws`
- **DO NOT modify files in `_yahboomcar_ws/` directly**
- This is the original source code and must remain unchanged

### 2. Making Changes
- To modify any package from `yahboomcar_ws/`:
  1. Copy the package to `yahboomcar_packages/`
  2. Make your changes in `yahboomcar_packages/`
  3. Commit and push changes from `yahboomcar_packages/`

### 3. Git Operations
- Git repository is located at `/home/jetson/wbt_ws/WBT_ROS2_WS/.git`
- You can run git commands directly in `/home/jetson/wbt_ws/WBT_ROS2_WS/`
- Only files in `yahboomcar_packages/` and `docs/` are tracked by git

## Current Packages

### In yahboomcar_packages/ (tracked by git):
- `yahboomcar_nav` - Navigation package (ROS2)
- `yahboomcar_ctrl` - Control package (ROS2)
- `yahboomcar_nav_rrt` - RRT exploration (converted to ROS2)

### In yahboomcar_ws/ (not tracked, read-only):
- Original source packages from yahboomcar

## Workflow Example

```bash
# If you need to modify a package from yahboomcar_ws:
cp -r yahboomcar_ws/src/package_name yahboomcar_packages/

# Make your changes in yahboomcar_packages/package_name

# Commit and push
git add yahboomcar_packages/package_name
git commit -m "Modify package_name"
git push origin main
```

## 正确启动ROS2导航系统

使用yahboomcar官方的一体化launch文件，不要分开启动各个组件：

```bash
# 终端1：启动SLAM建图
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
ros2 launch yahboomcar_nav map_gmapping_launch.py rplidar_type:=a1

# 终端2：启动RViz可视化
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
ros2 launch yahboomcar_nav display_map_launch.py
```

参考文档：`docs/1.gmapping_guide.md`
