#!/bin/bash
# RRT探索导航启动脚本 - 确保无重复节点

echo "=== 停止所有旧进程 ==="
pkill -9 -f "rrt_full.launch.py"
pkill -9 -f "display_map_launch.py"
killall -9 bt_navigator controller_server planner_server 2>/dev/null
killall -9 global_rrt_detector local_rrt_detector 2>/dev/null
killall -9 sllidar_node rviz2 2>/dev/null

echo "等待进程清理..."
sleep 3

echo "=== 验证清理结果 ==="
NODE_COUNT=$(ros2 node list 2>/dev/null | grep -E "bt_navigator|controller_server|planner_server|global_rrt|local_rrt|sllidar|rviz" | wc -l)
if [ $NODE_COUNT -gt 0 ]; then
    echo "警告：仍有 $NODE_COUNT 个相关节点在运行"
    echo "建议重启系统：sudo reboot"
    exit 1
fi

echo "=== 启动RRT系统 ==="
cd /home/jetson/wbt_ws/WBT_ROS2_WS
source install/setup.bash
ros2 launch yahboomcar_nav_rrt rrt_full.launch.py &
RRT_PID=$!

echo "等待RRT系统启动..."
sleep 10

echo "=== 启动RViz ==="
export DISPLAY=:0
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
ros2 launch yahboomcar_nav display_map_launch.py &
RVIZ_PID=$!

sleep 5

echo "=== 启动激光雷达电机 ==="
ros2 service call /start_motor std_srvs/srv/Empty

echo ""
echo "=== 验证节点状态 ==="
ros2 node list | grep -E "bt_navigator|controller_server|planner_server|global_rrt|local_rrt|sllidar|rviz" | sort | uniq -c

echo ""
echo "=== 启动完成 ==="
echo "RRT进程ID: $RRT_PID"
echo "RViz进程ID: $RVIZ_PID"
echo ""
echo "使用方法："
echo "1. 手动控制机器人建图（手柄或键盘）"
echo "2. 在RViz中点击5个点定义探索区域（前4个点框区域，第5个点为种子点）"
echo "3. 观察RRT自主探索"
