#!/bin/bash

# RRT Exploration 完整实验启动脚本

echo "=== RRT Exploration 实验启动 ==="
echo ""

# 设置工作目录
cd /home/jetson/wbt_ws/WBT_ROS2_WS
source install/setup.bash

# 启动底盘+雷达+SLAM
echo "1. 启动底盘、雷达和SLAM..."
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
ros2 launch yahboomcar_nav map_gmapping_launch.py rplidar_type:=a1 &
SLAM_PID=$!
echo "   SLAM PID: $SLAM_PID"

sleep 5

# 启动RRT探索
echo "2. 启动RRT探索节点..."
cd /home/jetson/wbt_ws/WBT_ROS2_WS
source install/setup.bash
ros2 launch yahboomcar_nav_rrt rrt_exploration_ros2.launch.py &
RRT_PID=$!
echo "   RRT PID: $RRT_PID"

sleep 3

# 启动RViz
echo "3. 启动RViz可视化..."
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
ros2 launch yahboomcar_nav display_map_launch.py &
RVIZ_PID=$!
echo "   RViz PID: $RVIZ_PID"

echo ""
echo "=== 所有节点已启动 ==="
echo "SLAM PID: $SLAM_PID"
echo "RRT PID: $RRT_PID"
echo "RViz PID: $RVIZ_PID"
echo ""
echo "按 Ctrl+C 停止所有节点"

wait
