from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 底盘+雷达
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
            '/laser_bringup_launch.py'
        ])
    )

    # SLAM建图
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_nav_rrt'), 'launch/library'),
            '/slam_toolbox.launch.py'
        ])
    )

    # Nav2导航
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_nav_rrt'), 'launch/library'),
            '/nav2.launch.py'
        ])
    )

    # RRT探索
    rrt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_nav_rrt'), 'launch'),
            '/rrt_exploration_ros2.launch.py'
        ])
    )

    return LaunchDescription([
        laser_bringup_launch,
        slam_launch,
        nav2_launch,
        rrt_launch,
    ])
