from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 使用yahboomcar官方的底盘+雷达launch（已处理命名空间）
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
            '/laser_bringup_launch.py'
        ])
    )

    # 启动SLAM和Nav2
    rrt_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_nav_rrt'), 'launch'),
            '/rrt_exploration.launch.py'
        ])
    )

    return LaunchDescription([
        laser_bringup_launch,
        rrt_nav_launch,
    ])
