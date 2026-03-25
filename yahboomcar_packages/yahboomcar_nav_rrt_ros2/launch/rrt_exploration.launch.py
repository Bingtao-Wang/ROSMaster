import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('yahboomcar_nav_rrt')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    lidar_type = os.environ.get('RPLIDAR_TYPE', 'a1')

    scan_dilute_node = Node(
        package='yahboomcar_nav_rrt',
        executable='scan_dilute.py',
        output='screen',
        respawn=True,
        condition=IfCondition(str(lidar_type == 's2').lower())
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'library', 'slam_toolbox.launch.py'])
        ]),
        launch_arguments={'scan_topic': 'scan_dilute' if lidar_type == 's2' else 'scan'}.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'library', 'nav2.launch.py'])
        ])
    )

    return LaunchDescription([
        use_rviz_arg,
        scan_dilute_node,
        slam_toolbox_launch,
        nav2_launch,
    ])
