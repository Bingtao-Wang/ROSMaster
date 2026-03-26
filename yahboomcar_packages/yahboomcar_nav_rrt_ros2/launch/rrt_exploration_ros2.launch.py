#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('local_eta', default_value='0.5'),
        DeclareLaunchArgument('global_eta', default_value='2.0'),
        DeclareLaunchArgument('local_range', default_value='5.0'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('info_radius', default_value='1.0'),
        DeclareLaunchArgument('global_costmap_topic', default_value='/global_costmap/costmap'),
        DeclareLaunchArgument('frontiers_topic', default_value='/filtered_points'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        DeclareLaunchArgument('global_frame', default_value='map'),

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='yahboomcar_nav_rrt',
                    executable='global_rrt_detector',
                    name='global_rrt_detector',
                    parameters=[{
                        'eta': LaunchConfiguration('global_eta'),
                        'map_topic': LaunchConfiguration('map_topic'),
                    }],
                    output='screen'
                ),
                Node(
                    package='yahboomcar_nav_rrt',
                    executable='local_rrt_detector',
                    name='local_rrt_detector',
                    parameters=[{
                        'eta': LaunchConfiguration('local_eta'),
                        'range': LaunchConfiguration('local_range'),
                        'map_topic': LaunchConfiguration('map_topic'),
                        'robot_frame': LaunchConfiguration('robot_frame'),
                    }],
                    output='screen'
                ),
                Node(
                    package='yahboomcar_nav_rrt',
                    executable='filter_ros2.py',
                    name='filter',
                    parameters=[{
                        'map_topic': LaunchConfiguration('map_topic'),
                        'info_radius': LaunchConfiguration('info_radius'),
                        'global_costmap_topic': LaunchConfiguration('global_costmap_topic'),
                    }],
                    output='screen'
                ),
                Node(
                    package='yahboomcar_nav_rrt',
                    executable='assigner_ros2.py',
                    name='assigner',
                    parameters=[{
                        'map_topic': LaunchConfiguration('map_topic'),
                        'info_radius': LaunchConfiguration('info_radius'),
                        'frontiers_topic': LaunchConfiguration('frontiers_topic'),
                        'global_frame': LaunchConfiguration('global_frame'),
                        'robot_frame': LaunchConfiguration('robot_frame'),
                    }],
                    output='screen'
                ),
            ]
        ),
    ])
