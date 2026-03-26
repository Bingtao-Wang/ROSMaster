#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('trace_file', default_value=''),
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('navigate_action', default_value='navigate_to_pose'),
        Node(
            package='yahboomcar_nav_rrt',
            executable='trace_replay.py',
            name='trace_replay',
            parameters=[{
                'trace_file': LaunchConfiguration('trace_file'),
                'global_frame': LaunchConfiguration('global_frame'),
                'navigate_action': LaunchConfiguration('navigate_action'),
            }],
            output='screen',
        ),
    ])
