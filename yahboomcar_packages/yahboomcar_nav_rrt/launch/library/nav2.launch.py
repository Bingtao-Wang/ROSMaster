import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('yahboomcar_nav_rrt')
    robot_type = os.environ.get('ROBOT_TYPE', 'X3')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'param', 'common', 'nav2_params.yaml'])],
        remappings=[('cmd_vel', 'cmd_vel')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'param', 'common', 'nav2_params.yaml'])]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'param', 'common', 'nav2_params.yaml'])]
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        bt_navigator,
    ])
