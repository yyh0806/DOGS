"""Go2W 全系统启动文件。

启动顺序:
  1. go2w_bridge       - 连接 Go2W SDK，发布 /scan, /odom, /tf, /camera
  2. slam_toolbox      - 实时 SLAM 建图
  3. nav2              - 导航栈 (路径规划 + 避障)
  4. go2w_detector     - YOLO 目标检测
  5. go2w_orchestrator - 任务编排 + VLM
  6. go2w_web          - Web 控制面板

用法:
  ros2 launch go2w_bringup search.launch.py
  ros2 launch go2w_bringup search.launch.py network_interface:=enx001e06300000
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'default.yaml'
    )
    nav_dir = os.path.join(
        os.path.dirname(__file__), '..', '..', 'go2w_nav'
    )

    # 参数
    network_interface_arg = DeclareLaunchArgument(
        'network_interface', default_value='',
        description='Go2W 连接网卡名'
    )

    # 节点 1: Bridge（立即启动）
    bridge_node = Node(
        package='go2w_bridge',
        executable='bridge_node',
        name='go2w_bridge',
        output='screen',
        parameters=[config_file, {
            'network_interface': LaunchConfiguration('network_interface'),
        }],
    )

    # 节点 2: SLAM Toolbox（延迟 2 秒，等 /scan 就绪）
    slam_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(nav_dir, 'config', 'slam_toolbox.yaml'),
                {'use_sim_time': False},
            ],
        )],
    )

    # 节点 3: Nav2（延迟 5 秒，等 SLAM 就绪）
    nav2_params = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    nav2_lifecycle = TimerAction(
        period=5.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ],
            }],
        )],
    )

    nav2_controller = TimerAction(
        period=5.5,
        actions=[Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}],
        )],
    )

    nav2_planner = TimerAction(
        period=5.5,
        actions=[Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}],
        )],
    )

    nav2_behavior = TimerAction(
        period=5.5,
        actions=[Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}],
        )],
    )

    nav2_bt = TimerAction(
        period=5.5,
        actions=[Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}],
        )],
    )

    # 节点 4: 检测器（延迟 3 秒）
    detector_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='go2w_detector',
            executable='detector_node',
            name='go2w_detector',
            output='screen',
            parameters=[config_file],
        )],
    )

    # 节点 5: 编排器（延迟 7 秒，等 Nav2 就绪）
    orchestrator_node = TimerAction(
        period=7.0,
        actions=[Node(
            package='go2w_orchestrator',
            executable='orchestrator_node',
            name='go2w_orchestrator',
            output='screen',
            parameters=[config_file],
        )],
    )

    # 节点 6: Web 桥接（延迟 8 秒）
    web_node = TimerAction(
        period=8.0,
        actions=[Node(
            package='go2w_web',
            executable='web_bridge_node',
            name='go2w_web_bridge',
            output='screen',
            parameters=[config_file],
        )],
    )

    return LaunchDescription([
        network_interface_arg,
        bridge_node,
        slam_node,
        nav2_lifecycle,
        nav2_controller,
        nav2_planner,
        nav2_behavior,
        nav2_bt,
        detector_node,
        orchestrator_node,
        web_node,
    ])
