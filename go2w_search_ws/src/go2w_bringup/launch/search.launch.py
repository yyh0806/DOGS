"""Go2W 全系统启动文件 (ROS2 原生模式)。

不使用 go2w_bridge (CycloneDDS segfault 问题)。
直接订阅 Go2W 机器狗原生 ROS2 topic:
  - /utlidar/cloud_base → pointcloud_to_laserscan → /scan
  - /cmd_vel → 机器狗运动控制
  - /uslam/frontend/odom → 里程计

用法:
  ros2 launch go2w_bringup search.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('go2w_nav')
    bringup_dir = get_package_share_directory('go2w_bringup')
    nav2_params = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    config_file = os.path.join(bringup_dir, 'config', 'default.yaml')

    # 1. PointCloud2 → LaserScan 转换 (机器人发 /utlidar/cloud_base)
    pc_to_scan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pc_to_scan', output='screen',
        remappings=[
            ('cloud_in', '/utlidar/cloud_base'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872,
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )

    # 2. SLAM Toolbox (延迟2秒, 等/scan就绪)
    slam_params = os.path.join(nav_dir, 'config', 'slam_toolbox.yaml')
    slam_node = TimerAction(period=2.0, actions=[Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[slam_params, {'use_sim_time': False}],
    )])

    # 3. Nav2 导航栈 (延迟6秒, 让节点先注册)
    nav2_lifecycle = TimerAction(period=6.0, actions=[Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': False, 'autostart': True,
            'node_names': ['controller_server', 'planner_server',
                           'behavior_server', 'bt_navigator'],
        }],
    )])

    nav2_controller = TimerAction(period=5.5, actions=[Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel', '/cmd_vel')],
    )])

    nav2_planner = TimerAction(period=5.5, actions=[Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
    )])

    nav2_behavior = TimerAction(period=5.5, actions=[Node(
        package='nav2_recoveries', executable='recoveries_server',
        name='behavior_server', output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
    )])

    nav2_bt = TimerAction(period=5.5, actions=[Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
    )])

    return LaunchDescription([
        pc_to_scan,
        slam_node,
        nav2_lifecycle,
        nav2_controller,
        nav2_planner,
        nav2_behavior,
        nav2_bt,
    ])