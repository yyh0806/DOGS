"""Go2W 在线 SLAM 启动 (无预建图 frontier 探索用, 阶段G plan 2026-07-03 §3.1)。

================================================================================
职责
================================================================================
本 launch 启动**在线 SLAM** 子集, 供 RoomSearchOrchestrator._run_frontier_explore 使用:
  1. pointcloud_to_laserscan: MID360 `/livox/lidar` (PointCloud2) → `/scan_mid360` (LaserScan)
     切片高度沿用 spec-stage-d 决策 6 已验证值 (min_height=0.10, max_height=1.20)。
  2. async_slam_toolbox_node (实例名 slam_toolbox_frontier): 订阅 `/scan_mid360`,
     发 `/map_frontier` (OccupancyGrid) + `map → odom` TF。

================================================================================
运维红线 (Critical, 改本文件务必保住)
================================================================================
  - **使用前必须停掉 slam.launch.py (mapping 模式)**, 否则 TF 多源跳变
    (两个节点都发 map→odom, TF 跳变, Nav2 规划崩溃)。
    本 launch 单实例运行, slam_toolbox_frontier 仍发 map→odom,
    topic /map_frontier 的 header.frame_id = "map", Nav2 直接复用现有 costmap 配置零改动。
  - **无 static_transform_publisher** (slam_toolbox 发 map→odom, nx_sensor 发 odom→base_link,
    无 TF 桥。加了会多源 TF 跳变, Critical)。
  - topic 解耦: 用 `/scan_mid360` (不覆盖 nx_sensor 的 `/scan`),
    用 `/map_frontier` (不覆盖 mapping 模式的 `/map`)。
  - use_sensor_data_qos=false 在 slam_toolbox_online.yaml (spec-slam 决策 4 头号风险)。

================================================================================
运行 (NX 上, 先停 slam.launch.py, nx_sensor 先起)
================================================================================
  # 1. 停掉旧 mapping 实例 (如果还在跑)
  ros2 lifecycle set /slam_toolbox shutdown  # 或直接 kill slam.launch.py 进程
  # 2. 起在线 frontier SLAM
  ros2 launch go2w_nav slam_online.launch.py
  # 3. 验证: /scan_mid360 有数据
  ros2 topic echo /scan_mid360 --once
  # 4. 验证: /map_frontier 有 OccupancyGrid
  ros2 topic echo /map_frontier --once
  # 5. 验证: 只有一个 map→odom 发布者
  ros2 run tf2_tools view_frames  # 期望: only slam_toolbox_frontier publishes map→odom
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'slam_toolbox_online.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ----------------------------------------------------------------------
    # 1. pointcloud_to_laserscan: MID360 点云 → 2D LaserScan
    #    切片高度沿用 spec-stage-d 决策 6 已验证值 (min=0.10, max=1.20),
    #    避免吃地面 (min 太低) 或天花板 (max 太高)。
    #    target_frame=base_link: scan 直接在 base_link 系, 无需 laser_frame TF。
    #    发 /scan_mid360 (不覆盖 nx_sensor 的 /scan)。
    # ----------------------------------------------------------------------
    p2l_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_mid360',
        parameters=[{
            'target_frame': 'base_link',
            'min_height': 0.10,
            'max_height': 1.20,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872,  # 0.5 度
            'range_min': 0.15,
            'range_max': 10.0,
            'use_inf': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan_mid360'),
        ],
        output='screen',
    )

    # ----------------------------------------------------------------------
    # 2. async_slam_toolbox_node (实例名 slam_toolbox_frontier):
    #    订阅 /scan_mid360, 发 /map_frontier (OccupancyGrid) + map→odom TF。
    #    单实例运行 (要求运维先停 slam.launch.py), frame_id 用 "map"。
    # ----------------------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox_frontier',
        parameters=[config, {
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('scan', '/scan_mid360'),
            ('map', '/map_frontier'),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        p2l_node,
        slam_node,
    ])
