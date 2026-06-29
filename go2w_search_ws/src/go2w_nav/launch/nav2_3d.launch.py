"""Go2W 阶段3: Nav2 3D导航启动文件 (NX本机)

组合: MID360点云 → pointcloud_to_laserscan → /scan
      + FAST_LIO 定位 (camera_init→body, 需TF桥到map→base_link)
      + Nav2 规划 (吃点云+scan)

前置:
  1. livox_ros_driver2 已启动 (发 /livox/lidar + /livox/imu)
  2. FAST_LIO 已启动 (发 /Odometry + camera_init→body TF)
  3. nx_motion_node 已启动 (订阅 /cmd_vel 控狗)

运行 (NX上):
  ros2 launch go2w_nav nav2_3d.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    go2w_nav_dir = get_package_share_directory('go2w_nav')
    default_params = os.path.join(go2w_nav_dir, 'config', 'nav2_params_3d.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # pointcloud_to_laserscan: MID360点云 → 2D /scan
    # 切0.1~1.2m高度, 抓桌腿/人腿/墙, 剔地面
    p2l = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.10,
            'max_height': 1.20,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087266,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 0.001,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # TF桥: FAST_LIO发camera_init→body, Nav2要map→base_link
    # 用static_transform近似: map=camera_init, base_link=body (雷达装在狗中心时成立)
    # 更精确的做法: robot_localization EKF拆分, 但起步用static够用
    tf_bridge_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
    )
    tf_bridge_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_body_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link'],
    )

    # Nav2 全栈 (lifecycle_manager 自动激活)
    nav2 = Node(
        package='nav2_bringup',
        executable='navigation_launch',
        name='nav2_bringup',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        p2l,
        tf_bridge_map,
        tf_bridge_body,
        nav2,
    ])
