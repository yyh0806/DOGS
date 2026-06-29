"""Go2W 阶段D: Nav2 3D 导航启动文件 (NX 本机服务端)。

组合:
  MID360 点云 → pointcloud_to_laserscan → /scan → costmap ObstacleLayer (主)
  + local_costmap VoxelLayer 直接吃 PointCloud2 (辅, 双保险)
  + FAST_LIO 定位 (camera_init→body TF + /Odometry, 阶段C 提供)
  + TF 桥临时 static_transform (map↔camera_init, body↔base_link)
  + Nav2 全栈 (navigation_launch + lifecycle_manager_navigation)

================================================================================
阶段C 依赖 (前置, 不在本 launch 启动)
================================================================================
本 launch 只管 Nav2 + p2l + TF 桥。下列节点由阶段A/C 的独立 launch 提供,
启动顺序见 docs/nav2_3d_runbook.md:
  1. livox_ros_driver2 (阶段C launch)        → 发 /livox/lidar + /livox/imu
  2. FAST_LIO (阶段C launch)                  → 发 /Odometry + camera_init→body TF
  3. nx_motion_node (阶段A)                   → 订阅 /cmd_vel 控狗
  4. nx_web_server (阶段A/B/E)                → Nav2 action client 在此进程内

================================================================================
TF 桥临时方案 (阶段C 就绪后删)
================================================================================
FAST_LIO 发 camera_init→body; Nav2 要 map→odom→base_link。临时用两个
static_transform_publisher 做 identity 桥接 (map==camera_init, body==base_link)。
**阶段C FAST_LIO 装好后**, 应配置 FAST_LIO 直接发 map→odom (改 FAST_LIO 的 frame
名参数 camera_init→map / body→odom, 再单独发 odom→base_link), 或用 robot_localization
EKF 拆分。届时删掉下面 tf_bridge_map / tf_bridge_body 两个节点。

前提假设 (顶部注释, 阶段C 装好后需复核):
  - 雷达装在狗中心 (body==base_link, 零偏移); 若装在头部需填真实偏移。
  - FAST_LIO 全局原点 = map 原点 (建图起始点 == map==camera_init)。

================================================================================
关键红线 (rubric Critical 项, 改本文件务必保住)
================================================================================
  - /cmd_vel 零反转: controller_server **无** /cmd_vel remap (Nav2 默认 → nx_motion_node
    直接消费, REP-103 全程一致, 决策 5)。
  - 不启 amcl: lifecycle_manager_navigation.node_names **不含 amcl** (FAST_LIO 发 map→odom,
    amcl 会抢 TF); 用 navigation_launch (默认不启 amcl)。
  - 不启 slam_toolbox: 与 FAST_LIO 互斥 (二者都发 map→odom 会 TF 冲突)。
  - 不启 map_server: FAST_LIO 不发 OccupancyGrid, global_costmap 靠 obstacle_layer 实时累积。
  - action 名不 remap: bt_navigator 默认暴露 /navigate_to_pose (阶段E 客户端契约)。

运行 (NX 上, 前置 4 节点先起):
  ros2 launch go2w_nav nav2_3d.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    go2w_nav_dir = get_package_share_directory('go2w_nav')
    default_params = os.path.join(go2w_nav_dir, 'config', 'nav2_params_3d.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ----------------------------------------------------------------------
    # 1. pointcloud_to_laserscan: MID360 点云 → 2D /scan
    #    参数 (决策 6, 复用休眠值, 对齐 SDK_CAPABILITIES MID360 特征):
    #      target_frame=base_link: scan 跟着 base_link 转 (local_costmap rolling window 需要)
    #      min/max_height 0.10~1.20: 切桌腿/人腿/墙, 剔地面和头顶
    #      range 0.2~20.0: MID360 有效距离内 (costmap obstacle_max_range 7.0 会再裁)
    #    ⚠️ 依赖 TF 树 map→odom→base_link 完整, 否则 transform_tolerance 内 lookup
    #       失败 → scan 为空 → costmap 无障碍 → 狗撞墙。
    # ----------------------------------------------------------------------
    p2l = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/lidar'),   # 外置 MID360; 用狗自带 utlidar 改 /utlidar/cloud_base
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.10,
            'max_height': 1.20,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087266,   # 0.5° 分辨率 (~720 点/scan)
            'scan_time': 0.1,               # 10Hz scan
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,                # 无回波用 inf (costmap 正确处理)
            'inf_epsilon': 0.001,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ----------------------------------------------------------------------
    # 2. TF 桥 (临时方案, 见顶部注释, 阶段C 就绪后删)
    #    identity static_transform: map==camera_init, body==base_link
    #    让 FAST_LIO 发的 camera_init→body 等价于 map→base_link 链路。
    # ----------------------------------------------------------------------
    tf_bridge_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    tf_bridge_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_body_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ----------------------------------------------------------------------
    # 3. Nav2 全栈 (navigation_launch, 默认不启 amcl/map_server, 符合决策 3)
    #    加载 nav2_params_3d.yaml (bt_navigator/controller/planner/behavior/waypoint)。
    #    **不加 /cmd_vel remap** (决策 5: Nav2 默认发 /cmd_vel → nx_motion_node 直接消费,
    #    REP-103 全程一致, 零反转)。**不 remap action 名** (/navigate_to_pose 默认,
    #    阶段E 客户端契约)。**不 remap /Odometry** (yaml 显式 odom_topic: /Odometry, 决策 1)。
    # ----------------------------------------------------------------------
    nav2 = Node(
        package='nav2_bringup',
        executable='navigation_launch',
        name='nav2_bringup',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
        # 显式注释: 此处**禁止**加 remappings=[('/cmd_vel', ...)] 反转 (狗乱转 Critical)。
    )

    # ----------------------------------------------------------------------
    # 4. lifecycle_manager_navigation (navigation_launch 不自带, 必须单独启)
    #    autostart=True 让 Nav2 节点自动 configure→activate。
    #    node_names **不含 amcl** (决策 3, FAST_LIO 替代), **不含 map_server**
    #    (FAST_LIO 不发 OccupancyGrid), **不含 slam_toolbox** (与 FAST_LIO 互斥)。
    #    **含 bt_navigator** (否则 /navigate_to_pose action 不暴露, 阶段E 客户端超时)。
    # ----------------------------------------------------------------------
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                # ⚠️ 不含 amcl (决策 3, FAST_LIO 发 map→odom)
                # ⚠️ 不含 map_server (FAST_LIO 不发 OccupancyGrid)
                # ⚠️ 不含 slam_toolbox (与 FAST_LIO 互斥, 会抢 map→odom TF)
            ],
        }],
        output='screen',
    )

    # ----------------------------------------------------------------------
    # 启动编排:
    #   T=0s: p2l + TF 桥 (先起, 让 TF 链 + /scan 就绪)
    #   T=2s: Nav2 stack + lifecycle_manager (等 TF 就绪, 否则启动时 lookup
    #         map→base_link 失败报错)
    # ----------------------------------------------------------------------
    nav2_stack = TimerAction(
        period=2.0,
        actions=[nav2, lifecycle_manager],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        p2l,
        tf_bridge_map,
        tf_bridge_body,
        nav2_stack,
    ])
