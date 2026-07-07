"""Go2W 阶段D: Nav2 3D 导航启动文件 (NX 本机服务端)。

组合:
  MID360 点云 → pointcloud_to_laserscan → /scan → costmap ObstacleLayer (主)
  + local_costmap VoxelLayer 直接吃 PointCloud2 (辅, 双保险)
  + FAST_LIO 定位 (camera_init→body TF + /Odometry, 阶段C 提供)
  + map→odom fuser (go2w_bridge, 根治 C1 base_link 双 parent 拓扑硬伤)
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
map→odom fuser (根治 critic C1 TF 拓扑硬伤, 2026-07-07 GAN-Flow)
================================================================================
原临时方案两个 static_transform_publisher (map→camera_init, body→base_link) identity
桥 → base_link 双 parent (body + nx_sensor 的 odom) → costmap two-trees (拓扑必然,
非偶发). 根治: map_odom_fuser 节点 (go2w_bridge) 订阅 camera_init→body (FastLIO) +
odom→base_link (nx_sensor 死推算), 算 map→odom = T(camera_init→body) × inv(T(odom→base_link))
发布. TF 树变单链: map ─(fuser 20Hz)─▶ odom ─(nx_sensor 50Hz)─▶ base_link.

前提假设:
  - 雷达装在狗中心 (body==base_link, 零偏移); 若装在头部需在 fuser 加 offset.
  - map == camera_init (建图起始原点, FastLIO 全局原点 = map 原点).

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
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # 2. map→odom fuser (根治 C1, 替代原两个 static_transform_publisher)
    #    见顶部注释. go2w_bridge 包, entry_point map_odom_fuser.
    #    必须与 p2l 同期起 (T=0), nav2 stack T=2s 给它时间发 map→odom.
    # ----------------------------------------------------------------------
    map_odom_fuser = Node(
        package='go2w_bridge',
        executable='map_odom_fuser',
        name='map_odom_fuser',
        parameters=[{
            'world_frame': 'map',
            'fastlio_world': 'camera_init',
            'fastlio_body': 'body',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_hz': 20.0,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ----------------------------------------------------------------------
    # 3. Nav2 全栈 (navigation_launch, 默认不启 amcl/map_server, 符合决策 3)
    #    加载 nav2_params_3d.yaml (bt_navigator/controller/planner/behavior/waypoint)。
    #    **不加 /cmd_vel remap** (决策 5: Nav2 默认发 /cmd_vel → nx_motion_node 直接消费,
    #    REP-103 全程一致, 零反转)。**不 remap action 名** (/navigate_to_pose 默认,
    #    阶段E 客户端契约)。**不 remap /Odometry** (yaml 显式 odom_topic: /Odometry, 决策 1)。
    # ----------------------------------------------------------------------
    # nav2_bringup 的 navigation_launch 是 **launch 文件, 不是 executable**
    # (/opt/ros/humble/lib/nav2_bringup libexec 目录不存在; 同 nx-online-debug-log 坑4)。
    # 必须 IncludeLaunchDescription, 不能 Node+executable。
    # **禁止** /cmd_vel remap (狗乱转 Critical, 决策 5 零反转)。
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            # 禁 navigation_launch 自带 lifecycle_manager 的 autostart
            # (它 node_names 含 waypoint_follower, 在本机 configure FATAL 会 abort 整个 bringup
            #  并连累 bt_navigator 回退; 由下方 lifecycle_manager_navigation 接管, 已移 waypoint_follower)
            'autostart': 'false',
        }.items(),
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
                # ⚠️ 不含 waypoint_follower (单点 navigate_to_pose 不需要;
                #   Humble 本机 waypoint_follower configure 报 error-state FATAL
                #   会 abort 整个 bringup。要 NavigateThroughPoses 再加回 + 修 yaml)
                # ⚠️ 不含 amcl (决策 3, FAST_LIO 发 map→odom)
                # ⚠️ 不含 map_server (FAST_LIO 不发 OccupancyGrid)
                # ⚠️ 不含 slam_toolbox (与 FAST_LIO 互斥, 会抢 map→odom TF)
            ],
        }],
        output='screen',
    )

    # ----------------------------------------------------------------------
    # 启动编排 (critic 二轮 High: 原 T=2s 不够 FastLIO 5-10s 就绪, fuser 静默 return
    #           不发 map→odom → nav2 activate 时 two-trees 重现. 改 T=8s 给 fuser 等
    #           FastLIO+nx_sensor TF 就绪 + 首次发布 map→odom 的时间):
    #   T=0s: p2l + map_odom_fuser (fuser lookup 失败时 throttle return, 等就绪后发 map→odom)
    #   T=8s: Nav2 stack + lifecycle_manager (等 fuser map→odom 就绪, 否则 bt_navigator
    #         activate 时 lookup map→base_link 失败 → C1 two-trees 重现)
    # ----------------------------------------------------------------------
    nav2_stack = TimerAction(
        period=8.0,
        actions=[nav2, lifecycle_manager],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        p2l,
        map_odom_fuser,
        nav2_stack,
    ])
