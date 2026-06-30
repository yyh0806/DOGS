"""Go2W 阶段F: Nav2 slim 导航启动 (slam_toolbox 降级路线, 阶段F)。

本 launch 是 nav2_3d.launch.py 的 slam_toolbox 路线适配派生版 (spec-slam.md 决策 7)。
diff 极小 (与 3d 版对比):
  - 删 pointcloud_to_laserscan  (nx_sensor 已发 /scan, 不需要 p2l)
  - 删 2 个 static_transform_publisher  (slam_toolbox 发 map→odom, nx_sensor 发 odom→base_link, 无 TF 桥)
  - params_file 默认 nav2_params_slim.yaml  (不是 nav2_params_3d.yaml)
  - 无 TimerAction 编排  (无 p2l/TF 桥前置依赖, 直接起 Nav2)
  - lifecycle node_names 与 3d 版一致 (controller/planner/behavior/bt_navigator/waypoint_follower,
    不含 amcl/map_server/slam_toolbox)

不改 nav2_3d.launch.py (阶段D 红线, commit 9b0c397 已 gan 收敛)。两 launch 并存,
slam_toolbox 路线用本文件, FAST_LIO 路线用 nav2_3d.launch.py。**两路线互斥, 不可混起**
(都依赖 map→odom TF, 混起会多源 TF 跳变)。

================================================================================
阶段F 依赖 (前置, 不在本 launch 启动)
================================================================================
本 launch 只管 Nav2 slim stack。下列节点由独立 launch / systemd 提供,
启动顺序见 docs/slam_runbook.md:
  1. nx_sensor (go2w-sensor.service, 阶段A)   → 发 /scan /imu /odom + odom→base_link TF
  2. slam_toolbox (slam.launch.py, 阶段F)     → 发 /map + map→odom TF
                                                (mode:=localization 时发静态载入图;
                                                 mode:=mapping 建图时也能导航, 但推荐先建图再导航)
  3. nx_motion_node (阶段A)                   → 订阅 /cmd_vel 控狗
  4. nx_web_server (阶段A/B/E)                → Nav2 action client 在此进程内

================================================================================
关键红线 (rubric Critical, 改本文件务必保住)
================================================================================
  - /cmd_vel 零反转: controller_server **无** /cmd_vel remap (Nav2 默认 → nx_motion_node
    直接消费, REP-103 全程一致, 继承阶段D 决策 5)。
  - 不启 amcl: lifecycle_manager_navigation.node_names **不含 amcl** (slam_toolbox localization
    替代, amcl 会抢 map→odom TF)。
  - 不启 map_server: slam_toolbox 自己发 /map (OccupancyGrid), map_server 会与之冲突。
  - 不启 slam_toolbox: 由 slam.launch.py 启 (职责单一)。
  - 不启 nx_sensor / nx_motion_node / nx_web: 阶段A 独立 launch / systemd。
  - 无 static_transform_publisher: slim 路线 TF 链由 slam_toolbox + nx_sensor 直接覆盖,
    无 TF 桥 (与 3d 版的 map↔camera_init, body↔base_link static 不同)。
  - 无 pointcloud_to_laserscan: nx_sensor 已发 /scan (狗自带 LiDAR 投影)。
  - action 名不 remap: bt_navigator 默认暴露 /navigate_to_pose (阶段E 客户端契约)。

运行 (NX 上, 前置 nx_sensor + slam_toolbox localization 先起):
  ros2 launch go2w_nav nav2_slim.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    go2w_nav_dir = get_package_share_directory('go2w_nav')
    # 【slim diff】默认加载 nav2_params_slim.yaml (不是 nav2_params_3d.yaml)
    default_params = os.path.join(go2w_nav_dir, 'config', 'nav2_params_slim.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ----------------------------------------------------------------------
    # 1. Nav2 全栈 (navigation_launch, 默认不启 amcl/map_server, 符合决策 2)
    #    加载 nav2_params_slim.yaml (bt_navigator/controller/planner/behavior/waypoint)。
    #    **不加 /cmd_vel remap** (决策 5: Nav2 默认发 /cmd_vel → nx_motion_node 直接消费,
    #    REP-103 全程一致, 零反转)。**不 remap action 名** (/navigate_to_pose 默认,
    #    阶段E 客户端契约)。**不 remap /odom** (yaml 显式 odom_topic: /odom, slim diff ①)。
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
    # 2. lifecycle_manager_navigation (navigation_launch 不自带, 必须单独启)
    #    autostart=True 让 Nav2 节点自动 configure→activate。
    #    node_names 与 3d 版一致: controller/planner/behavior/bt_navigator/waypoint_follower。
    #    **不含 amcl** (决策 2, slam_toolbox localization 发 map→odom 替代),
    #    **不含 map_server** (slam_toolbox 自己发 /map),
    #    **不含 slam_toolbox** (由 slam.launch.py 启, 职责单一)。
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
                # ⚠️ 不含 amcl (决策 2, slam_toolbox localization 发 map→odom)
                # ⚠️ 不含 map_server (slam_toolbox 自己发 /map OccupancyGrid)
                # ⚠️ 不含 slam_toolbox (由 slam.launch.py 启, 职责单一)
            ],
        }],
        output='screen',
    )

    return LaunchDescription([
        # params_file: 默认 nav2_params_slim.yaml (slam_toolbox 路线适配派生, slim diff 4 处)
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        nav2,
        lifecycle_manager,
    ])
