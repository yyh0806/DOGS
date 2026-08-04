"""sim_full_bringup: 真机一致全栈仿真 (spec 2026-07-25-real-fidelity-simulation-design §9).

整合 sim_fastlio(world 可选) + nav2 + motion(GO2W_SIM) + telemetry + web :8000.
所有模块 real 代码; 唯一简化 = SportGateway→SimSportGateway (Effect→/cmd_vel→planar_move)
+ 视频/AI/Gimbal try/except 退化 (前端显示"等待视频", 主服务不受影响).

用法 (PC WSL2):
  source ~/go2w_ws/install/setup.bash
  ros2 launch go2w_sim sim_full_bringup.launch.py
  # 切 MVP 单房:
  ros2 launch go2w_sim sim_full_bringup.launch.py world:=$(ros2 pkg prefix go2w_sim)/share/go2w_sim/worlds/indoor_empty.world

上真机:
  - 不设 GO2W_SIM (nx_motion_node 走真机 SportGatewayClient socket)
  - GO2W_WEB_DIR=~/go2w_ws/web (真机 web 部署路径)
  - 去掉 nav2_bringup/sim_odom_tf/sim_telemetry_bridge/sim_fastlio (真机 FastLIO+nav2-3d systemd 起)

importer: 用户 `ros2 launch go2w_sim sim_full_bringup.launch.py` (CLI, 无 python importer).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('go2w_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    go2w_nav_share = get_package_share_directory('go2w_nav')

    # FastRTPS profile (禁 SHM 强制 UDP) —— 实测致 libgazebo_ros_init /clock
    # publisher 注册失败 (Publisher count 0 → 24 节点时钟冻结) + gzserver SIGFPE -8
    # 反复崩溃 (Gazebo 插件间 DDS 通信断, 某插件计算异常). 改用默认 DDS (SHM+UDP),
    # SHM 偶发 "open_and_lock_file" lock 但不全断, 可接受. fastudp_profile.xml
    # 保留作备用 (严重 SHM 冲突时手动 export FASTRTPS_DEFAULT_PROFILES_FILE).
    # 真机 NX 不在此 launch 设 profile (systemd 各 service 自管 RMW).
    # fastprofile = os.path.join(sim_share, 'config', 'fastudp_profile.xml')
    # set_fastprofile = SetEnvironmentVariable(
    #     'FASTRTPS_DEFAULT_PROFILES_FILE', fastprofile)

    # default world = indoor_rooms (多房间 frontier 探索); 可 override 回 MVP 单房
    default_world = os.path.join(sim_share, 'worlds', 'indoor_rooms.world')
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='indoor_rooms.world (frontier 多房) | indoor_empty.world (MVP)')

    # Task 2: gzserver + spawn go2_sim_livox URDF + fastlio_mapping (world 可选)
    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim_fastlio_bringup.launch.py')),
        launch_arguments=[('world', LaunchConfiguration('world'))])

    # SimTelemetryBridge: /odom_planar → /wheel_feedback (sport_mode 速度推断:
    # 停→6/JOINT_LOCK 推进 BOOT_HOLD→PARKED; 动→3/WHEEL 推进 ACTIVATING→NAV_ACTIVE)
    telemetry = Node(
        package='go2w_sim', executable='sim_telemetry_bridge', output='screen',
        parameters=[{'use_sim_time': True}])  # wall stamp, 配合 web/motion use_sim_time=false

    # nx_motion_node: GO2W_SIM=1 → SimSportGateway (Effect→/cmd_vel→planar_move).
    # launch_ros Node additional_env 不生效 (go2w_bridge 缓存), 用 ExecuteProcess bash.
    motion = ExecuteProcess(
        cmd=['bash', '-c',
             'GO2W_SIM=1 exec ros2 run go2w_bridge nx_motion_node '
             '--ros-args -p use_sim_time:=true '
             '--remap scan_mid360:=/scan'],
        output='screen')

    # pointcloud_to_laserscan: livox PointCloud2 → /scan (Nav2 + scan_watchdog)
    p2l = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='p2l', output='screen',
        remappings=[('cloud_in', '/livox/lidar_PointCloud2'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link',
            'min_height': 0.10, 'max_height': 1.0,
            'angle_min': -3.14159, 'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'range_min': 0.35, 'range_max': 20.0,
            'use_sim_time': True,
        }])

    # mock_scan: 10Hz 假 LaserScan 兜底 livox WSL2 退化 (visualize=false 后 brztink6u 偶发
    # 不发点云 → /scan 空 → nav_scan_fresh=False → motion 不激活 → 狗不位移).
    # b463cl71d/brztink6u 真链路已验证, 本节点是环境退化 fallback.
    mock_scan = Node(
        package='go2w_sim', executable='mock_scan_node', output='screen',
        parameters=[{'use_sim_time': True}])  # wall stamp, motion use_sim_time=false 一致

    # mock_planar_move: /cmd_vel 运动学积分 → /odom_planar + TF (绕过 gzserver planar_move
    # WSL2 SIGFPE 崩 → planar_move 死 → 狗不动). 用户允许运动模型简化.
    mock_planar_move = Node(
        package='go2w_sim', executable='mock_planar_move_node', output='screen',
        parameters=[{'use_sim_time': True}])  # wall dt, 避免 gzserver 崩 /clock 冻结 dt=0

    # slam_toolbox 在线建图 (订 /scan → /map_frontier_raw) + map_padding_bridge
    # (/map_frontier_raw → /map_frontier) 让 room_orchestrator frontier_explore 有地图.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2w_nav_share, 'launch', 'slam_online.launch.py')),
        launch_arguments=[('use_sim_time', 'true')])
    map_padding = Node(
        package='go2w_bridge', executable='map_padding_bridge', output='screen',
        parameters=[{'use_sim_time': True}])

    # relay /Odometry (FastLIO) → /odom (Nav2 默认期望 + web diagnostic)
    relay_odom = ExecuteProcess(
        cmd=['ros2', 'run', 'topic_tools', 'relay', '/Odometry', '/odom'],
        output='screen')
    # relay 仿真话题名 → web 期望名 (web 订阅 /scan_mid360 /mid360/points_nav /localization_pose)
    relay_scan = ExecuteProcess(
        cmd=['ros2', 'run', 'topic_tools', 'relay', '/scan', '/scan_mid360'],
        output='screen')
    relay_points = ExecuteProcess(
        cmd=['ros2', 'run', 'topic_tools', 'relay',
             '/livox/lidar_PointCloud2', '/mid360/points_nav'],
        output='screen')
    # sim_amcl_to_odom: /amcl_pose (PoseWithCovarianceStamped) → /localization_pose (Odometry).
    # web 订阅 /localization_pose 是 Odometry, amcl 发 PoseWithCovarianceStamped, 类型不匹配.
    # topic_tools relay 不能跨类型, 用转换节点 (替换原 relay_loc).
    sim_amcl_to_odom = Node(
        package='go2w_sim', executable='sim_amcl_to_odom', output='screen',
        # use_sim_time=false: stamp=wall, 配合 web use_sim_time=false 让 loc age 小
        # (gzserver /clock DDS 不稳致 sim clock 冻结, web now sim 跟 stamp 差大 → stale)
        parameters=[{'use_sim_time': True}])

    # nav2_bringup: Nav2 stack (controller/planner/bt/amcl, slam:=False 用静态 map)
    # warehouse_map (50x50m 全 free) 替代 indoor_empty (10x10m): warehouse world 大,
    # 用户点击 x>5m 在 indoor_empty map 外 (unknown) → planner failed. warehouse_map
    # 覆盖 ±25m 让点击位置可达.
    nav2_map = os.path.join(sim_share, 'maps', 'warehouse_map.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'False',
            'map': nav2_map,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items())

    # map→odom 静态 identity TF: 仿真 odom=map 初始对齐, FastLIO 无全局漂移.
    # 用 static_transform_publisher (发 /tf_static, latched, stamp=0 永久有效) 替代
    # sim_odom_tf rclpy 定时广播 (use_sim_time 低频 /clock 下 timer 触发不稳,
    # map frame 间歇从 TF buffer 丢失 → nav2 "Invalid frame map" → nav_scan_stale).
    # odom→base_footprint 由 planar_move 独占 (去 FASTRTPS 后稳定).
    # 不设 use_sim_time: stamp=0 永久, 不受仿真时钟波动影响 (nav2 use_sim_time 查询
    # 任意时间都返回 static TF).
    map_odom_static = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],  # stamp=sim 匹配 nav2 sim buffer
        output='screen')

    # nx_web_server: real web 代码 (AI/Gimbal/Lidar try/except 退化; websockets 已装).
    # GO2W_WEB_DIR 默认 PC /mnt/c 路径; 真机设为 ~/go2w_ws/web.
    web_dir = os.environ.get(
        'GO2W_WEB_DIR',
        '/mnt/c/Users/ROG/yangyuhui/DOGS/go2w_search_ws/web')
    web = ExecuteProcess(
        cmd=['bash', '-c',
             f'cd "{web_dir}" && GO2W_SIM=1 GO2W_FRONTIER_NAV_TIMEOUT=600 exec python3 nx_web_server.py '
             '--ros-args -p use_sim_time:=true -p localization_max_tilt:=30.0'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        declare_world,
        # SIGFPE 根因 (gdb 定位): libros2_livox.so LivoxPointsPlugin::OnNewLaserScans →
        # ignition::math Quaternion::Euler → glibc trig (sin/cos) AVX/FMA 实现在 WSL2 虚拟化
        # 下都 SIGFPE (__sin_fma, __cos_avx 均崩, 输入 -0 触发). 禁 AVX+AVX2+FMA 让 glibc 用 SSE2.
        SetEnvironmentVariable('GLIBC_TUNABLES', 'glibc.cpu.hwcaps=-AVX,-AVX2,-FMA'),
        # LD_PRELOAD mycos.so: 泰勒 cos/sin 替换 glibc do_cos (WSL2 glibc 2.35 do_cos(0)
        # SIGFPE 根治, 让真 livox Quaternion::Euler 工作). 编译: tools/mycos.c.
        SetEnvironmentVariable('LD_PRELOAD', '/root/mycos.so'),
        fastlio, map_odom_static, p2l, slam, map_padding,
        relay_odom, relay_scan, relay_points, sim_amcl_to_odom,
        nav2, telemetry, motion, web,
    ])
