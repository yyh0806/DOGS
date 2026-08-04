"""sim_nav_bringup: 真机一致 Nav2 点选闭环 (Task 4, spec 2026-07-25 §9 step 7).

集成:
  - sim_fastlio_bringup (Task 2: gzserver + spawn go2_sim_livox URDF + fastlio_mapping)
  - SimTelemetryBridge (/odom_planar → /wheel_feedback, 让 motion 状态机跑)
  - nx_motion_node (GO2W_SIM=1 → SimSportGateway, 实机零改)
  - pointcloud_to_laserscan (/livox/lidar_PointCloud2 → /scan)
  - relay /Odometry → /odom (Nav2 默认期望 /odom)
  - nav2_bringup bringup_launch.py (slam_toolbox 在线建图 + Nav2 controller/planner/bt)

cmd_vel 链 (无回环):
  Nav2 → /cmd_vel → nx_motion_node._on_cmd_vel → motion_machine → Effect →
  SimSportGateway → /cmd_vel_motion → planar_move.
  nx_motion_node 不订阅 /cmd_vel_motion (只订阅 /cmd_vel operator + /cmd_vel_nav),
  避免 SimSportGateway 输出回环.

用法:
  source ~/go2w_ws/install/setup.bash
  ros2 launch go2w_sim sim_nav_bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('go2w_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # Task 2: gzserver + spawn go2_sim_livox URDF + fastlio_mapping
    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim_fastlio_bringup.launch.py')))

    # SimTelemetryBridge: /odom_planar → /wheel_feedback (motion 状态机 feedback)
    telemetry = Node(
        package='go2w_sim', executable='sim_telemetry_bridge', output='screen',
        parameters=[{'use_sim_time': True}])

    # nx_motion_node: GO2W_SIM=1 必须 (否则用真机 SportGatewayClient socket).
    # launch_ros Node additional_env 实测未生效, 用 ExecuteProcess bash 直接传 env.
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
            'min_height': -0.5, 'max_height': 1.0,
            'angle_min': -3.14159, 'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'range_min': 0.1, 'range_max': 20.0,
            'use_sim_time': True,
        }])

    # relay /Odometry (FastLIO) → /odom (Nav2 默认期望)
    relay_odom = ExecuteProcess(
        cmd=['ros2', 'run', 'topic_tools', 'relay', '/Odometry', '/odom'],
        output='screen')

    # nav2_bringup: slam_toolbox 在线建图 + Nav2 stack (controller/planner/bt/amcl)
    # map 参数必填 (nav2_bringup 要求); slam:=True 时 slam_toolbox 实际建图覆盖静态 map
    nav2_map = os.path.join(sim_share, 'maps', 'indoor_empty_map.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'False',
            'map': nav2_map,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items())

    # NOTE: motion 状态机集成 (nx_motion_node GO2W_SIM + SimTelemetryBridge) 在 Task 3
    # 验证 (SimSportGateway 7/7 + /wheel_feedback). Task 4 Nav2 闭环先用 planar_move
    # 直接 (Nav2 → /cmd_vel → planar_move), motion_machine BOOT_HOLD→PARKED 推进需
    # MotionIntent (仿真缺 web 发 intent), 留 motion 启动时序深调.
    # sim_odom_tf: /odom_planar → TF odom→base_footprint (绕 planar_move TF bug)
    odom_tf = Node(package='go2w_sim', executable='sim_odom_tf', output='screen',
                   parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        SetEnvironmentVariable('GO2W_SIM', '1'),
        fastlio, odom_tf, p2l, relay_odom, nav2,
    ])
