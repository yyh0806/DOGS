"""Go2W 阶段F: slam_toolbox 2D 建图/定位启动 (降级路径, 替代未就绪的 FAST_LIO)。

================================================================================
职责与组合
================================================================================
本 launch **只管 slam_toolbox**。启动它 (mapping 或 localization 二选一),
吃 nx_sensor 的 /scan 建图/定位, 发 /map + map→odom TF。

nx_sensor / nx_motion_node / nx_web / Nav2 / amcl / map_server **都不在本 launch 启**,
由独立 launch / systemd 提供。启动顺序见 docs/slam_runbook.md。

完整 TF 链 (谁发布, 不能多源, spec-slam.md 决策 3):
  map ──(slam_toolbox, /tf, ≈10Hz)──▶ odom ──(nx_sensor, /tf, 50Hz)──▶ base_link
  /scan 直接在 base_link 系 (nx_sensor 投影时假设零偏移, 无 laser_frame, 无 TF 桥)

================================================================================
双模式 (mode arg, 决策 1)
================================================================================
  mode:=mapping       → async_slam_toolbox_node      (从头建图, /map 实时生长)
  mode:=localization  → localization_slam_toolbox_node (载入 .posegraph+.data, scan-matching 定位)

二者互斥 (IfCondition, 同一 name=slam_toolbox, 不会同时起)。建图完成后用
/slamp_toolbox/save_map service 存图, 再切 localization 载入。运维流程见 runbook。

================================================================================
关键红线 (rubric Critical, 改本文件务必保住)
================================================================================
  - use_sensor_data_qos=false 在 yaml (决策 4 头号风险, QoS 不匹配静默失败);
    launch 不覆盖, 由 yaml 统一。
  - 无 static_transform_publisher (slam_toolbox 发 map→odom, nx_sensor 发 odom→base_link,
    无 TF 桥。加了会多源 TF 跳变, Critical)。
  - 不启 amcl / map_server / Nav2 (slam_toolbox localization 替代 amcl, 自己发 /map)。
  - 不启 nx_sensor / nx_motion_node / nx_web (阶段A 独立 launch / systemd)。

================================================================================
运行 (NX 上, nx_sensor 先起)
================================================================================
  ros2 launch go2w_nav slam.launch.py                                  # 默认 mapping
  ros2 launch go2w_nav slam.launch.py mode:=localization map_file:=/home/nx/maps/room
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # slam_toolbox.yaml: mapping + localization 共用 (use_sensor_data_qos=false 等关键参数在此)
    config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'slam_toolbox.yaml')

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')

    # IfCondition 切 executable (决策 1): mode==mapping 起 async, mode==localization 起 localization
    is_mapping = IfCondition(
        PythonExpression(["'", mode, "' == 'mapping'"]))
    is_localization = IfCondition(
        PythonExpression(["'", mode, "' == 'localization'"]))

    # ----------------------------------------------------------------------
    # 1. mapping 模式: async_slam_toolbox_node
    #    从头建图, /map 实时生长 (map_update_interval=2.0), 发 map→odom TF。
    #    完成后调 /slam_toolbox/save_map service 存 .posegraph + .data。
    # ----------------------------------------------------------------------
    mapping_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        condition=is_mapping,
        parameters=[config, {
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ----------------------------------------------------------------------
    # 2. localization 模式: localization_slam_toolbox_node (替代 amcl, 决策 1/2)
    #    启动载入 map_file_name 指向的 .posegraph + .data, 持续 scan-matching
    #    发 map→odom (替代 amcl 粒子滤波), /map 发布静态载入的地图。
    #    map_start_pose=[0,0,0]: 建图起始位姿 = map 原点 identity (建图从原点开始)。
    # ----------------------------------------------------------------------
    localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        condition=is_localization,
        parameters=[config, {
            'use_sim_time': use_sim_time,
            'map_file_name': map_file,
            'map_start_pose': [0.0, 0.0, 0.0],
        }],
        output='screen',
    )

    return LaunchDescription([
        # mode: mapping (建图, 默认) | localization (载入图定位, 替代 amcl)
        DeclareLaunchArgument('mode', default_value='mapping',
                              choices=['mapping', 'localization']),
        # map_file: localization 模式用, .posegraph 路径 (无扩展名, slam_toolbox 自带序列化)
        # mapping 模式忽略此 arg
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        mapping_node,
        localization_node,
    ])
