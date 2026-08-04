"""sim_fastlio_bringup: 真机一致仿真 Task2 bringup (spec 2026-07-25 §9 step 6).

gzserver + indoor_empty.world + spawn(go2_sim_livox URDF: planar_move + Livox
CustomMsg + IMU) + robot_state_publisher + fast_lio(fastlio_mapping, mid360.yaml,
use_sim_time).

静止原点验证: /Odometry 三轴 max<0.02m (test_fastlio_static_origin).

用法:
  source ~/go2w_ws/install/setup.bash
  ros2 launch go2w_sim sim_fastlio_bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('go2w_sim')
    fastlio_share = get_package_share_directory('fast_lio')
    default_world = os.path.join(sim_share, 'worlds', 'indoor_empty.world')
    default_urdf = os.path.join(sim_share, 'urdf', 'go2_sim_livox.urdf.xacro')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    # world 可选: indoor_empty.world (单房 MVP) / indoor_rooms.world (4房间 frontier 探索)
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Gazebo world (indoor_empty.world | indoor_rooms.world)')

    # xacro 处理 URDF (含 Livox CustomMsg 插件 + IMU 插件)
    xacro_cmd = Command(['xacro ', default_urdf])

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world')],
        output='screen',
    )

    # robot_state_publisher: xacro URDF -> /robot_description + 静态 TF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(xacro_cmd, value_type=str),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # spawn URDF from /robot_description topic (Task1 模式, executable=spawn_entity.py)
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'go2_sim', '-topic', '/robot_description',
                   '-x', '0', '-y', '0', '-z', '0'],
        output='screen',
    )

    # fast_lio: fastlio_mapping + mid360.yaml (话题 lid_topic=/livox/lidar,
    # imu_topic=/livox/imu 已对; use_sim_time; 无 rviz)
    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'use_sim_time': 'true',
            'rviz': 'false',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gzserver, rsp, spawn, fastlio,
    ])
