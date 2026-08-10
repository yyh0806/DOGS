"""sim_fastlio_bringup: 真机一致仿真 Task2 bringup (spec 2026-07-25 §9 step 6).

gzserver + indoor_empty.world + spawn(go2_sim_livox URDF: planar_move + Livox
CustomMsg + IMU) + robot_state_publisher + fast_lio(fastlio_mapping, mid360.yaml,
use_sim_time).

静止原点验证: /Odometry 三轴 max<0.02m (test_fastlio_static_origin).

用法:
  source ~/go2w_ws/install/setup.bash
  ros2 launch go2w_sim sim_fastlio_bringup.launch.py

环境变量 GO2W_NO_GAZEBO=1 跳过 gzserver+spawn+fastlio (纯 mock 模式, WSL2 用).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('go2w_sim')
    fastlio_share = get_package_share_directory('fast_lio')
    default_world = os.path.join(sim_share, 'worlds', 'indoor_empty.world')
    default_urdf = os.path.join(sim_share, 'urdf', 'go2_sim_livox.urdf.xacro')

    # GO2W_NO_GAZEBO=1: WSL2 跳过 gzserver+spawn+fastlio, 纯 mock 节点跑导航
    no_gazebo = PythonExpression(["'", os.environ.get("GO2W_NO_GAZEBO", ""), "' == '1'"])

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Gazebo world (indoor_empty.world | indoor_rooms.world)')

    xacro_cmd = Command(['xacro ', default_urdf])

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world')],
        output='screen',
        condition=UnlessCondition(no_gazebo),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(xacro_cmd, value_type=str),
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'go2_sim', '-topic', '/robot_description',
                   '-x', '0', '-y', '0', '-z', '0'],
        output='screen',
        condition=UnlessCondition(no_gazebo),
    )

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'use_sim_time': 'true',
            'rviz': 'false',
        }.items(),
        condition=UnlessCondition(no_gazebo),
    )

    no_gazebo_log = LogInfo(
        msg='GO2W_NO_GAZEBO=1: skipping gzserver/spawn/fastlio, mock nodes only',
        condition=IfCondition(no_gazebo),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gzserver, rsp, spawn, fastlio, no_gazebo_log,
    ])
