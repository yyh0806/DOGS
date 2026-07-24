"""M1a spawn launch: gzserver + indoor_empty.world + go2_sim URDF spawn.

Bridges /cmd_vel (planar_move consumes) and /clock. No Nav2/FastLIO here —
Task 1 only verifies kinematic-base teleop does not penetrate walls.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2w_sim')
    world = os.path.join(pkg_share, 'worlds', 'indoor_empty.world')
    urdf = os.path.join(pkg_share, 'urdf', 'go2_sim.urdf')
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world,
            ],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'go2_sim', '-file', urdf],
            output='screen',
        ),
    ])
