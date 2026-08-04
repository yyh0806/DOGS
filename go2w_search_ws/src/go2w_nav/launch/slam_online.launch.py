"""Persistent online SLAM for unknown-floor frontier exploration.

The calibrated ``mid360_nav_bridge`` is the single owner of
``/scan_mid360``. This launch only starts SLAM Toolbox as the persistent
``/map_frontier_raw`` grid source. ``map_padding_bridge`` adds an unknown
border and republishes it as ``/map_frontier`` for Nav2. FAST_LIO and
``map_odom_fuser`` retain ownership of ``map -> odom`` so an erroneous 2D
scan match cannot corrupt the physical navigation frame.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        os.path.dirname(__file__), "..", "config", "slam_toolbox_online.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        # Keep this name aligned with the YAML root key. Renaming it causes
        # ROS 2 to silently ignore the complete parameter file.
        name='slam_toolbox',
        parameters=[config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("map", "/map_frontier_raw"),
            ("pose", "/slam_pose"),
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        slam_node,
    ])
