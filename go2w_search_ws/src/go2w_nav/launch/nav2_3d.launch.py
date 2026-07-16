"""Start the Go2W Nav2 stack with an isolated autonomous velocity channel.

External prerequisites are owned by persistent services and gated by
``bringup_slam_nav2.sh``:

* Livox + FAST_LIO provide mapping/localization;
* SLAM Toolbox provides the single ``map -> odom`` edge;
* ``map_odom_fuser.py`` provides the single ``odom -> base_link`` edge and
  time-synchronized ``/odom``/``/localization_pose`` outputs;
* ``mid360_nav_bridge.py`` provides leveled ``/mid360/points_nav`` and
  ``/scan_mid360`` from FAST_LIO's deskewed body cloud;
* ``nx_motion_node.py`` consumes operator ``/cmd_vel`` and separately gates
  autonomous ``/cmd_vel_nav`` immediately before Unitree SDK ``Move``.

Humble's stock ``navigation_launch.py`` hard-codes its final smoother output
to ``/cmd_vel``.  That aliases operator and autonomous commands, so the source
cannot be safety-gated at the SDK boundary.  The same stock nodes are declared
explicitly here with two private topics: controller ``cmd_vel_nav_raw`` and
final autonomous ``cmd_vel_nav``.  Recovery behaviors also publish only to the
final autonomous topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    go2w_nav_dir = get_package_share_directory('go2w_nav')
    default_params = os.path.join(go2w_nav_dir, 'config', 'nav2_params_3d.yaml')
    safe_dynamic_replan_bt = os.path.join(
        go2w_nav_dir,
        'behavior_trees',
        'navigate_to_pose_dynamic_safe.xml',
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': 'false',
                'default_nav_to_pose_bt_xml': safe_dynamic_replan_bt,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    component_common = {
        'parameters': [configured_params],
    }

    # Keep the controller's raw command private.  Only velocity_smoother may
    # promote it to the final autonomous channel consumed by nx_motion_node.
    controller = ComposableNode(
        package='nav2_controller',
        plugin='nav2_controller::ControllerServer',
        name='controller_server',
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav_raw')],
        **component_common,
    )
    smoother = ComposableNode(
        package='nav2_smoother',
        plugin='nav2_smoother::SmootherServer',
        name='smoother_server',
        remappings=tf_remaps,
        **component_common,
    )
    planner = ComposableNode(
        package='nav2_planner',
        plugin='nav2_planner::PlannerServer',
        name='planner_server',
        remappings=tf_remaps,
        **component_common,
    )
    behavior = ComposableNode(
        package='nav2_behaviors',
        plugin='behavior_server::BehaviorServer',
        name='behavior_server',
        remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
        **component_common,
    )
    navigator = ComposableNode(
        package='nav2_bt_navigator',
        plugin='nav2_bt_navigator::BtNavigator',
        name='bt_navigator',
        remappings=tf_remaps,
        **component_common,
    )
    velocity_smoother = ComposableNode(
        package='nav2_velocity_smoother',
        plugin='nav2_velocity_smoother::VelocitySmoother',
        name='velocity_smoother',
        remappings=tf_remaps + [
            ('cmd_vel', 'cmd_vel_nav_raw'),
            ('cmd_vel_smoothed', 'cmd_vel_nav'),
        ],
        **component_common,
    )

    # One manager, one explicit node set.  No amcl/map_server/slam_toolbox
    # (FAST_LIO owns localization), and no waypoint_follower (single-point
    # NavigateToPose only; the NX Humble waypoint node enters error on config).
    lifecycle_manager = ComposableNode(
        package='nav2_lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='go2w_lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
            ],
        }],
    )

    # Humble's isolated container gives every lifecycle component a dedicated
    # executor thread while all seven nodes share one DDS participant.  This
    # prevents the UDP-only graph fan-out from starving FAST_LIO when Nav2
    # activates, without serializing costmap and lifecycle callbacks.
    nav2_container = ComposableNodeContainer(
        name='go2w_nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        # ControllerServer and PlannerServer construct their costmap nodes at
        # runtime.  Supplying the file only to the loaded components does not
        # propagate overrides to those child nodes on Humble; the result is a
        # deceptively healthy costmap using /map and no MID360 source.  Match
        # the official Nav2 composed bringup and give the container the same
        # parameter file as well.
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[
            controller,
            smoother,
            planner,
            behavior,
            navigator,
            velocity_smoother,
            lifecycle_manager,
        ],
    )

    nav2_stack = TimerAction(
        period=8.0,
        actions=[nav2_container],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        nav2_stack,
    ])
