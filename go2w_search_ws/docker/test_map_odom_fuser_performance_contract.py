"""Regression contract for the NX map->odom fuser CPU usage."""

import ast
import math
import re
from pathlib import Path

import numpy as np
import pytest


FUSER = (
    Path(__file__).resolve().parents[1]
    / "src/go2w_bridge/go2w_bridge/map_odom_fuser.py"
)
ROOT = Path(__file__).resolve().parents[1]
SENSOR = ROOT / "src/go2w_bridge/go2w_bridge/nx_sensor_node.py"


PURE_FUNCTION_NAMES = (
    "_rpy_to_mat",
    "_build_static_tf",
    "_conjugate_pose",
    "_relative_planar_pose",
    "_propagate_map_pose",
    "_lio_message_age_is_fresh",
)

SENSOR_PURE_FUNCTION_NAMES = (
    "_transform_lidar_to_base",
    "_classify_base_scan_point",
    "_build_scan_ranges",
)


def _load_fuser_pure_functions():
    """Compile only numerical helpers so this contract never imports ROS."""
    tree = ast.parse(FUSER.read_text(encoding="utf-8"))
    definitions = {
        node.name: node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    missing = [name for name in PURE_FUNCTION_NAMES if name not in definitions]
    assert not missing, f"map_odom_fuser is missing pure helpers: {missing}"

    module = ast.Module(
        body=[definitions[name] for name in PURE_FUNCTION_NAMES],
        type_ignores=[],
    )
    ast.fix_missing_locations(module)
    namespace = {"math": math, "np": np}
    exec(compile(module, str(FUSER), "exec"), namespace)
    return {name: namespace[name] for name in PURE_FUNCTION_NAMES}


def _load_sensor_pure_functions():
    """Compile only scan geometry helpers; importing the module would require ROS."""
    tree = ast.parse(SENSOR.read_text(encoding="utf-8"))
    definitions = {
        node.name: node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    missing = [name for name in SENSOR_PURE_FUNCTION_NAMES if name not in definitions]
    assert not missing, f"nx_sensor_node is missing pure helpers: {missing}"

    module = ast.Module(
        body=[definitions[name] for name in SENSOR_PURE_FUNCTION_NAMES],
        type_ignores=[],
    )
    ast.fix_missing_locations(module)
    namespace = {"math": math}
    exec(compile(module, str(SENSOR), "exec"), namespace)
    return {name: namespace[name] for name in SENSOR_PURE_FUNCTION_NAMES}


def _method(tree, class_name, method_name):
    cls = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return next(
        node
        for node in cls.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == method_name
    )


def _assignment_map(node):
    assignments = {}
    for statement in ast.walk(node):
        if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
            continue
        assignments[ast.unparse(statement.targets[0])] = ast.unparse(statement.value)
    return assignments


def _is_lookup(call, target, source, time_expression):
    return (
        isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "lookup_transform"
        and [ast.unparse(arg) for arg in call.args[:3]]
        == [target, source, time_expression]
    )


@pytest.mark.parametrize(
    "stamp_ns, now_ns, max_age, future_tolerance, expected",
    [
        (9_700_000_000, 10_000_000_000, 0.35, 0.05, True),
        (9_650_000_000, 10_000_000_000, 0.35, 0.05, True),
        (9_649_999_999, 10_000_000_000, 0.35, 0.05, False),
        (10_050_000_000, 10_000_000_000, 0.35, 0.05, True),
        (10_050_000_001, 10_000_000_000, 0.35, 0.05, False),
        (0, 10_000_000_000, 0.35, 0.05, False),
        (9_900_000_000, 10_000_000_000, float("nan"), 0.05, False),
    ],
)
def test_lio_message_age_fails_closed_at_latency_and_clock_skew_boundaries(
    stamp_ns, now_ns, max_age, future_tolerance, expected,
):
    helper = _load_fuser_pure_functions()["_lio_message_age_is_fresh"]
    assert helper(
        stamp_ns,
        now_ns,
        max_age_sec=max_age,
        future_tolerance_sec=future_tolerance,
    ) is expected


def test_limits_openblas_threads_before_importing_numpy():
    """Small periodic inversions must not start OpenBLAS busy-wait workers."""
    tree = ast.parse(FUSER.read_text(encoding="utf-8"))
    numpy_import = next(
        index
        for index, statement in enumerate(tree.body)
        if isinstance(statement, (ast.Import, ast.ImportFrom))
        and (
            any(alias.name == "numpy" for alias in statement.names)
            if isinstance(statement, ast.Import)
            else statement.module == "numpy"
        )
    )

    assignments_before_numpy = {
        statement.targets[0].slice.value: statement.value.value
        for statement in tree.body[:numpy_import]
        if isinstance(statement, ast.Assign)
        and isinstance(statement.targets[0], ast.Subscript)
        and isinstance(statement.targets[0].value, ast.Attribute)
        and isinstance(statement.targets[0].value.value, ast.Name)
        and statement.targets[0].value.value.id == "os"
        and statement.targets[0].value.attr == "environ"
        and isinstance(statement.targets[0].slice, ast.Constant)
        and isinstance(statement.value, ast.Constant)
    }

    assert assignments_before_numpy.get("OPENBLAS_NUM_THREADS") == "1"


def test_global_costmap_keeps_a_fifty_metre_live_obstacle_window():
    """The rolling live costmap covers a 20 m leg without stale map authority."""
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )
    global_section = params.split("global_costmap:", 1)[1].split(
        "planner_server:", 1
    )[0]
    assert re.search(r"^\s+rolling_window:\s*true\s*(?:#.*)?$", global_section, re.M)
    assert not re.search(r"^\s+map_topic:", global_section, re.M)
    assert _costmap_plugins(global_section) == ["obstacle_layer", "inflation_layer"]
    for dimension in ("width", "height"):
        match = re.search(
            rf"^\s+{dimension}:\s*([0-9.]+)", global_section, re.M
        )
        assert match is not None
        assert float(match.group(1)) >= 50.0


def _costmap_sections(params):
    local = params.split("local_costmap:", 1)[1].split("global_costmap:", 1)[0]
    global_ = params.split("global_costmap:", 1)[1].split("planner_server:", 1)[0]
    return (local, global_)


def _costmap_plugins(section):
    plugin_match = re.search(r"^\s+plugins:\s*\[([^]]+)]", section, re.M)
    assert plugin_match is not None, "costmap is missing an explicit plugins list"
    return [item.strip().strip("\"") for item in plugin_match.group(1).split(",")]


def test_global_inflation_is_smaller_than_local_but_covers_robot_corner():
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )
    local_section, global_section = _costmap_sections(params)

    def value(section, name):
        match = re.search(rf"^\s+{name}:\s*([0-9.]+)", section, re.M)
        assert match is not None
        return float(match.group(1))

    local_radius = value(local_section, "inflation_radius")
    global_radius = value(global_section, "inflation_radius")
    global_scaling = value(global_section, "cost_scaling_factor")

    # The padded 0.90 x 0.64 m footprint has a 0.552 m corner radius.
    # Keep one 5 cm cell of global margin, but let the local controller own
    # the larger soft-clearance envelope around live obstacles.
    assert global_radius == pytest.approx(0.60)
    assert 0.552 + 0.04 <= global_radius < local_radius
    assert global_scaling == pytest.approx(4.0)


def test_costmaps_use_temporally_stable_mid360_scan_for_marking_and_clearing():
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )

    local_section, global_section = _costmap_sections(params)
    assert _costmap_plugins(local_section) == ["obstacle_layer", "inflation_layer"]
    assert _costmap_plugins(global_section) == ["obstacle_layer", "inflation_layer"]

    for section in (local_section, global_section):
        assert re.search(r"^\s+observation_sources:\s*mid360\s*$", section, re.M)
        assert re.search(r"^\s+topic:\s*/scan_mid360\s*$", section, re.M)
        assert re.search(r'^\s+data_type:\s*["\']?LaserScan["\']?\s*$', section, re.M)
        assert re.search(r"^\s+inf_is_valid:\s*True\s*$", section, re.M)
        assert re.search(
            r"^\s+min_obstacle_height:\s*0\.0\s*(?:#.*)?$", section, re.M
        )
        assert re.search(r"^\s+marking:\s*True\s*$", section, re.M)
        assert re.search(r"^\s+clearing:\s*True\s*$", section, re.M)
        assert re.search(
            r"^\s+expected_update_rate:\s*1\.8\s*(?:#.*)?$", section, re.M
        )

        assert "ObstacleLayer" in section
        assert "VoxelLayer" not in section
        assert "PointCloud2" not in section
        assert "/livox/lidar" not in section


def test_nav_diagnostic_parses_current_tf2_echo_translation_format():
    script = (ROOT / "docker/diagnose_nav2_goal.sh").read_text(encoding="utf-8")

    assert "Translation:" in script
    assert "sed -n" in script


def test_nav_launch_does_not_start_a_competing_sensor_converter():
    launch = (ROOT / "src/go2w_nav/launch/nav2_3d.launch.py").read_text(
        encoding="utf-8"
    )

    assert "pointcloud_to_laserscan" not in launch
    assert "p2l" not in launch
    assert "/livox/lidar" not in launch
    assert "nx_sensor_node" not in launch


def test_official_utlidar_extrinsic_projects_ground_below_obstacle_band():
    helpers = _load_sensor_pure_functions()
    transform = helpers["_transform_lidar_to_base"]
    classify = helpers["_classify_base_scan_point"]

    # Official Unitree Go2W base->radar joint: base <- utlidar_lidar.
    base_point = transform(
        0.0, 0.0, 0.5,
        0.28945, 0.0, -0.046825,
        0.0, 2.8782, 0.0,
    )

    assert base_point[2] < 0.05
    assert classify(*base_point, 0.05, 1.5, -0.25, 0.30, -0.20, 0.20, 10.0, 360) is None
    ground_only = helpers["_build_scan_ranges"](
        [base_point], 0.05, 1.5, -0.25, 0.30, -0.20, 0.20, 10.0, 360
    )
    assert all(math.isinf(value) and value > 0.0 for value in ground_only)


def test_utlidar_extrinsic_recovers_a_synthetic_base_obstacle_range():
    helpers = _load_sensor_pure_functions()
    transform = helpers["_transform_lidar_to_base"]
    build_ranges = helpers["_build_scan_ranges"]

    pitch = 2.8782
    rotation = np.array([
        [math.cos(pitch), 0.0, math.sin(pitch)],
        [0.0, 1.0, 0.0],
        [-math.sin(pitch), 0.0, math.cos(pitch)],
    ])
    translation = np.array([0.28945, 0.0, -0.046825])
    expected_base = np.array([0.65, -0.20, 0.40])
    lidar_point = rotation.T @ (expected_base - translation)
    recovered = transform(
        *lidar_point,
        *translation,
        0.0, pitch, 0.0,
    )

    np.testing.assert_allclose(recovered, expected_base, atol=1e-12)
    ranges = build_ranges(
        [recovered], 0.05, 1.5, -0.25, 0.30, -0.20, 0.20, 10.0, 360
    )
    finite = [value for value in ranges if math.isfinite(value) and value < 10.0]
    assert finite == [pytest.approx(math.hypot(expected_base[0], expected_base[1]))]


def test_close_obstacle_outside_self_box_is_finite_but_self_return_is_nan():
    helpers = _load_sensor_pure_functions()
    build_ranges = helpers["_build_scan_ranges"]
    args = (0.05, 1.5, -0.25, 0.30, -0.20, 0.20, 10.0, 360)

    close_obstacle = build_ranges([(0.31, 0.0, 0.30)], *args)
    obstacle_bin = 180
    assert close_obstacle[obstacle_bin] == pytest.approx(0.31)

    # A farther external hit in the same ray must not turn a nearer robot hit
    # into a clearing/max-range observation.
    self_return = build_ranges(
        [(0.80, 0.0, 0.30), (0.20, 0.0, 0.30)], *args
    )
    assert math.isnan(self_return[obstacle_bin])
    assert self_return[obstacle_bin] != 10.0

    # Self occlusion is geometric: a chassis return must block clearing even
    # when its z lies outside the external-obstacle height band.
    for self_z in (-0.40, 1.80):
        out_of_band_self = build_ranges([(0.20, 0.0, self_z)], *args)
        assert math.isnan(out_of_band_self[obstacle_bin])

    ground_outside_robot = build_ranges([(0.40, 0.0, -0.40)], *args)
    assert all(
        math.isinf(value) and value > 0.0
        for value in ground_outside_robot
    )


def test_sensor_uses_official_extrinsic_and_stops_fresh_stamping_stale_raw_scan():
    sensor = SENSOR.read_text(encoding="utf-8")
    tree = ast.parse(sensor)
    init = _method(tree, "NxSensorNode", "__init__")
    on_lidar = _method(tree, "NxSensorNode", "_on_lidar")
    publish_scan = _method(tree, "NxSensorNode", "_publish_scan")

    assert "declare_parameter('lidar_frame', 'utlidar_lidar')" in sensor
    assert "declare_parameter('lidar_to_base_x', 0.28945)" in sensor
    assert "declare_parameter('lidar_to_base_z', -0.046825)" in sensor
    assert "declare_parameter('lidar_to_base_pitch', 2.8782)" in sensor
    assert "declare_parameter('obstacle_min_height', 0.05)" in sensor
    assert "declare_parameter('obstacle_max_height', 1.5)" in sensor
    assert "declare_parameter('self_min_x', -0.25)" in sensor
    assert "declare_parameter('self_max_x', 0.30)" in sensor
    assert "declare_parameter('self_min_y', -0.20)" in sensor
    assert "declare_parameter('self_max_y', 0.20)" in sensor
    assert "declare_parameter('scan_frame', 'utlidar_scan')" in sensor
    assert "declare_parameter('raw_scan_timeout', 0.3)" in sensor
    assert "scan_min_range" not in sensor

    assert "_transform_lidar_to_base(" in ast.unparse(on_lidar)
    assert "_build_scan_ranges(" in ast.unparse(on_lidar)
    assert "msg.header.frame_id" in ast.unparse(on_lidar)
    assert "time.monotonic()" in ast.unparse(on_lidar)
    assert "self._last_raw_scan_monotonic = None" in ast.unparse(init)

    publish_source = ast.unparse(publish_scan)
    assert "time.monotonic() - last_raw_scan_monotonic" in publish_source
    assert "self._raw_scan_timeout" in publish_source
    stale_if = next(
        node
        for node in ast.walk(publish_scan)
        if isinstance(node, ast.If)
        and "self._raw_scan_timeout" in ast.unparse(node.test)
    )
    assert any(isinstance(node, ast.Return) for node in stale_if.body)
    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "publish"
        for node in ast.walk(stale_if)
    )


def test_nav2_uses_bounded_tf_tolerances_and_distinct_lifecycle_manager():
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )
    launch = (ROOT / "src/go2w_nav/launch/nav2_3d.launch.py").read_text(
        encoding="utf-8"
    )

    assert "transform_tolerance: 3.5" not in params
    # FollowPath and both live costmaps use bounded transform waits; there is
    # no persistent static layer in the navigation costmap.
    assert params.count("transform_tolerance: 0.5") == 4
    assert "'autostart': 'false'" in launch
    assert "name='go2w_lifecycle_manager_navigation'" in launch
    assert "name='lifecycle_manager_navigation'" not in launch


def test_nav2_components_share_one_isolated_dds_container():
    launch = (ROOT / "src/go2w_nav/launch/nav2_3d.launch.py").read_text(
        encoding="utf-8"
    )
    assert "ComposableNodeContainer(" in launch
    assert "executable='component_container_isolated'" in launch
    # The container itself must receive the full parameter file.  Costmaps are
    # child nodes created by controller/planner and otherwise silently fall
    # back to defaults (no MID360 observation source and /map rather than
    # /map_frontier) when components are dynamically loaded.
    container = launch.split("nav2_container = ComposableNodeContainer(", 1)[1]
    assert "parameters=[configured_params]" in container
    # Humble's smoother creates a transient-local endpoint.  Enabling intra-
    # process comms makes its configure transition fail because that transport
    # only permits volatile durability.
    assert "'use_intra_process_comms': True" not in launch
    assert launch.count("ComposableNode(") == 7
    for plugin in (
        "nav2_controller::ControllerServer",
        "nav2_smoother::SmootherServer",
        "nav2_planner::PlannerServer",
        "behavior_server::BehaviorServer",
        "nav2_bt_navigator::BtNavigator",
        "nav2_velocity_smoother::VelocitySmoother",
        "nav2_lifecycle_manager::LifecycleManager",
    ):
        assert plugin in launch


def test_velocity_smoother_is_configured_and_lifecycle_managed():
    launch = (ROOT / "src/go2w_nav/launch/nav2_3d.launch.py").read_text(
        encoding="utf-8"
    )
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )

    lifecycle_nodes = launch.split("'node_names': [", 1)[1].split("]", 1)[0]
    assert "'velocity_smoother'" in lifecycle_nodes
    assert "\nvelocity_smoother:\n" in params
    assert 'feedback: "OPEN_LOOP"' in params


def test_nav_controller_and_smoother_share_one_forward_only_envelope():
    """DWB must never sample commands that the motion boundary changes."""
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )

    assert "min_vel_x: 0.0" in params
    assert "max_vel_x: 0.8" in params
    assert "max_vel_theta: 0.5" in params
    assert "min_speed_theta: 0.15" in params
    assert "acc_lim_theta: 1.0" in params
    assert "decel_lim_theta: -1.5" in params
    assert "required_movement_radius: 0.1" in params
    assert "movement_time_allowance: 60.0" in params
    assert "max_velocity: [0.8, 0.0, 0.5]" in params
    assert "min_velocity: [0.0, 0.0, -0.5]" in params
    assert "max_accel: [1.2, 0.0, 1.0]" in params
    assert "max_decel: [-1.0, 0.0, -1.5]" in params
    behavior = params.split("behavior_server:", 1)[1].split(
        "waypoint_follower:", 1
    )[0]
    assert 'behavior_plugins: ["wait"]' in behavior
    for motion_behavior in ("spin:", "backup:", "drive_on_heading:"):
        assert motion_behavior not in behavior


def test_point_goal_is_position_only_and_costmap_display_contracts():
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )
    controller = params.split("controller_server:", 1)[1].split(
        "local_costmap:", 1
    )[0]

    assert "RotateToGoal" not in controller
    assert controller.count("xy_goal_tolerance: 0.20") == 2
    assert controller.count("yaw_goal_tolerance: 3.14") == 2
    assert "stateful: false" in controller
    for section in _costmap_sections(params):
        assert "always_send_full_costmap: true" in section
        assert re.search(r"^\s+observation_sources:\s*mid360\s*$", section, re.M)


def test_costmap_collision_envelope_covers_go2w_wheels_and_braking_margin():
    """The planning body must exceed the official 0.70 x 0.43 m envelope.

    Field collision evidence showed Navfn accepting 0.247 m center clearance.
    Add 0.10 m static clearance on every side and retain another 0.15 m of
    inflated gradient beyond the padded body's turning radius.
    """
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )

    local_section = _costmap_sections(params)[0]
    encoded = re.search(r'^\s+footprint:\s*(.+)$', local_section, re.M).group(1)
    footprint = ast.literal_eval(ast.literal_eval(encoded))
    xs = [float(point[0]) for point in footprint]
    ys = [float(point[1]) for point in footprint]
    turning_radius = max(math.hypot(x, y) for x, y in footprint)
    inflation = float(
        re.search(
            r'^\s+inflation_radius:\s*([0-9.]+)', local_section, re.M
        ).group(1)
    )

    assert min(xs) <= -0.45 and max(xs) >= 0.45
    assert min(ys) <= -0.315 and max(ys) >= 0.315
    assert inflation >= turning_radius + 0.15


def test_dwb_rejects_trajectories_using_the_full_rectangular_footprint():
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )
    controller = params.split("controller_server:", 1)[1].split(
        "local_costmap:", 1
    )[0]
    critics = re.search(r'^\s+critics:\s*\[(.+)\]$', controller, re.M)

    assert critics is not None
    critic_names = [item.strip().strip('"') for item in critics.group(1).split(",")]
    assert "ObstacleFootprint" in critic_names
    assert critic_names.index("ObstacleFootprint") < critic_names.index("PathAlign")
    assert "ObstacleFootprint.scale: 0.02" in controller


def test_global_planner_uses_orientation_independent_turning_envelope():
    """Navfn plans robot centers, so its body must cover every local yaw."""
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )
    global_section = _costmap_sections(params)[1]
    radius_match = re.search(
        r'^\s+robot_radius:\s*([0-9.]+)', global_section, re.M
    )

    assert not re.search(r'^\s+footprint:', global_section, re.M)
    assert radius_match is not None
    radius = float(radius_match.group(1))
    inflation = float(
        re.search(
            r'^\s+inflation_radius:\s*([0-9.]+)', global_section, re.M
        ).group(1)
    )
    local_section = _costmap_sections(params)[0]
    local_inflation = float(
        re.search(
            r'^\s+inflation_radius:\s*([0-9.]+)', local_section, re.M
        ).group(1)
    )
    # The global layer keeps the hard turning envelope; the local controller
    # owns the wider soft-clearance gradient used during motion.
    assert radius >= math.hypot(0.45, 0.32) + 0.04
    assert inflation >= radius
    assert local_inflation >= radius + 0.10


def test_dwb_samples_forward_only_low_speed_approaches():
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )

    assert "min_vel_x: 0.0" in params
    assert "min_speed_xy: 0.05" in params


def test_negative_twenty_degree_body_to_base_pitch_points_lidar_x_down():
    helpers = _load_fuser_pure_functions()
    build_static_tf = helpers["_build_static_tf"]
    E = build_static_tf(0.0, 0.0, 0.0, 0.0, -math.radians(20.0), 0.0)

    lidar_x_in_base = np.linalg.inv(E[:3, :3]) @ np.array([1.0, 0.0, 0.0])

    assert lidar_x_in_base[2] < 0.0


def test_conjugate_pose_recovers_expected_base_yaw():
    helpers = _load_fuser_pure_functions()
    build_static_tf = helpers["_build_static_tf"]
    conjugate_pose = helpers["_conjugate_pose"]
    E = build_static_tf(0.0, 0.0, 0.0, 0.0, -math.radians(20.0), 0.0)
    X_expected = build_static_tf(1.2, -0.4, 0.0, 0.0, 0.0, 0.7)
    T_camera_body = E @ X_expected @ np.linalg.inv(E)

    X_actual = conjugate_pose(T_camera_body, E)

    np.testing.assert_allclose(X_actual, X_expected, atol=1e-12)


def test_slam_map_correction_propagates_current_wheel_pose():
    helpers = _load_fuser_pure_functions()
    build_static_tf = helpers["_build_static_tf"]
    propagate = helpers["_propagate_map_pose"]
    observed_odom = build_static_tf(1.0, 0.0, 0.0, 0.0, 0.0, 0.1)
    observed_map = build_static_tf(5.0, 2.0, 0.0, 0.0, 0.0, 0.2)
    current_odom = build_static_tf(1.5, 0.3, 0.0, 0.0, 0.0, 0.3)

    correction, current_map = propagate(
        observed_map, observed_odom, current_odom
    )

    np.testing.assert_allclose(correction @ observed_odom, observed_map, atol=1e-12)
    np.testing.assert_allclose(current_map, correction @ current_odom, atol=1e-12)


def test_relative_planar_pose_anchors_arbitrary_fastlio_initial_attitude():
    helpers = _load_fuser_pure_functions()
    build_static_tf = helpers["_build_static_tf"]
    relative_planar_pose = helpers["_relative_planar_pose"]
    initial = build_static_tf(4.0, -3.0, 0.7, -0.16, -0.47, 1.13)
    motion = build_static_tf(1.2, -0.4, 0.3, 0.2, -0.1, 0.7)

    anchored = relative_planar_pose(initial, initial @ motion)
    expected = build_static_tf(1.2, -0.4, 0.0, 0.0, 0.0, 0.7)

    np.testing.assert_allclose(anchored, expected, atol=1e-12)


def test_fuser_creates_mid360_odom_and_localization_publishers():
    tree = ast.parse(FUSER.read_text(encoding="utf-8"))
    assert any(
        isinstance(node, ast.ImportFrom)
        and node.module == "nav_msgs.msg"
        and any(alias.name == "Odometry" for alias in node.names)
        for node in tree.body
    )
    init = _method(tree, "MapOdomFuser", "__init__")
    assignments = _assignment_map(init)

    assert assignments["self._pose_pub"].startswith("self.create_publisher(")
    assert "Odometry, '/localization_pose'" in assignments["self._pose_pub"]
    assert "Odometry, '/odom'" in assignments["self._odom_pub"]
    assert "Odometry, '/Odometry'" in assignments["self._lio_sub"]
    assert assignments["self._last_stamp_ns"] == "None"
    assert assignments["self._initial_leveled"] == "None"


def test_fastlio_stamp_directly_drives_leveled_tf_and_odometry():
    tree = ast.parse(FUSER.read_text(encoding="utf-8"))
    callback = _method(tree, "MapOdomFuser", "_on_lio")
    assignments = _assignment_map(callback)

    assert assignments["leveled"].startswith("_conjugate_pose(")
    assert assignments["planar"].startswith("_relative_planar_pose(")
    assert assignments["identity.header.frame_id"] == "self._world"
    assert assignments["identity.child_frame_id"] == "self._odom"
    assert assignments["base_tf.header.frame_id"] == "self._odom"
    assert assignments["base_tf.child_frame_id"] == "self._base"
    assert assignments["base_tf.header.stamp"] == "output_stamp"
    assert assignments["odom.header.stamp"] == "output_stamp"
    assert assignments["localization.header.stamp"] == "output_stamp"
    source = ast.unparse(callback)
    assert "50000000" not in source
    assert "self._broadcaster.sendTransform([identity, base_tf])" in source
    assert "self._odom_pub.publish(odom)" in source
    assert "self._pose_pub.publish(localization)" in source
    assert "lookup_transform" not in source


def test_bringup_passes_body_to_base_pitch_to_fuser():
    """bringup 必须把标定的倾斜角通过 ROS 参数传给 fuser.

    实机安装参数: body_to_base_pitch = -0.3490658504 rad (-20°).
    不传参时 fuser _T_body_base=I, 公式退化为无补偿 (倾斜传感器旋转直接灌进 map→odom).
    """
    script = (ROOT / "docker/bringup_slam_nav2.sh").read_text(encoding="utf-8")

    assert "BODY_TO_BASE_PITCH" in script
    assert "${BODY_TO_BASE_PITCH:--0.3490658504}" in script
    assert "body_to_base_pitch:=$BODY_TO_BASE_PITCH" in script
