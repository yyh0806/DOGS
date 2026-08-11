"""Production contracts for the NX global planner configuration."""

import ast
import math
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PARAMS = ROOT / "src/go2w_nav/config/nav2_params_3d.yaml"
PACKAGE_XML = ROOT / "src/go2w_nav/package.xml"
BRINGUP = ROOT / "docker/bringup_slam_nav2.sh"
WEB_SERVICE = ROOT / "docker/go2w-web.service"
SIMULATOR = ROOT / "tools/sim_strategy_compare.py"

FRONTIER_DEFAULTS = {
    "GO2W_FRONTIER_UTILITY_MODE": "mixed",
    "GO2W_FRONTIER_TIME_PENALTY": "14.5",
    "GO2W_FRONTIER_MIXED_WALL_BONUS": "1.0",
    "GO2W_FRONTIER_MIXED_EXPANSION_BONUS": "0.1",
    "GO2W_FRONTIER_PROBE_WORKERS": "4",
    "GO2W_FRONTIER_ANALYSIS_LIMIT": "24",
    "GO2W_FRONTIER_YAW_CANDIDATE_LIMIT": "12",
    "GO2W_FRONTIER_YAW_STEP_DEG": "45",
    "GO2W_FRONTIER_MAX_VEL_X": "0.8",
    "GO2W_FRONTIER_MAX_VEL_THETA": "0.5",
    # 搜索流畅性优化默认启用 (PR #4 D-yaw-optional + PR #5 D-prefetch).
    # D-yaw-optional: 大转向仅在 visual_gain 提升>=MIN_GAIN 才执行 (减到达后原地转).
    # D-prefetch: 导航期间后台预选下一个 frontier (隐藏选点耗时).
    "GO2W_FRONTIER_YAW_OPTIONAL": "1",
    "GO2W_FRONTIER_PREFETCH": "1",
}


def test_dynamic_replanning_uses_smac_2d_instead_of_navfn_path_extraction():
    """Avoid NavFn's intermittent legal-potential/empty-path failure on Humble."""
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    planner = params["planner_server"]["ros__parameters"]
    grid = planner["GridBased"]

    assert grid["plugin"] == "nav2_smac_planner/SmacPlanner2D"
    assert grid["allow_unknown"] is True
    assert grid["downsample_costmap"] is False
    assert grid["downsampling_factor"] == 1
    assert grid["max_iterations"] >= 1_000_000
    assert grid["max_on_approach_iterations"] >= 1_000
    assert grid["max_planning_time"] >= 1.0
    assert grid["cost_travel_multiplier"] >= 1.0
    # A tolerated endpoint can make ComputePath report success even when the
    # requested pose itself is inside an inscribed/lethal costmap cell.  Room
    # exploration needs the requested cell to be the reachable endpoint.
    assert grid["tolerance"] <= 0.05
    assert "use_astar" not in grid


def test_go2w_nav_declares_the_selected_smac_runtime_dependency():
    package_xml = PACKAGE_XML.read_text(encoding="utf-8")

    assert "<exec_depend>nav2_smac_planner</exec_depend>" in package_xml


def test_exploration_velocity_profile_is_aligned_end_to_end():
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]
    bringup = BRINGUP.read_text(encoding="utf-8")

    assert follow_path["min_vel_x"] == 0.0
    assert follow_path["max_vel_x"] == 0.8
    assert follow_path["max_speed_xy"] == 0.8
    assert follow_path["max_vel_theta"] == 0.5
    assert follow_path["acc_lim_x"] == 1.2
    assert follow_path["decel_lim_x"] == -1.0

    assert smoother["min_velocity"] == [0.0, 0.0, -0.5]
    assert smoother["max_velocity"] == [0.8, 0.0, 0.5]
    assert smoother["max_accel"] == [1.2, 0.0, 1.0]
    assert smoother["max_decel"] == [-1.0, 0.0, -1.5]
    assert smoother["min_velocity"] == [
        follow_path["min_vel_x"], follow_path["min_vel_y"],
        -follow_path["max_vel_theta"],
    ]
    assert smoother["max_velocity"] == [
        follow_path["max_vel_x"], follow_path["max_vel_y"],
        follow_path["max_vel_theta"],
    ]
    assert smoother["max_accel"] == [
        follow_path["acc_lim_x"], follow_path["acc_lim_y"],
        follow_path["acc_lim_theta"],
    ]
    assert smoother["max_decel"] == [
        follow_path["decel_lim_x"], follow_path["decel_lim_y"],
        follow_path["decel_lim_theta"],
    ]

    assert "export GO2W_FRONTIER_MAX_VEL_X=0.8" in bringup
    assert "export GO2W_FRONTIER_MAX_VEL_THETA=0.5" in bringup


def test_local_planner_predicts_beyond_stopping_distance_and_retains_obstacle_authority():
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]

    footprint = yaml.safe_load(local["footprint"])
    assert "ObstacleFootprint" in follow_path["critics"]
    assert local["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert local["obstacle_layer"]["enabled"] is True
    assert footprint == [
        [0.45, 0.32], [0.45, -0.32],
        [-0.45, -0.32], [-0.45, 0.32],
    ]
    assert local["inflation_layer"]["inflation_radius"] == 0.71

    max_velocity = float(follow_path["max_vel_x"])
    braking_deceleration = abs(float(follow_path["decel_lim_x"]))
    stopping_center_travel = max_velocity ** 2 / (2.0 * braking_deceleration)
    required_center_travel = stopping_center_travel + float(local["resolution"])
    predicted_center_travel = max_velocity * float(follow_path["sim_time"])

    # Compare robot-center travel to robot-center travel: DWB must collision-
    # check farther ahead than the 0.32 m braking distance plus one 0.05 m
    # costmap cell.  Inflation radius is obstacle-to-cell-center distance, not
    # a hard clearance beyond the footprint, so it is intentionally asserted
    # above but never misrepresented as additional braking travel.
    assert math.isclose(required_center_travel, 0.37, abs_tol=1e-9)
    assert predicted_center_travel >= required_center_travel


def test_frontier_time_penalty_records_the_gated_physical_profile_selection():
    bringup = BRINGUP.read_text(encoding="utf-8")

    assert "export GO2W_FRONTIER_TIME_PENALTY=14.5" in bringup
    assert "k_time=14.5" in bringup
    assert "0.8/0.5 m/s/rad/s" in bringup


def test_web_owner_and_nav_bringup_share_authoritative_frontier_defaults():
    bringup = BRINGUP.read_text(encoding="utf-8")
    service = WEB_SERVICE.read_text(encoding="utf-8")
    hardware_env_offset = service.index(
        "EnvironmentFile=-/etc/go2w/hardware.env")

    bringup_defaults = dict(
        line.strip()[len("export "):].split("=", 1)
        for line in bringup.splitlines()
        if line.strip().startswith("export GO2W_FRONTIER_"))
    service_defaults = dict(
        line.strip()[len("Environment="):].split("=", 1)
        for line in service.splitlines()
        if line.strip().startswith("Environment=GO2W_FRONTIER_"))

    assert bringup_defaults == FRONTIER_DEFAULTS
    assert service_defaults == FRONTIER_DEFAULTS

    for key, value in FRONTIER_DEFAULTS.items():
        service_default = f"Environment={key}={value}"
        assert service.index(service_default) < hardware_env_offset


def test_simulator_exit_gate_requires_every_hypothesis_h1_through_h9():
    tree = ast.parse(SIMULATOR.read_text(encoding="utf-8"))
    main = next(
        node for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "main")
    exit_return = next(
        node for node in reversed(main.body)
        if isinstance(node, ast.Return))
    assert isinstance(exit_return.value, ast.IfExp)
    assert isinstance(exit_return.value.test, ast.BoolOp)
    gated_names = {
        node.id for node in exit_return.value.test.values
        if isinstance(node, ast.Name)
    }

    assert gated_names == {f"h{index}" for index in range(1, 10)}


def test_simulator_bfs_reports_cached_planned_path_and_heading_turn():
    from tools.sim_strategy_compare import _KnownFreePlanner

    # The direct two-metre chord is blocked.  BFS must go south, east twice,
    # then north: four metres and four quarter-turns including start/end yaw.
    planner = _KnownFreePlanner(
        observed=[0, 100, 0, 0, 0, 0],
        width=3,
        height=2,
        resolution=1.0,
        pose=[0.5, 0.5, 0.0],
    )
    result = planner.compute_path_to_pose(2.5, 0.5, 0.0)
    cached = planner.planned_metrics_for_pose(2.5, 0.5, 0.0)

    assert result["path_length"] == 4.0
    assert result["path_heading_turn_rad"] == pytest.approx(2.0 * math.pi)
    assert cached == {
        "path_length": 4.0,
        "path_heading_turn_rad": pytest.approx(2.0 * math.pi),
    }


def test_simulator_deployment_constants_match_nav_and_service_profile():
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    tree = ast.parse(SIMULATOR.read_text(encoding="utf-8"))
    constants = {
        target.id: node.value.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance((target := node.targets[0]), ast.Name)
        and isinstance(node.value, ast.Constant)
    }

    assert float(constants["DEPLOY_MAX_VEL_X"]) == float(
        FRONTIER_DEFAULTS["GO2W_FRONTIER_MAX_VEL_X"])
    assert float(constants["DEPLOY_MAX_VEL_THETA"]) == float(
        FRONTIER_DEFAULTS["GO2W_FRONTIER_MAX_VEL_THETA"])
    assert float(constants["DEPLOY_K_TIME"]) == float(
        FRONTIER_DEFAULTS["GO2W_FRONTIER_TIME_PENALTY"])
    assert float(constants["DEPLOY_MAX_VEL_X"]) == follow_path["max_vel_x"]
    assert float(constants["DEPLOY_MAX_VEL_THETA"]) == follow_path["max_vel_theta"]


def test_hardware_env_comment_does_not_promise_persistent_frontier_tuning():
    service = WEB_SERVICE.read_text(encoding="utf-8")

    assert "hardware.env is rebuilt by deploy_release.sh" in service
    assert "Persistent frontier tuning belongs in this managed service" in service
    assert "commissioning may override any of them" not in service
