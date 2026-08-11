"""Contracts for the NX-connected MID360 + C13-only navigation chain."""

import ast
import math
from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "src/go2w_bridge/go2w_bridge"
POINT_BRIDGE = BRIDGE / "mid360_nav_bridge.py"
CPP_POINT_BRIDGE = ROOT / "src/go2w_nav/src/mid360_nav_bridge.cpp"
NAV_CMAKE = ROOT / "src/go2w_nav/CMakeLists.txt"
NAV_PACKAGE = ROOT / "src/go2w_nav/package.xml"
FUSER = BRIDGE / "map_odom_fuser.py"
MOTION = BRIDGE / "nx_motion_node.py"
MOTION_SAFETY = BRIDGE / "motion_safety.py"
PARAMS = ROOT / "src/go2w_nav/config/nav2_params_3d.yaml"
SAFE_REPLAN_BT = (
    ROOT / "src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml"
)
BRINGUP = ROOT / "docker/bringup_slam_nav2.sh"
SERVICE = ROOT / "docker/go2w-slam-nav.service"
WEB = ROOT / "web/nx_web_server.py"
AI = ROOT / "web/nx_ai_node.py"


def _load_functions(path, names):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    nodes = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {"math": math, "np": np}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), "exec"), namespace)
    return namespace


def test_mid360_points_are_leveled_by_the_measured_twenty_degree_mount():
    helpers = _load_functions(POINT_BRIDGE, {"_rpy_matrix", "_transform_points"})
    rotation = helpers["_rpy_matrix"](0.0, math.radians(20.0), 0.0)

    # A point on the tilted sensor's +X axis points 20 degrees downward in
    # base_link.  Applying base<-body (+20 deg) restores a horizontal ray.
    p_body = np.array([[math.cos(math.radians(20.0)), 0.0,
                        math.sin(math.radians(20.0))]], dtype=np.float32)
    actual = helpers["_transform_points"](p_body, rotation, np.zeros(3))
    assert actual[0] == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)


def test_mid360_fastlio_metres_remain_nav2_metres():
    helpers = _load_functions(POINT_BRIDGE, {"_scale_points"})

    # Livox CustomMsg and FAST_LIO both use metres.  Keep the conversion
    # explicit/configurable, but never silently compensate for a diverged
    # estimator by shrinking its physically invalid output.
    raw_metres = np.array([
        [1.0, 0.0, 0.25],
        [2.5, -0.5, -0.55],
    ], dtype=np.float32)
    actual = helpers["_scale_points"](raw_metres, 1.0)

    np.testing.assert_allclose(actual, [
        [1.0, 0.0, 0.25],
        [2.5, -0.5, -0.55],
    ], atol=1e-7)
    source = POINT_BRIDGE.read_text(encoding="utf-8")
    assert 'declare_parameter("input_xyz_scale", 1.0)' in source
    assert "_scale_points(points, self._input_xyz_scale)" in source


def test_mid360_raw_livox_points_decode_directly_in_metres():
    helpers = _load_functions(POINT_BRIDGE, {"_decode_livox_xyz"})
    msg = SimpleNamespace(
        point_num=3,
        points=[
            SimpleNamespace(x=1.0, y=2.0, z=3.0),
            SimpleNamespace(x=-0.5, y=0.25, z=4.0),
            SimpleNamespace(x=8.0, y=-1.0, z=0.125),
        ],
    )

    actual = helpers["_decode_livox_xyz"](msg)

    np.testing.assert_allclose(actual, [
        [1.0, 2.0, 3.0],
        [-0.5, 0.25, 4.0],
        [8.0, -1.0, 0.125],
    ])


def test_mid360_raw_decode_is_rate_bounded_and_latest_only():
    helper = _load_functions(POINT_BRIDGE, {"_advance_sample_deadline"})[
        "_advance_sample_deadline"
    ]
    assert helper(10.0, 0.0, 0.2) == (True, pytest.approx(10.2))
    assert helper(10.1, 10.2, 0.2) == (False, pytest.approx(10.2))
    assert helper(10.2, 10.2, 0.2) == (True, pytest.approx(10.4))

    source = POINT_BRIDGE.read_text(encoding="utf-8")
    assert 'declare_parameter("input_hz", 5.0)' in source
    assert "depth=1" in source
    assert "reliability=ReliabilityPolicy.BEST_EFFORT" in source
    assert "self._pending = [filtered]" in source


def test_mid360_nav_obstacles_default_to_raw_livox_not_fastlio_output():
    source = POINT_BRIDGE.read_text(encoding="utf-8")

    assert 'declare_parameter("input_source", "livox_custom")' in source
    assert 'LivoxCustomMsg, "/livox/lidar"' in source
    assert 'PointCloud2, "/cloud_registered_body"' in source
    assert "self._on_livox" in source
    assert "self._on_fastlio_cloud" in source


def test_mid360_scan_uses_nearest_return_and_infinity_for_clear_bins():
    helpers = _load_functions(POINT_BRIDGE, {"_points_to_scan"})
    points = np.array([
        [2.0, 0.0, 0.3],
        [1.0, 0.0, 0.4],
        [0.0, 1.0, 0.2],
    ], dtype=np.float32)
    ranges = helpers["_points_to_scan"](points, bins=360, range_min=0.2,
                                         range_max=8.0)
    assert ranges[180] == pytest.approx(1.0)
    assert ranges[270] == pytest.approx(1.0)
    assert math.isinf(ranges[0]) and ranges[0] > 0.0


def test_mid360_scan_memory_fills_sparse_bins_with_recent_nearest_returns():
    helpers = _load_functions(POINT_BRIDGE, {"_merge_scan_ranges"})
    merged = helpers["_merge_scan_ranges"]([
        [math.inf, 2.0, math.inf],
        [1.5, math.inf, 3.0],
    ])
    assert merged == pytest.approx([1.5, 2.0, 3.0])
    source = POINT_BRIDGE.read_text(encoding="utf-8")
    assert 'declare_parameter("scan_memory_sec", 0.5)' in source
    assert "self._scan_history" in source


def test_mid360_height_filter_rejects_floor_but_keeps_low_obstacles():
    helpers = _load_functions(POINT_BRIDGE, {"_filter_nav_points"})
    points = np.array([
        [1.0, 0.0, -0.59],  # field floor band after mount leveling
        [1.1, -0.1, -0.52], # upper edge of the measured floor band
        [1.2, 0.1, -0.40],  # obstacle about 20 cm above the floor
        [2.0, -0.2, 0.30],
    ], dtype=np.float32)

    actual = helpers["_filter_nav_points"](
        points,
        range_min=0.20,
        range_max=8.0,
        min_height=-0.45,
        max_height=1.50,
    )

    assert actual == pytest.approx(points[2:])
    assert 'declare_parameter("min_height", -0.45)' in POINT_BRIDGE.read_text(
        encoding="utf-8"
    )


def test_mid360_self_filter_covers_the_padded_nav2_footprint():
    helpers = _load_functions(POINT_BRIDGE, {"_filter_nav_points"})
    points = np.array([
        [0.44, 0.31, 0.10],    # front-left chassis/wheel envelope
        [-0.44, -0.31, 0.10],  # rear-right chassis/wheel envelope
        [0.46, 0.33, 0.10],    # immediately outside the padded footprint
    ], dtype=np.float32)

    actual = helpers["_filter_nav_points"](
        points,
        range_min=0.20,
        range_max=8.0,
        min_height=-0.45,
        max_height=1.50,
    )

    assert actual == pytest.approx(points[2:])


def test_mid360_bridge_does_not_starve_the_10hz_publish_timer():
    source = POINT_BRIDGE.read_text(encoding="utf-8")
    assert "MultiThreadedExecutor(num_threads=2)" in source
    assert "depth=1" in source
    assert "ReliabilityPolicy.BEST_EFFORT" in source
    assert "ExternalShutdownException" in source
    assert "self._publish_timer = self.create_timer(" in source


def test_active_mid360_bridge_uses_compact_fastlio_body_cloud():
    source = CPP_POINT_BRIDGE.read_text(encoding="utf-8")
    cmake = NAV_CMAKE.read_text(encoding="utf-8")
    package = NAV_PACKAGE.read_text(encoding="utf-8")
    bringup = BRINGUP.read_text(encoding="utf-8")

    assert "sensor_msgs::msg::PointCloud2" in source
    assert '"/cloud_registered_body"' in source
    assert "livox_ros_driver2::msg::CustomMsg" not in source
    assert "rclcpp::SensorDataQoS" in source
    assert "keep_last(1)" in source and "best_effort()" in source
    assert 'frame_id = "base_link"' in source
    assert 'create_wall_timer' in source
    assert "mid360_nav_bridge_cpp" in cmake
    for dependency in ("rclcpp", "sensor_msgs"):
        assert f"find_package({dependency} REQUIRED)" in cmake
        assert f"<depend>{dependency}</depend>" in package
    assert "find_package(livox_ros_driver2 REQUIRED)" not in cmake
    active = bringup.split("start_transient mid360-nav-bridge", 1)[1].split(
        "wait_hz /mid360/points_nav", 1
    )[0]
    assert "mid360_nav_bridge_cpp" in active
    assert "mid360_nav_bridge.py" not in active

    deploy = (ROOT / "docker/deploy_release.sh").read_text(encoding="utf-8")
    source_ros = deploy.split("source_ros() {", 1)[1].split("\n}", 1)[0]
    assert (
        "source /home/nx/ws_livox/install/livox_ros_driver2/share/"
        "livox_ros_driver2/local_setup.bash"
    ) in source_ros
    assert "source /home/nx/ws_livox/install/setup.bash" not in source_ros


def test_fastlio_backbone_owns_live_pose_and_wheel_callback_cannot_publish_pose():
    source = FUSER.read_text(encoding="utf-8")
    assert "create_subscription(Odometry, '/Odometry'" in source
    assert "create_subscription(Odometry, '/wheel_odom'" in source
    assert "create_publisher(Odometry, '/odom'" in source
    assert "child_frame_id = self._base" in source
    assert "lookup_transform(self._odom, self._base" not in source
    assert "_compute_map_odom_correction" not in source

    tree = ast.parse(source)
    fuser_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "MapOdomFuser"
    )
    methods = {
        node.name: node for node in fuser_class.body
        if isinstance(node, ast.FunctionDef)
    }
    on_lio = ast.unparse(methods["_on_lio"])
    on_wheel = ast.unparse(methods["_on_wheel"])
    pose_ready = on_lio.index("self._latest_lio_planar = planar")
    pose_publish = on_lio.index("self._odom_pub.publish(odom)", pose_ready)
    assert "return" not in on_lio[pose_ready:pose_publish]
    assert "self._latest_odom_planar = planar.copy()" in on_lio
    assert "self._broadcaster.sendTransform" in on_lio
    assert "self._odom_pub.publish(odom)" in on_lio
    assert "self._broadcaster.sendTransform" not in on_wheel
    assert "self._odom_pub.publish" not in on_wheel
    assert "self._pose_pub.publish" not in on_wheel
    assert "_bounded_fuse_planar" not in on_wheel

    fuser = FUSER.read_text(encoding="utf-8")
    sensor = (BRIDGE / "nx_sensor_node.py").read_text(encoding="utf-8")
    bringup = BRINGUP.read_text(encoding="utf-8")
    sensor_service = (ROOT / "docker/go2w-sensor.service").read_text(
        encoding="utf-8"
    )
    assert "'/wheel_odom'" in fuser
    assert "publish_odom_tf:=false" in sensor_service
    assert "publish_scan:=false" in sensor_service
    assert 'WHEEL_ODOM_MIN_HZ="${WHEEL_ODOM_MIN_HZ:-10}"' in bringup
    assert 'wait_hz /wheel_odom "$WHEEL_ODOM_MIN_HZ" 30' in bringup
    assert "odom_topic" in sensor and "publish_odom_tf" in sensor


def test_active_navigation_chain_uses_bounded_builtin_sensor_feedback():
    bringup = BRINGUP.read_text(encoding="utf-8")
    service = SERVICE.read_text(encoding="utf-8")
    sensor_service = (ROOT / "docker/go2w-sensor.service").read_text(
        encoding="utf-8"
    )
    params = PARAMS.read_text(encoding="utf-8")

    assert "for svc in livox-mid360-driver go2w-motion go2w-web go2w-sensor" in bringup
    assert "systemctl stop go2w-sensor.service" not in bringup
    assert "start_transient wheel-odom" not in bringup
    assert "publish_odom_tf:=false" in sensor_service
    assert "odom_topic:=/wheel_odom" in sensor_service
    assert "Conflicts=go2w-sensor.service" not in service
    assert "Wants=go2w-sensor.service" in service
    assert "After=go2w-sensor.service" in service
    assert "wait_hz /mid360/points_nav" in bringup
    assert "wait_hz /scan_mid360" not in bringup
    assert "wait_motion_ready 200" in bringup
    assert "topic: /mid360/points_nav" not in params
    assert params.count("topic: /scan_mid360") == 2
    assert params.count('data_type: "LaserScan"') == 2
    assert 'data_type: "PointCloud2"' not in params
    assert params.count("inf_is_valid: True") == 2
    assert "topic: /scan\n" not in params


def test_point_navigation_planning_failure_cannot_trigger_motion_recovery():
    params = PARAMS.read_text(encoding="utf-8")
    tree = SAFE_REPLAN_BT.read_text(encoding="utf-8")

    assert "go2w_safe_dynamic_replan" in params
    assert '<RateController hz="2.0">' in tree
    assert "<ComputePathToPose" in tree
    assert "<FollowPath" in tree
    assert '<RecoveryNode number_of_retries="1"' in tree
    assert "global_costmap/clear_entirely_global_costmap" in tree
    assert '<Wait wait_duration="1"' in tree
    for unsafe_motion_recovery in ("<Spin", "<BackUp", "<DriveOnHeading"):
        assert unsafe_motion_recovery not in tree
    assert "min_vel_x: 0.0" in params
    assert "max_vel_x: 0.8" in params
    assert "max_vel_theta: 0.5" in params
    assert "max_velocity: [0.8, 0.0, 0.5]" in params
    assert "min_velocity: [0.0, 0.0, -0.5]" in params
    behavior = params.split("behavior_server:", 1)[1].split(
        "waypoint_follower:", 1
    )[0]
    assert 'behavior_plugins: ["wait"]' in behavior


def test_motion_readiness_prefers_local_status_and_bounds_dds_fallback():
    bringup = BRINGUP.read_text(encoding="utf-8")
    helper = bringup.split("wait_motion_ready() {", 1)[1].split("\n}\n", 1)[0]
    assert "http://127.0.0.1:8000/api/status" in helper
    assert helper.index("/api/status") < helper.index("ros2 topic echo")
    assert "(( query_s > 6 )) && query_s=6" in helper
    assert "ros2 topic echo --no-daemon /dog_state --once --field data" in helper
    assert "ExecStopPost=-+" in SERVICE.read_text(encoding="utf-8")


def test_motion_readiness_requires_canonical_safe_park_state():
    bringup = BRINGUP.read_text(encoding="utf-8")
    helper = bringup.split("wait_motion_ready() {", 1)[1].split(
        "\n}\n", 1)[0]

    for required in (
        '"session":"parked"',
        '"drive_session":"parked"',
        '"physical_mode":"joint_lock"',
        '"actual_motion":"stopped"',
        '"velocity_authorized":false',
        '"fault":null',
        '"drive_fault":null',
    ):
        assert required in helper
    for legacy in ("EMERGENCY", "STOOD", "STAND_UNCONFIRMED",
                   "BALANCE_UNCONFIRMED"):
        assert legacy not in helper


def test_bringup_requires_fastlio_pose_and_raw_livox_obstacles():
    main = BRINGUP.read_text(encoding="utf-8").split("main() {", 1)[1]
    assert "wait_hz /livox/lidar" not in main
    assert "wait_hz /livox/imu" not in main
    assert 'FASTLIO_ENABLE="${FASTLIO_ENABLE:-1}"' in BRINGUP.read_text(
        encoding="utf-8"
    )
    assert 'FASTLIO_ENABLE=1 is required for physical-pose navigation' in main
    assert 'wait_hz /Odometry "$ODOM_MIN_HZ" 60' in main
    assert "wheel/IMU localization backbone" not in main
    assert 'grep -q "^/Odometry$"' not in main
    assert 'start_transient fastlio' in main


def test_bringup_fails_closed_when_base_link_has_two_parents():
    bringup = BRINGUP.read_text(encoding="utf-8")
    helper = bringup.split("check_tf_topology() {", 1)[1].split(
        "\n}\n", 1)[0]

    assert 'die "base_link 双 parent:' in helper
    assert "check_tf_topology || warn" not in bringup
    assert "根治需 map_odom_fuser" not in bringup
    assert "当前定点移动可能失败" not in bringup


def test_nav_outputs_use_wall_time_and_measured_jitter_bounds():
    bridge = POINT_BRIDGE.read_text(encoding="utf-8")
    fuser = FUSER.read_text(encoding="utf-8")
    params = PARAMS.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")

    assert "output_stamp = self.get_clock().now().to_msg()" in bridge
    assert "cloud.header.stamp = output_stamp" in bridge
    assert bridge.index("cloud.data =") < bridge.index("output_stamp =")
    assert bridge.index("output_stamp =") < bridge.index("self._cloud_pub.publish(cloud)")
    assert "scan.header.stamp = self.get_clock().now().to_msg()" in bridge
    # The fuser intentionally samples the clock once: the same ``now`` is
    # used both for the raw FAST_LIO age gate and the wall-time output stamp.
    assert "now = self.get_clock().now()" in fuser
    assert "output_stamp = now.to_msg()" in fuser
    assert "base_tf.header.stamp = output_stamp" in fuser
    assert "odom.header.stamp = output_stamp" in fuser
    assert "origin_z:" not in params
    assert params.count("expected_update_rate: 1.8") == 2
    assert '_parameter_value(self, "nav_scan_timeout", 1.8)' in motion
    assert '"$unit" = "fastlio"' in BRINGUP.read_text(encoding="utf-8")
    assert '-p "Nice=-5"' in BRINGUP.read_text(encoding="utf-8")


def test_fuser_rejects_impossible_fastlio_pose_jumps():
    helper = _load_functions(FUSER, {"_lio_pose_is_plausible"})[
        "_lio_pose_is_plausible"
    ]
    origin = np.array([0.0, 0.0, 0.0])

    assert helper(None, None, origin, 1_000_000_000, 3.0, 0.5, 10_000.0)
    assert helper(origin, 1_000_000_000,
                  np.array([0.2, 0.0, 0.0]), 1_100_000_000,
                  3.0, 0.5, 10_000.0)
    assert not helper(origin, 1_000_000_000,
                      np.array([5.0, 0.0, 0.0]), 1_100_000_000,
                      3.0, 0.5, 10_000.0)
    assert not helper(None, None, np.array([20_000.0, 0.0, 0.0]),
                      1_000_000_000, 3.0, 0.5, 10_000.0)
    assert not helper(origin, 1_000_000_000,
                      np.array([float("nan"), 0.0, 0.0]), 1_100_000_000,
                      3.0, 0.5, 10_000.0)


def test_motion_and_panel_safety_consume_only_mid360_scan():
    motion = MOTION.read_text(encoding="utf-8")
    safety = MOTION_SAFETY.read_text(encoding="utf-8")
    web = WEB.read_text(encoding="utf-8")

    assert 'msg.header.frame_id != "base_link"' in safety
    assert 'LaserScan, "/scan_mid360"' in motion
    assert "LaserScan, '/scan_mid360'" in web
    assert "LaserScan, '/scan'" not in motion
    assert "LaserScan, '/scan'" not in web


def test_c13_is_the_default_and_dog_camera_is_opt_in_disabled():
    ai = AI.read_text(encoding="utf-8")
    assert 'os.environ.get("GO2W_AI_VIDEO_ENABLE", "0")' in ai
    assert 'source_kinds = ["external"]' in ai
    assert 'submit_external_frame(c13_vis_frame, source="c13_vis")' in WEB.read_text(
        encoding="utf-8"
    )
