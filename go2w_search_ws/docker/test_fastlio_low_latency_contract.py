"""Contracts for the fail-closed, latest-frame FAST_LIO deployment path."""

from __future__ import annotations

import ast
import os
from pathlib import Path
import shutil
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / "docker" / "patches" / "fast_lio_latest_frame.patch"
LIVOX_RELIABLE_PATCH = (
    ROOT / "docker" / "patches" / "fast_lio_livox_reliable_qos.patch"
)
BODY_PATCH = ROOT / "docker" / "patches" / "fast_lio_body_cloud.patch"
BODY_QOS_PATCH = (
    ROOT / "docker" / "patches" / "fast_lio_body_cloud_qos.patch"
)
BOUNDED_BODY_PATCH = (
    ROOT / "docker" / "patches" / "fast_lio_bounded_body_cloud.patch"
)
ANGULAR_BODY_PATCH = (
    ROOT / "docker" / "patches" / "fast_lio_angular_body_cloud.patch"
)
PREPARE = ROOT / "docker" / "prepare_fastlio_low_latency.sh"
BRINGUP = ROOT / "docker" / "bringup_slam_nav2.sh"
CONFIG = (
    ROOT / "src" / "go2w_nav" / "config" / "fastlio_low_latency"
    / "mid360.yaml"
)
WATCHDOG = ROOT / "tools" / "livox_stream_watchdog.py"
FUSER = ROOT / "src" / "go2w_bridge" / "go2w_bridge" / "map_odom_fuser.py"
LATENCY_GATE = ROOT / "tools" / "fastlio_latency_gate.py"
TOPIC_RATE_GATE = ROOT / "tools" / "topic_rate_gate.py"


STOCK_SNIPPET = """\

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \\
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        }
        else
        {

        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);

            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
"""


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _bash_executable():
    candidates = []
    git = shutil.which("git")
    if git:
        candidates.append(Path(git).resolve().parents[1] / "bin" / "bash.exe")
    candidates.extend([
        Path(r"C:\Program Files\Git\bin\bash.exe"),
        Path(r"C:\Program Files (x86)\Git\bin\bash.exe"),
    ])
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    bash = shutil.which("bash")
    if bash and "WindowsApps" not in bash:
        return bash
    return None


def _function_source(path: Path, name: str) -> str:
    source = _source(path)
    tree = ast.parse(source)
    node = next(
        item for item in ast.walk(tree)
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
        and item.name == name
    )
    return ast.get_source_segment(source, node) or ""


def test_fastlio_patch_is_latest_frame_best_effort_and_buffer_safe():
    source = _source(PATCH)
    assert "SensorDataQoS" in source
    assert "keep_last(1)" in source
    assert "best_effort()" in source
    assert "if (!lidar_pushed)" in source
    assert "lidar_buffer.clear()" in source
    assert "time_buffer.clear()" in source
    assert source.index("lidar_buffer.clear()") < source.index(
        "lidar_buffer.push_back(ptr)")
    assert source.index("time_buffer.clear()") < source.index(
        "time_buffer.push_back(last_timestamp_lidar)")
    reliable = _source(LIVOX_RELIABLE_PATCH)
    assert "latest_livox_qos.best_effort()" in reliable
    assert "latest_livox_qos.reliable()" in reliable


def test_fastlio_body_cloud_is_independent_from_expensive_world_cloud():
    source = _source(BODY_PATCH)
    assert "if (scan_body_pub_en) publish_frame_body" in source
    assert "scan_pub_en && scan_body_pub_en" in source

    qos = _source(BODY_QOS_PATCH)
    assert "latest_body_cloud_qos.keep_last(1)" in qos
    assert "latest_body_cloud_qos.best_effort()" in qos
    assert '"/cloud_registered_body", latest_body_cloud_qos' in qos

    bounded = _source(BOUNDED_BODY_PATCH)
    assert "constexpr int max_body_cloud_points = 1000" in bounded
    assert "body_cloud_phase" not in bounded
    assert "source_index = 0; source_index < source_size" in bounded

    angular = _source(ANGULAR_BODY_PATCH)
    assert "angular_stratified_body_cloud" in angular
    assert "azimuth_bins = 360" in angular
    assert "elevation_bins = 8" in angular
    assert "azimuth_bins * elevation_bins" in angular
    assert "selected_range_sq" in angular
    assert "source_index += stride" in bounded
    assert "feats_undistort->points[source_index]" in bounded


def test_prepare_helper_applies_once_and_rejects_unknown_preimage(tmp_path):
    bash = _bash_executable()
    if bash is None:
        pytest.skip("bash is required for the deployment-helper contract")

    workspace = tmp_path / "ws_livox"
    source_dir = workspace / "src" / "FAST_LIO_ROS2" / "src"
    source_dir.mkdir(parents=True)
    laser_mapping = source_dir / "laserMapping.cpp"
    laser_mapping.write_text(STOCK_SNIPPET, encoding="utf-8")

    env = {
        **os.environ,
        "FASTLIO_WS": str(workspace),
        "FASTLIO_SOURCE_ONLY": "1",
    }
    first = subprocess.run(
        [bash, PREPARE.as_posix()], cwd=ROOT, env=env,
        text=True, encoding="utf-8", errors="replace",
        capture_output=True, check=False,
    )
    assert first.returncode == 0, first.stderr
    patched = laser_mapping.read_text(encoding="utf-8")
    assert "keep_last(1)" in patched
    assert "latest_livox_qos.reliable()" in patched
    assert "latest_livox_qos.best_effort()" not in patched
    assert "if (scan_body_pub_en) publish_frame_body" in patched
    assert "latest_body_cloud_qos.best_effort()" in patched
    assert "constexpr int max_body_cloud_points = 1000" in patched
    assert "body_cloud_phase" not in patched
    assert "angular_stratified_body_cloud" not in patched

    second = subprocess.run(
        [bash, PREPARE.as_posix()], cwd=ROOT, env=env,
        text=True, encoding="utf-8", errors="replace",
        capture_output=True, check=False,
    )
    assert second.returncode == 0, second.stderr
    assert laser_mapping.read_text(encoding="utf-8") == patched

    # Releases share the external FAST_LIO workspace. Verify the helper can
    # migrate the experimentally rejected 6000-point revision.
    fixed = patched.replace(
        "constexpr int max_body_cloud_points = 1000;",
        "constexpr int max_body_cloud_points = 6000;",
    )
    laser_mapping.write_text(fixed, encoding="utf-8")
    migrated = subprocess.run(
        [bash, PREPARE.as_posix()], cwd=ROOT, env=env,
        text=True, encoding="utf-8", errors="replace",
        capture_output=True, check=False,
    )
    assert migrated.returncode == 0, migrated.stderr
    migrated_source = laser_mapping.read_text(encoding="utf-8")
    assert "constexpr int max_body_cloud_points = 1000" in migrated_source
    assert "body_cloud_phase" not in migrated_source

    laser_mapping.write_text("// unknown source revision\n", encoding="utf-8")
    unknown = subprocess.run(
        [bash, PREPARE.as_posix()], cwd=ROOT, env=env,
        text=True, encoding="utf-8", errors="replace",
        capture_output=True, check=False,
    )
    assert unknown.returncode != 0
    assert "unknown" in (unknown.stderr + unknown.stdout).lower()


def test_prepare_helper_sources_ros_without_leaking_nounset():
    source = _source(PREPARE)
    disable = source.index("set +u", source.index("/opt/ros/humble/setup.bash"))
    ros_setup = source.index("source /opt/ros/humble/setup.bash", disable)
    restore = source.index("set -u", ros_setup)
    build = source.index("colcon build", restore)
    assert disable < ros_setup < restore < build


def test_release_fastlio_config_keeps_pose_but_disables_expensive_outputs():
    data = yaml.safe_load(_source(CONFIG))["/**"]["ros__parameters"]
    assert data["common"]["lid_topic"] == "/livox/lidar"
    assert data["common"]["imu_topic"] == "/livox/imu"
    assert data["mapping"]["extrinsic_est_en"] is False
    assert data["publish"]["path_en"] is False
    assert data["publish"]["scan_publish_en"] is False
    assert data["publish"]["dense_publish_en"] is False
    assert data["publish"]["scan_bodyframe_pub_en"] is True
    assert data["publish"].get("map_en", False) is False
    assert data["pcd_save"]["pcd_save_en"] is False


def test_bringup_prepares_and_gates_release_owned_fastlio_before_fuser():
    source = _source(BRINGUP)
    assert 'FASTLIO_CONFIG="${FASTLIO_CONFIG:-$RUNTIME_ROOT/' in source
    assert "fastlio_low_latency" in source
    prepare = source.index("prepare_fastlio_low_latency.sh")
    start = source.index("ros2 launch fast_lio")
    latency = source.index("health_gate 25 --stamp-age /Odometry 0.35")
    fuser = source.index("map_odom_fuser.py --ros-args")
    assert prepare < start < latency < fuser
    assert "src/FAST_LIO_ROS2/config}" not in source


def test_bringup_uses_one_persistent_participant_for_topic_rate_gates():
    source = _source(BRINGUP)
    marker = source.index("# ---- single-participant health gates ----")
    runtime = source[marker:]
    assert "nav_health_supervisor.py" in runtime
    assert 'health_gate "$timeout_s" --rate "$topic" "$min"' in runtime
    assert 'timeout "$sample_s" ros2 topic hz' not in source
    supervisor = _source(ROOT / "tools" / "nav_health_supervisor.py")
    assert "samples = {topic: deque(maxlen=40)" in supervisor
    assert "ReliabilityPolicy.RELIABLE" in supervisor


def test_watchdog_uses_small_imu_sensor_qos_not_full_custom_cloud():
    source = _source(WATCHDOG)
    assert "from sensor_msgs.msg import Imu" in source
    assert "qos_profile_sensor_data" in source
    assert 'Imu, "/livox/imu"' in source
    assert "CustomMsg" not in source
    assert "/livox/lidar" not in source


def test_fuser_declares_and_checks_raw_lio_age_before_state_mutation():
    source = _source(FUSER)
    assert 'declare_parameter("max_lio_age_sec", 0.35)' in source
    assert "_lio_message_age_is_fresh" in source
    callback = _function_source(FUSER, "_on_lio")
    age_check = callback.index("_lio_message_age_is_fresh")
    mutation = callback.index("self._last_stamp_ns = stamp_ns")
    publish = callback.index("self._odom_pub.publish(odom)")
    assert age_check < mutation < publish


def test_fuser_consumes_only_latest_raw_lio_pose_without_depth_backlog():
    source = _source(FUSER)
    assert "from rclpy.qos import QoSProfile, ReliabilityPolicy" in source
    assert "latest_lio_qos = QoSProfile(" in source
    assert "depth=1" in source
    assert "reliability=ReliabilityPolicy.RELIABLE" in source
    assert "create_subscription(Odometry, '/Odometry'" in source
    assert "self._on_lio" in source
    assert "latest_lio_qos)" in source


def test_fuser_broadcasts_only_latest_dynamic_tf_without_depth_100_backlog():
    source = _source(FUSER)
    assert "latest_tf_qos = QoSProfile(" in source
    assert "depth=1" in source
    assert "reliability=ReliabilityPolicy.RELIABLE" in source
    assert "TransformBroadcaster(self, qos=latest_tf_qos)" in source


def test_runtime_artifact_contract_mentions_new_low_latency_files():
    verifier = _source(ROOT / "tools" / "verify_release_artifact.py")
    build = _source(ROOT / "docker" / "build_release.sh")
    for relative in (
        "docker/patches/fast_lio_latest_frame.patch",
        "docker/patches/fast_lio_livox_reliable_qos.patch",
        "docker/patches/fast_lio_body_cloud.patch",
        "docker/patches/fast_lio_body_cloud_qos.patch",
        "docker/patches/fast_lio_bounded_body_cloud.patch",
        "docker/patches/fast_lio_rotating_body_sample.patch",
        "docker/patches/fast_lio_angular_body_cloud.patch",
        "docker/prepare_fastlio_low_latency.sh",
        "tools/fastlio_latency_gate.py",
        "tools/topic_rate_gate.py",
        "tools/nav_health_supervisor.py",
        "tools/nav_health_gate.py",
        "src/go2w_nav/config/fastlio_low_latency/mid360.yaml",
    ):
        assert relative in verifier
        assert relative in build or "tar" in build
