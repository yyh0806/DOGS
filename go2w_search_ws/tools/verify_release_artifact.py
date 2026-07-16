#!/usr/bin/env python3
"""Strict, read-only validation for a Go2W NX release archive."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import tarfile
from typing import Mapping


_RELEASE_RE = re.compile(r"^[A-Za-z0-9._-]{1,128}$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_SERVICE_MAP = {
    "motion": [
        "go2w-sport-gateway.service",
        "go2w-safety-observer.service",
        "go2w-motion.service",
    ],
    "web": ["go2w-web.service"],
    "nav": ["go2w-slam-nav.service"],
    "sensor": ["go2w-sensor.service"],
    "all": [
        "go2w-sport-gateway.service",
        "go2w-safety-observer.service",
        "go2w-motion.service",
        "go2w-web.service",
        "go2w-slam-nav.service",
        "go2w-sensor.service",
    ],
}
_VERIFICATION_COMMANDS = {
    "motion": (
        "python3 -m py_compile "
        "src/go2w_bridge/go2w_bridge/nx_safety_observer.py "
        "src/go2w_bridge/go2w_bridge/nx_sport_gateway.py "
        "src/go2w_bridge/go2w_bridge/nx_motion_node.py"),
    "web": "python3 -m compileall -q ai web",
    "nav": (
        "python3 -m compileall -q src tools && "
        "test -f src/go2w_nav/config/nav2_params_3d.yaml"),
    "sensor": (
        "python3 -m py_compile "
        "src/go2w_bridge/go2w_bridge/nx_sensor_node.py"),
    "all": "python3 -m compileall -q ai web src tools",
}
_MAX_MEMBERS = 10_000
_MAX_UNPACKED_BYTES = 2 * 1024 * 1024 * 1024
_REQUIRED_PAYLOAD_FILES = frozenset({
    "config/rooms.yaml",
    "ai/__init__.py",
    "ai/config.py",
    "ai/detector.py",
    "ai/locate_anything.py",
    "ai/tracker.py",
    "ai/vlm.py",
    "web/nx_ai_node.py",
    "web/nx_active_search.py",
    "web/nx_camera_calibration.py",
    "web/nx_c13_image_node.py",
    "web/nx_control_auth.py",
    "web/nx_frontier_planner.py",
    "web/nx_gimbal_node.py",
    "web/nx_lidar_node.py",
    "web/nx_web_server.py",
    "web/nx_mission_schema.py",
    "web/nx_motion_intent.py",
    "web/nx_navigation_arbiter.py",
    "web/nx_room_orchestrator.py",
    "web/nx_navigation_gateway.py",
    "web/nx_observation_sync.py",
    "web/nx_exploration_manager.py",
    "web/nx_person_localizer.py",
    "web/nx_person_mission.py",
    "web/nx_point_nav.py",
    "web/nx_product_command.py",
    "web/nx_slam_map.py",
    "web/costmap_bridge.py",
    "web/start_go2w_web.sh",
    "web/voice_command.py",
    "web/static/locate_anything_demo.png",
    "web/static/locate_anything_demo_en.png",
    "web/static/map.js",
    "web/static/mock_person.png",
    "web/static/panel.html",
    "src/go2w_bridge/go2w_bridge/build_info.py",
    "src/go2w_bridge/go2w_bridge/nx_motion_node.py",
    "src/go2w_bridge/go2w_bridge/nx_safety_observer.py",
    "src/go2w_bridge/go2w_bridge/nx_sport_gateway.py",
    "src/go2w_bridge/go2w_bridge/nx_sensor_node.py",
    "src/go2w_bridge/go2w_bridge/map_odom_fuser.py",
    "src/go2w_bridge/go2w_bridge/map_padding_bridge.py",
    "src/go2w_bridge/go2w_bridge/mid360_nav_bridge.py",
    "src/go2w_bridge/go2w_bridge/motion_controller.py",
    "src/go2w_bridge/go2w_bridge/motion_machine.py",
    "src/go2w_bridge/go2w_bridge/motion_protocol.py",
    "src/go2w_bridge/go2w_bridge/motion_safety.py",
    "src/go2w_bridge/go2w_bridge/motion_types.py",
    "src/go2w_bridge/go2w_bridge/unitree_sport_adapter.py",
    "src/go2w_bridge/go2w_bridge/sport_gateway_protocol.py",
    "src/go2w_bridge/go2w_bridge/sport_gateway_server.py",
    "src/go2w_bridge/go2w_bridge/sport_gateway_client.py",
    "src/go2w_bridge/go2w_bridge/safety_event_recorder.py",
    "src/go2w_nav/CMakeLists.txt",
    "src/go2w_nav/package.xml",
    "src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml",
    "src/go2w_nav/config/c13_intrinsic.yaml",
    "src/go2w_nav/config/fastlivo2_mid360_c13.yaml",
    "src/go2w_nav/config/fastlio_low_latency/mid360.yaml",
    "src/go2w_nav/config/nav2_params.yaml",
    "src/go2w_nav/config/nav2_params_3d.yaml",
    "src/go2w_nav/config/nav2_params_slim.yaml",
    "src/go2w_nav/config/slam_toolbox.yaml",
    "src/go2w_nav/config/slam_toolbox_online.yaml",
    "src/go2w_nav/launch/nav2.launch.py",
    "src/go2w_nav/launch/nav2_3d.launch.py",
    "src/go2w_nav/launch/nav2_slim.launch.py",
    "src/go2w_nav/launch/slam.launch.py",
    "src/go2w_nav/launch/slam_online.launch.py",
    "docker/bringup_slam_nav2.sh",
    "docker/prepare_fastlio_low_latency.sh",
    "docker/patches/fast_lio_latest_frame.patch",
    "docker/patches/fast_lio_livox_reliable_qos.patch",
    "docker/patches/fast_lio_body_cloud.patch",
    "docker/patches/fast_lio_body_cloud_qos.patch",
    "docker/patches/fast_lio_bounded_body_cloud.patch",
    "docker/patches/fast_lio_rotating_body_sample.patch",
    "docker/patches/fast_lio_angular_body_cloud.patch",
    "docker/diagnose_nav2_goal.sh",
    "docker/fastdds_udp.xml",
    "docker/go2w-motion.service",
    "docker/go2w-sport-gateway.service",
    "docker/go2w-safety-observer.service",
    "docker/go2w-web.service",
    "docker/go2w-slam-nav.service",
    "docker/go2w-sensor.service",
    "docker/costmap-bridge.service",
    "docker/livox-mid360-net.service",
    "docker/livox-mid360-driver.service",
    "docker/livox-mid360-watchdog.service",
    "tools/verify_release_artifact.py",
    "tools/nx_release_probe.py",
    "tools/nav2_preflight.py",
    "tools/fastlio_latency_gate.py",
    "tools/topic_rate_gate.py",
    "tools/nav_health_supervisor.py",
    "tools/nav_health_gate.py",
    "tools/wait_lifecycle_active.py",
    "tools/nav2_benchmark.py",
    "tools/diag_sport_requests.py",
    "tools/diag_sport_state.py",
    "tools/diag_wheel_dq.py",
    "tools/capture_map_pose.py",
    "tools/perception_preflight.py",
    "tools/sport_gateway_bootstrap_preflight.py",
    "tools/livox_stream_watchdog.py",
})


class ArtifactVerificationError(ValueError):
    pass


@dataclass(frozen=True)
class ArtifactSummary:
    path: str
    release_id: str
    payload_digest: str
    subsystem: str
    required_services: tuple[str, ...]
    file_count: int
    archive_bytes: int
    unpacked_bytes: int


def _safe_path(name: object, *, allow_manifest: bool = False) -> str:
    value = str(name or "")
    path = PurePosixPath(value)
    if (
        not value
        or value.startswith("/")
        or "\\" in value
        or any(part in {"", ".", ".."} for part in path.parts)
    ):
        raise ArtifactVerificationError(f"unsafe archive path: {value!r}")
    if allow_manifest and value == "manifest.json":
        return value
    if path.parts[0] != "payload":
        raise ArtifactVerificationError(f"unsafe archive path: {value!r}")
    return value


def _manifest_object(value: object) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ArtifactVerificationError("manifest must be a JSON object")
    return value


def _hash_manifest(value: object) -> dict[str, str]:
    if not isinstance(value, Mapping) or not value:
        raise ArtifactVerificationError("sha256 manifest must be non-empty")
    result = {}
    for raw_name, raw_digest in value.items():
        name = str(raw_name)
        _safe_path(f"payload/{name}")
        if name.startswith("payload/"):
            raise ArtifactVerificationError("sha256 paths must be payload-relative")
        digest = str(raw_digest)
        if not _SHA256_RE.fullmatch(digest):
            raise ArtifactVerificationError(f"invalid sha256 for {name}")
        result[name] = digest
    return result


def verify_artifact(path: object) -> ArtifactSummary:
    archive_path = Path(path).resolve()
    if not archive_path.is_file():
        raise ArtifactVerificationError(f"artifact not found: {archive_path}")

    try:
        archive = tarfile.open(archive_path, "r:gz")
    except (OSError, tarfile.TarError) as exc:
        raise ArtifactVerificationError("artifact is not a readable tar.gz") from exc

    with archive:
        members = archive.getmembers()
        if len(members) > _MAX_MEMBERS:
            raise ArtifactVerificationError("archive contains too many members")
        names = [member.name for member in members]
        if len(names) != len(set(names)):
            raise ArtifactVerificationError("archive contains duplicate member names")
        unpacked_bytes = 0
        for member in members:
            _safe_path(member.name, allow_manifest=True)
            if not (member.isfile() or member.isdir()):
                raise ArtifactVerificationError(
                    f"unsupported archive member: {member.name}")
            if member.size < 0:
                raise ArtifactVerificationError("archive contains a negative file size")
            unpacked_bytes += int(member.size)
        if unpacked_bytes > _MAX_UNPACKED_BYTES:
            raise ArtifactVerificationError("archive unpacked size exceeds safety limit")

        manifest_members = [
            member for member in members if member.name == "manifest.json"]
        if len(manifest_members) != 1 or not manifest_members[0].isfile():
            raise ArtifactVerificationError("archive must contain one manifest.json file")
        stream = archive.extractfile(manifest_members[0])
        if stream is None:
            raise ArtifactVerificationError("manifest.json cannot be read")
        try:
            manifest = _manifest_object(json.load(stream))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ArtifactVerificationError("manifest.json is invalid JSON") from exc

        if manifest.get("schema_version") != 1:
            raise ArtifactVerificationError("unsupported manifest schema_version")
        subsystem = str(manifest.get("subsystem", ""))
        if subsystem not in _SERVICE_MAP:
            raise ArtifactVerificationError("invalid release subsystem")
        services = manifest.get("required_services")
        if services != _SERVICE_MAP[subsystem]:
            raise ArtifactVerificationError("required services mismatch")
        if manifest.get("verification_command") != _VERIFICATION_COMMANDS[subsystem]:
            raise ArtifactVerificationError("verification command mismatch")
        release_id = str(manifest.get("release_id", ""))
        if not _RELEASE_RE.fullmatch(release_id):
            raise ArtifactVerificationError("invalid release_id")

        expected_hashes = _hash_manifest(manifest.get("sha256"))
        actual_members = {
            member.name[len("payload/"):]: member
            for member in members
            if member.isfile() and member.name.startswith("payload/")
        }
        expected_names = set(expected_hashes)
        actual_names = set(actual_members)
        unlisted = sorted(actual_names - expected_names)
        missing = sorted(expected_names - actual_names)
        if unlisted:
            raise ArtifactVerificationError(
                f"unlisted payload files: {', '.join(unlisted[:5])}")
        if missing:
            raise ArtifactVerificationError(
                f"missing payload files: {', '.join(missing[:5])}")
        runtime_missing = sorted(_REQUIRED_PAYLOAD_FILES - actual_names)
        if runtime_missing:
            raise ArtifactVerificationError(
                "missing required runtime files: "
                + ", ".join(runtime_missing[:5]))

        for relative, expected in expected_hashes.items():
            payload_stream = archive.extractfile(actual_members[relative])
            if payload_stream is None:
                raise ArtifactVerificationError(
                    f"payload file cannot be read: {relative}")
            actual = hashlib.sha256(payload_stream.read()).hexdigest()
            if actual != expected:
                raise ArtifactVerificationError(f"sha256 mismatch: {relative}")

        payload_digest = hashlib.sha256(
            json.dumps(
                expected_hashes, sort_keys=True, separators=(",", ":")
            ).encode("utf-8")
        ).hexdigest()
        if manifest.get("payload_digest") != payload_digest:
            raise ArtifactVerificationError("payload digest mismatch")
        if not release_id.endswith(f"-{payload_digest[:12]}"):
            raise ArtifactVerificationError("release_id is not content-addressed")

    return ArtifactSummary(
        path=str(archive_path),
        release_id=release_id,
        payload_digest=payload_digest,
        subsystem=subsystem,
        required_services=tuple(_SERVICE_MAP[subsystem]),
        file_count=len(expected_hashes),
        archive_bytes=archive_path.stat().st_size,
        unpacked_bytes=unpacked_bytes,
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)
    try:
        summary = verify_artifact(args.artifact)
    except ArtifactVerificationError as exc:
        parser.exit(1, f"release artifact invalid: {exc}\n")
    if not args.quiet:
        print(json.dumps(asdict(summary), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
