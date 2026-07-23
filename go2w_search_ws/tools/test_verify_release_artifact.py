import hashlib
import io
import json
import tarfile

import pytest


def test_completeness_contract_requires_motion_nav_web_ai_and_validation_runtime():
    from verify_release_artifact import _REQUIRED_PAYLOAD_FILES

    for required in (
        "ai/detector.py",
        "ai/vlm.py",
        "web/nx_ai_node.py",
        "web/nx_web_server.py",
        "src/go2w_bridge/go2w_bridge/nx_motion_node.py",
        "src/go2w_bridge/go2w_bridge/nx_sport_gateway.py",
        "src/go2w_bridge/go2w_bridge/nx_safety_observer.py",
        "src/go2w_bridge/go2w_bridge/sport_gateway_client.py",
        "src/go2w_nav/config/nav2_params_3d.yaml",
        "tools/nx_release_probe.py",
        "tools/nav2_preflight.py",
        "tools/capture_map_pose.py",
        "tools/sport_gateway_bootstrap_preflight.py",
        "docker/go2w-motion.service",
        "docker/go2w-sport-gateway.service",
        "docker/go2w-safety-observer.service",
        "docker/go2w-slam-nav.service",
    ):
        assert required in _REQUIRED_PAYLOAD_FILES


def test_completeness_contract_covers_direct_service_and_import_dependencies():
    from verify_release_artifact import _REQUIRED_PAYLOAD_FILES

    for required in (
        "docker/fastdds_udp.xml",
        "web/start_go2w_web.sh",
        "web/nx_control_auth.py",
        "web/nx_point_nav.py",
        "web/nx_person_localizer.py",
        "web/nx_frontier_planner.py",
        "web/nx_global_search_state.py",
        "web/nx_visibility_coverage.py",
        "web/static/panel.html",
        "web/static/map.js",
        "src/go2w_bridge/go2w_bridge/motion_controller.py",
        "src/go2w_bridge/go2w_bridge/motion_protocol.py",
        "src/go2w_bridge/go2w_bridge/motion_safety.py",
        "src/go2w_bridge/go2w_bridge/motion_types.py",
        "src/go2w_bridge/go2w_bridge/sport_gateway_protocol.py",
        "src/go2w_bridge/go2w_bridge/sport_gateway_server.py",
        "src/go2w_bridge/go2w_bridge/safety_event_recorder.py",
        "src/go2w_bridge/go2w_bridge/map_odom_fuser.py",
        "src/go2w_bridge/go2w_bridge/map_padding_bridge.py",
        "src/go2w_bridge/go2w_bridge/mid360_nav_bridge.py",
        "src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml",
        "src/go2w_nav/config/slam_toolbox_online.yaml",
        "src/go2w_nav/launch/slam_online.launch.py",
    ):
        assert required in _REQUIRED_PAYLOAD_FILES


def _artifact(tmp_path, *, extra=None, member_name="payload/app.py",
              tamper_digest=False, subsystem="web", services=None,
              symlink=False, verification_command=None):
    from verify_release_artifact import (
        _REQUIRED_PAYLOAD_FILES,
        _VERIFICATION_COMMANDS,
    )

    payload = b"print('ok')\n"
    payloads = {
        relative: f"# fixture {relative}\n".encode()
        for relative in _REQUIRED_PAYLOAD_FILES
    }
    payloads["app.py"] = payload
    hashes = {
        relative: hashlib.sha256(content).hexdigest()
        for relative, content in payloads.items()
    }
    digest = hashlib.sha256(
        json.dumps(hashes, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    manifest = {
        "schema_version": 1,
        "release_id": f"test-{digest[:12]}",
        "base_release_id": "test",
        "payload_digest": ("0" * 64 if tamper_digest else digest),
        "subsystem": subsystem,
        "required_services": (
            ["go2w-web.service"] if services is None else services),
        "verification_command": (
            _VERIFICATION_COMMANDS[subsystem]
            if verification_command is None else verification_command),
        "sha256": hashes,
    }
    path = tmp_path / "release.tar.gz"
    with tarfile.open(path, "w:gz") as archive:
        raw = json.dumps(manifest).encode()
        info = tarfile.TarInfo("manifest.json")
        info.size = len(raw)
        archive.addfile(info, io.BytesIO(raw))
        for relative, content in sorted(payloads.items()):
            name = member_name if relative == "app.py" else f"payload/{relative}"
            info = tarfile.TarInfo(name)
            if symlink and relative == "app.py":
                info.type = tarfile.SYMTYPE
                info.linkname = "/tmp/outside"
                archive.addfile(info)
            else:
                info.size = len(content)
                archive.addfile(info, io.BytesIO(content))
        if extra is not None:
            info = tarfile.TarInfo("payload/extra.py")
            info.size = len(extra)
            archive.addfile(info, io.BytesIO(extra))
    return path, digest


def test_valid_content_addressed_artifact_is_accepted(tmp_path):
    from verify_release_artifact import _REQUIRED_PAYLOAD_FILES, verify_artifact

    path, digest = _artifact(tmp_path)
    summary = verify_artifact(path)

    assert summary.release_id == f"test-{digest[:12]}"
    assert summary.payload_digest == digest
    assert summary.subsystem == "web"
    assert summary.file_count == len(_REQUIRED_PAYLOAD_FILES) + 1


def test_nav_artifact_requires_sensor_and_slam_services(tmp_path):
    from verify_release_artifact import verify_artifact

    path, _digest = _artifact(
        tmp_path,
        subsystem="nav",
        services=["go2w-sensor.service", "go2w-slam-nav.service"],
    )

    summary = verify_artifact(path)
    assert summary.required_services == (
        "go2w-sensor.service", "go2w-slam-nav.service")


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"extra": b"unlisted"}, "unlisted payload files"),
        ({"tamper_digest": True}, "payload digest mismatch"),
        ({"member_name": "../escape"}, "unsafe archive path"),
        ({"symlink": True}, "unsupported archive member"),
        ({"services": ["go2w-motion.service"]}, "required services mismatch"),
        ({"verification_command": "touch /tmp/pwned"},
         "verification command mismatch"),
    ],
)
def test_corrupt_or_unsafe_artifact_is_rejected(tmp_path, kwargs, message):
    from verify_release_artifact import ArtifactVerificationError, verify_artifact

    path, _digest = _artifact(tmp_path, **kwargs)

    with pytest.raises(ArtifactVerificationError, match=message):
        verify_artifact(path)
