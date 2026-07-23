from pathlib import Path

import verify_release
from verify_release import architecture_violations


ROOT = Path(__file__).resolve().parents[1]


def test_current_tree_satisfies_release_architecture_contract():
    assert architecture_violations(ROOT) == []


def test_room_orchestrator_contains_no_legacy_nav_client_or_clustering():
    room = (ROOT / "web" / "nx_room_orchestrator.py").read_text(
        encoding="utf-8")

    assert "class Nav2ActionClient" not in room
    assert "class _NavOperation" not in room
    assert "def _find_frontier_clusters" not in room
    assert room.count("def select_frontier_candidates") == 1


def test_command_path_contains_no_legacy_motion_fallback_parser():
    ai = (ROOT / "web" / "nx_ai_node.py").read_text(encoding="utf-8")
    web = (ROOT / "web" / "nx_web_server.py").read_text(encoding="utf-8")

    assert "def _fallback_parse" not in ai
    assert "def _fallback_parse" not in web
    assert "def _validate_vlm_search_result" in ai


def test_architecture_gate_rejects_deploy_without_safe_park_probe(monkeypatch):
    original_read = verify_release._read

    def read_without_probe(root, relative):
        source = original_read(root, relative)
        if relative == "docker/deploy_release.sh":
            source = source.replace("tools/nx_release_probe.py", "tools/removed.py")
        return source

    monkeypatch.setattr(verify_release, "_read", read_without_probe)

    assert "full deploy has no read-only safe-park, Nav2 and perception probes" in (
        architecture_violations(ROOT))


def test_architecture_gate_rejects_upload_before_remote_preflight(monkeypatch):
    original_read = verify_release._read

    def read_with_late_preflight(root, relative):
        source = original_read(root, relative)
        if relative == "docker/deploy_release.sh":
            source = source.replace(
                'preflight_output="$(remote_preflight)"',
                'preflight_output="late"',
            )
            source = source.replace(
                '"${SCP[@]}" "$ARTIFACT"',
                '"${SCP[@]}" "$ARTIFACT"\n'
                'preflight_output="$(remote_preflight)"',
                1,
            )
        return source

    monkeypatch.setattr(verify_release, "_read", read_with_late_preflight)

    assert "remote preflight must run before artifact upload" in (
        architecture_violations(ROOT))


def test_architecture_gate_recognizes_bound_scp_array_upload():
    problems = architecture_violations(ROOT)

    assert "remote preflight must run before artifact upload" not in problems
    assert "artifact must pass strict local verification before upload" not in problems


def test_architecture_gate_rejects_missing_safe_first_token_bootstrap(monkeypatch):
    original_read = verify_release._read

    def read_without_token_bootstrap(root, relative):
        source = original_read(root, relative)
        if relative == "web/nx_control_auth.py":
            source = source.replace(
                'return AuthorizationDecision(True, 200, "auth_disabled")',
                '# auth-enabled test fixture',
            )
        if relative == "docker/deploy_release.sh":
            source = source.replace(
                '"$SCRIPT_DIR/../tools/generate_control_token.py"',
                '"$SCRIPT_DIR/../tools/removed_token_tool.py"',
            )
        return source

    monkeypatch.setattr(verify_release, "_read", read_without_token_bootstrap)

    assert "first Web deployment has no safe explicit token bootstrap" in (
        architecture_violations(ROOT))


def test_architecture_gate_rejects_nav_restart_before_mid360_runtime(monkeypatch):
    assert "Nav2 deploy does not restart the current MID360 runtime first" not in (
        architecture_violations(ROOT))
    original_read = verify_release._read

    def read_with_wrong_restart_order(root, relative):
        source = original_read(root, relative)
        if relative == "docker/deploy_release.sh":
            source = source.replace(
                'restart_units="livox-mid360-net.service '
                'livox-mid360-driver.service livox-mid360-watchdog.service '
                'go2w-sensor.service go2w-slam-nav.service"',
                'restart_units="go2w-slam-nav.service '
                'livox-mid360-net.service livox-mid360-driver.service '
                'livox-mid360-watchdog.service go2w-sensor.service"',
            )
        return source

    monkeypatch.setattr(verify_release, "_read", read_with_wrong_restart_order)

    assert "Nav2 deploy does not restart the current MID360 runtime first" in (
        architecture_violations(ROOT))


def test_architecture_gate_accepts_explicit_auth_disabled_panel():
    problems = architecture_violations(ROOT)

    assert "panel does not send authenticated control requests" not in problems


def test_architecture_gate_rejects_dead_auth_disabled_return(monkeypatch):
    original_read = verify_release._read

    def read_with_dead_auth_return(root, relative):
        source = original_read(root, relative)
        if relative == "web/nx_control_auth.py":
            source = source.replace(
                '    return AuthorizationDecision(True, 200, "auth_disabled")',
                '    if False:\n'
                '        return AuthorizationDecision(True, 200, "auth_disabled")',
                1,
            )
        return source

    monkeypatch.setattr(verify_release, "_read", read_with_dead_auth_return)

    assert "panel does not send authenticated control requests" in (
        architecture_violations(ROOT))


def test_architecture_gate_rejects_sensor_publishing_primary_odom(monkeypatch):
    assert "sensor/Nav2 odometry ownership is ambiguous" not in (
        architecture_violations(ROOT))
    original_read = verify_release._read

    def read_with_duplicate_odom_owner(root, relative):
        source = original_read(root, relative)
        if relative == "docker/go2w-sensor.service":
            source = source.replace(
                "-p publish_odom_tf:=false -p odom_topic:=/wheel_odom",
                "-p publish_odom_tf:=true -p odom_topic:=/odom",
            )
        return source

    monkeypatch.setattr(verify_release, "_read", read_with_duplicate_odom_owner)

    assert "sensor/Nav2 odometry ownership is ambiguous" in (
        architecture_violations(ROOT))


def test_architecture_gate_rejects_duplicate_unsafe_sensor_params(monkeypatch):
    original_read = verify_release._read

    def read_with_overridden_sensor_params(root, relative):
        source = original_read(root, relative)
        if relative == "docker/go2w-sensor.service":
            source = source.replace(
                "-p publish_odom_tf:=false -p odom_topic:=/wheel_odom",
                "-p publish_odom_tf:=false -p odom_topic:=/wheel_odom "
                "-p publish_odom_tf:=true -p odom_topic:=/odom",
                1,
            )
        return source

    monkeypatch.setattr(
        verify_release, "_read", read_with_overridden_sensor_params)

    assert "sensor/Nav2 odometry ownership is ambiguous" in (
        architecture_violations(ROOT))


def test_architecture_gate_rejects_nav_reverse_at_either_boundary(monkeypatch):
    assert "autonomous reverse is not blocked at both Nav2 and SDK boundaries" not in (
        architecture_violations(ROOT))
    original_read = verify_release._read

    def read_with_reverse_enabled(root, relative):
        source = original_read(root, relative)
        if relative == "src/go2w_nav/config/nav2_params_3d.yaml":
            source = source.replace(
                "min_velocity: [0.0, 0.0, -0.5]",
                "min_velocity: [-0.1, 0.0, -0.5]",
            )
        return source

    monkeypatch.setattr(verify_release, "_read", read_with_reverse_enabled)

    assert "autonomous reverse is not blocked at both Nav2 and SDK boundaries" in (
        architecture_violations(ROOT))


def test_architecture_gate_rejects_sdk_clamp_left_only_in_comment(monkeypatch):
    original_read = verify_release._read

    def read_with_commented_sdk_clamp(root, relative):
        source = original_read(root, relative)
        if relative == "src/go2w_bridge/go2w_bridge/motion_controller.py":
            source = source.replace(
                "velocity = (max(0.0, velocity[0]), velocity[1], velocity[2])",
                "velocity = velocity  # max(0.0, velocity[0])",
                1,
            )
        return source

    monkeypatch.setattr(verify_release, "_read", read_with_commented_sdk_clamp)

    assert "autonomous reverse is not blocked at both Nav2 and SDK boundaries" in (
        architecture_violations(ROOT))


def test_architecture_gate_rejects_restart_units_later_override(monkeypatch):
    original_read = verify_release._read

    def read_with_late_restart_override(root, relative):
        source = original_read(root, relative)
        if relative == "docker/deploy_release.sh":
            marker = (
                'restart_units="livox-mid360-net.service '
                'livox-mid360-driver.service livox-mid360-watchdog.service '
                'go2w-sensor.service go2w-slam-nav.service"'
            )
            source = source.replace(
                marker,
                marker + '\nrestart_units="go2w-slam-nav.service"',
                1,
            )
        return source

    monkeypatch.setattr(verify_release, "_read", read_with_late_restart_override)

    assert "Nav2 deploy does not restart the current MID360 runtime first" in (
        architecture_violations(ROOT))
