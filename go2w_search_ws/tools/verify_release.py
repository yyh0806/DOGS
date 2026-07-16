#!/usr/bin/env python3
"""One offline release gate for the Go2W NX runtime."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def _read(root: Path, relative: str) -> str:
    return (root / relative).read_text(encoding="utf-8")


def architecture_violations(root: Path = ROOT) -> list[str]:
    root = Path(root)
    problems = []
    motion = _read(root, "src/go2w_bridge/go2w_bridge/nx_motion_node.py")
    gateway = _read(
        root, "src/go2w_bridge/go2w_bridge/nx_sport_gateway.py")
    safety_observer = _read(
        root, "src/go2w_bridge/go2w_bridge/nx_safety_observer.py")
    gateway_server = _read(
        root, "src/go2w_bridge/go2w_bridge/sport_gateway_server.py")
    sport_adapter = _read(
        root, "src/go2w_bridge/go2w_bridge/unitree_sport_adapter.py")
    web = _read(root, "web/nx_web_server.py")
    ai = _read(root, "web/nx_ai_node.py")
    room = _read(root, "web/nx_room_orchestrator.py")
    panel = _read(root, "web/static/panel.html")
    nav_deploy = _read(root, "docker/deploy_nav2_bprime.sh")
    legacy_deploys = {
        "motion": _read(root, "docker/deploy_nx.sh"),
        "web": _read(root, "docker/deploy_nx_web.sh"),
        "nav": nav_deploy,
    }
    build_release = _read(root, "docker/build_release.sh")
    deploy_release = _read(root, "docker/deploy_release.sh")
    bringup = _read(root, "docker/bringup_slam_nav2.sh")
    nav_unit = _read(root, "docker/go2w-slam-nav.service")
    nav_params = _read(root, "src/go2w_nav/config/nav2_params_3d.yaml")
    nav_preflight = _read(root, "tools/nav2_preflight.py")
    motion_controller = _read(
        root, "src/go2w_bridge/go2w_bridge/motion_controller.py")

    if motion.count("machine = Go2WMotionMachine(") != 1:
        problems.append("motion node must construct exactly one Go2WMotionMachine")
    if motion.count("adapter = SportGatewayClient(") != 1:
        problems.append("motion node must construct exactly one SportGatewayClient")
    if "SportClient(" in motion or "MotionSwitcherClient(" in motion:
        problems.append("motion policy must not own Unitree SDK clients")
    if gateway.count("SportClient(enableLease=True)") != 1:
        problems.append("stable gateway must own exactly one leased SportClient")
    for forbidden in (
        "ChannelSubscriber", "LowState_", "SportModeState_",
        "RawSafetyObserver",
    ):
        if forbidden in gateway:
            problems.append(
                f"stable lease gateway contains raw DDS observer: {forbidden}")
    for forbidden in ("SportClient", "MotionSwitcherClient", ".Move("):
        if forbidden in safety_observer:
            problems.append(
                f"read-only safety observer contains control client: {forbidden}")
    for legacy in (
        "DriveSessionModel", "Go2WStateModel", "LayeredMotionState",
        "BALANCE_UNCONFIRMED", "_synchronize_active_session_state",
        "_adopt_startup_feedback",
    ):
        if legacy in motion:
            problems.append(f"legacy motion authority remains: {legacy}")
    for forbidden in ('"Damp"', '"RecoveryStand"', '"StandDown"'):
        if forbidden in sport_adapter + gateway + gateway_server:
            problems.append(
                f"autonomous sport adapter exposes support-changing operation: {forbidden}")

    if web.count("PointNavigationController(") != 1:
        problems.append("web runtime must construct one NavigateToPose action port")
    if web.count("NavigationGateway(") != 1:
        problems.append("web runtime must construct one NavigationGateway")
    if "navigation_port=mission_navigation" not in web:
        problems.append("room missions are not injected with the shared gateway")
    if "class Nav2ActionClient" in room or "class _NavOperation" in room:
        problems.append("room orchestrator still defines a legacy Nav2 client")
    if "def _find_frontier_clusters" in room:
        problems.append("room orchestrator still embeds frontier clustering")
    if room.count("def select_frontier_candidates") != 1:
        problems.append("room orchestrator must expose one frontier planner facade")

    auth_index = web.find("decision = authorize_request(")
    body_index = web.find("Content-Length", auth_index)
    if auth_index < 0 or body_index < 0 or auth_index > body_index:
        problems.append("control authorization must run before request-body parsing")
    if "controlFetch(" not in panel or "Authorization" not in panel:
        problems.append("panel does not send authenticated control requests")
    if "Access-Control-Allow-Origin', '*'" in web:
        problems.append("wildcard control CORS is forbidden")

    if "restart go2w-motion" in nav_deploy or "nx_motion_node.py" in nav_deploy:
        problems.append("Nav2 deployment may not replace or restart motion")
    for subsystem, source in legacy_deploys.items():
        marker = "# LEGACY IMPLEMENTATION BELOW IS UNREACHABLE"
        wrapper = source.split(marker, 1)[0]
        if marker not in source or f'build_release.sh" {subsystem}' not in wrapper:
            problems.append(
                f"legacy {subsystem} deploy does not forward to atomic builder")
        if 'deploy_release.sh" "$artifact" "$@"' not in wrapper:
            problems.append(
                f"legacy {subsystem} deploy does not forward to atomic deployer")
    for required in (
        "manifest.json", "sha256", "/home/nx/go2w/releases",
        "mv -Tf \"$current.next\" \"$current\"",
    ):
        if required not in build_release + deploy_release:
            problems.append(f"atomic deployment contract missing: {required}")
    for packaged_tool in (
        'copy_path "tools/verify_release_artifact.py"',
        'copy_path "tools/nx_release_probe.py"',
        'copy_path "tools/diag_sport_requests.py"',
        'copy_path "tools/diag_sport_state.py"',
        'copy_path "tools/nav2_preflight.py"',
        'copy_path "tools/nav2_benchmark.py"',
        'copy_path "tools/capture_map_pose.py"',
        'copy_path "tools/perception_preflight.py"',
        'copy_path "tools/sport_gateway_bootstrap_preflight.py"',
        'copy_path "docker/costmap-bridge.service"',
        'copy_path "docker/go2w-sport-gateway.service"',
        'copy_path "docker/go2w-safety-observer.service"',
        'copy_path "docker/livox-mid360-net.service"',
        'copy_path "docker/livox-mid360-driver.service"',
        'copy_path "docker/livox-mid360-watchdog.service"',
        'copy_path "tools/livox_stream_watchdog.py"',
    ):
        if packaged_tool not in build_release:
            problems.append(f"release builder omits deployment tool: {packaged_tool}")
    for ai_module in (
        "ai/__init__.py", "ai/config.py", "ai/detector.py",
        "ai/locate_anything.py", "ai/tracker.py", "ai/vlm.py",
    ):
        if f'copy_path "{ai_module}"' not in build_release:
            problems.append(f"release builder omits AI runtime: {ai_module}")
    upload_index = deploy_release.find('"${SCP[@]}" "$ARTIFACT"')
    preflight_index = deploy_release.find(
        'preflight_output="$(remote_preflight)"')
    if (upload_index < 0 or preflight_index < 0
            or preflight_index > upload_index):
        problems.append("remote preflight must run before artifact upload")
    if ("ip route get 192.168.123.161" not in deploy_release
            or "/etc/go2w/hardware.env" not in deploy_release):
        problems.append("deployment does not persist the detected Go2W interface")
    if ("ip route get 192.168.1.160" not in deploy_release
            or "GO2W_LIVOX_INTERFACE=" not in deploy_release):
        problems.append("deployment does not persist the detected MID360 interface")
    if ("import numpy, cv2, torch, ultralytics" not in deploy_release
            or "GO2W_YOLO_MODEL_PATH=" not in deploy_release):
        problems.append("deployment does not preflight YOLO-World runtime")
    local_verify_index = deploy_release.find(
        '"$SCRIPT_DIR/../tools/verify_release_artifact.py"')
    if (local_verify_index < 0 or upload_index < 0
            or local_verify_index > upload_index):
        problems.append("artifact must pass strict local verification before upload")
    token_generator_index = deploy_release.find(
        '"$SCRIPT_DIR/../tools/generate_control_token.py"')
    if (not (root / "tools/generate_control_token.py").is_file()
            or "--generate-control-token-file" not in deploy_release
            or local_verify_index < 0
            or token_generator_index < 0
            or preflight_index < 0
            or not local_verify_index < token_generator_index < preflight_index):
        problems.append(
            "first Web deployment has no safe explicit token bootstrap")
    nav_restart_units = (
        'restart_units="livox-mid360-net.service '
        'livox-mid360-driver.service livox-mid360-watchdog.service '
        'go2w-slam-nav.service"'
    )
    if (nav_restart_units not in deploy_release
            or "for service in $restart_units" not in deploy_release):
        problems.append(
            "Nav2 deploy does not restart the current MID360 runtime first")
    restart_loop = deploy_release.find("for service in $restart_units")
    restart_done = deploy_release.find("done", restart_loop)
    if (restart_loop < 0 or restart_done < 0
            or "go2w-sport-gateway.service" in
            deploy_release[restart_loop:restart_done]
            or "gateway_bootstrap_required" not in deploy_release
            or "gateway_bootstrap_failed_zero_hold" not in deploy_release):
        problems.append(
            "release deployment may interrupt or unsafely roll back the Sport gateway")
    bootstrap_state_gate = deploy_release.find(
        "tools/sport_gateway_bootstrap_preflight.py")
    transaction_switch = deploy_release.find(
        "backup_system_state", bootstrap_state_gate)
    if (bootstrap_state_gate < 0 or transaction_switch < 0
            or "gateway_bootstrap_state_not_safe" not in deploy_release
            or deploy_release.count(
                "tools/sport_gateway_bootstrap_preflight.py") != 2
            or bootstrap_state_gate >= transaction_switch):
        problems.append(
            "gateway bootstrap lacks a read-only parked-state gate")
    restart_index = deploy_release.find('systemctl restart "$service"')
    probe_index = deploy_release.find("tools/nx_release_probe.py", restart_index)
    probe_success_index = deploy_release.find("trap - ERR", probe_index)
    if (restart_index < 0 or probe_index < 0 or probe_success_index < 0
            or not restart_index < probe_index < probe_success_index
            or "--require-sdk-ready" not in deploy_release[probe_index:probe_success_index]
            or "tools/nav2_preflight.py" not in
            deploy_release[probe_index:probe_success_index]
            or "tools/perception_preflight.py" not in
            deploy_release[probe_index:probe_success_index]
            or 'validation/${release_id}-deploy.json' not in
            deploy_release[probe_index:probe_success_index]
            or 'validation/${release_id}-perception-preflight.json' not in
            deploy_release[probe_index:probe_success_index]
            or 'validation/${release_id}-nav2-preflight.json' not in
            deploy_release[probe_index:probe_success_index]):
        problems.append(
            "full deploy has no read-only safe-park, Nav2 and perception probes")

    final_prefix = deploy_release.find('cd "$release_dir/payload"')
    colcon_build = deploy_release.find(
        "colcon build --packages-select go2w_nav --symlink-install")
    current_switch = deploy_release.find(
        'ln -sfn "$release_dir" "$current.next"')
    if not (0 <= final_prefix < colcon_build < current_switch):
        problems.append("Nav2 colcon build does not use the final release prefix")
    if ("Conflicts=go2w-sensor.service" not in nav_unit
            or "After=go2w-sensor.service" not in nav_unit
            or "systemctl disable go2w-sensor.service" not in deploy_release):
        problems.append("Nav2 and the fallback sensor can own odom concurrently")
    if ("check_tf_topology || warn" in bringup
            or 'die "base_link 双 parent:' not in bringup):
        problems.append("Nav2 bringup does not fail closed on a double-parent TF")
    if ("min_velocity: [0.0, 0.0, -0.15]" not in nav_params
            or "max(0.0, velocity[0])" not in motion_controller):
        problems.append("autonomous reverse is not blocked at both Nav2 and SDK boundaries")
    for probe in ("global_costmap_fresh", "costmap_bridge_active"):
        if probe not in nav_preflight:
            problems.append(f"Nav2 preflight omits {probe}")

    for relative in (
        "docker/go2w-sport-gateway.service",
        "docker/go2w-motion.service",
        "docker/go2w-web.service",
        "docker/go2w-slam-nav.service",
        "docker/go2w-sensor.service",
        "docker/costmap-bridge.service",
    ):
        service = _read(root, relative)
        if "/home/nx/go2w/current/payload" not in service:
            problems.append(f"service does not use atomic current link: {relative}")
        if "EnvironmentFile=-/etc/go2w/release.env" not in service:
            problems.append(f"service has no release fingerprint environment: {relative}")
        if relative in {
                "docker/go2w-sport-gateway.service",
                "docker/go2w-motion.service",
                "docker/go2w-web.service",
                "docker/go2w-sensor.service",
        } and "EnvironmentFile=-/etc/go2w/hardware.env" not in service:
            problems.append(
                f"dog-facing service has no detected hardware environment: {relative}")

    if "SearchMissionRequest.from_api_payload" not in web:
        problems.append("HTTP search does not validate the canonical mission schema")
    if "def _fallback_parse" in web or "def _fallback_parse" in ai:
        problems.append("legacy command fallback parser remains")
    if "canonicalize_search_tasks(data.get(\"tasks\"))" not in ai:
        problems.append("VLM output does not validate the canonical mission schema")
    if 't.get("type", "move")' in web:
        problems.append("command admission defaults a missing task type to motion")
    if "canonicalize_search_tasks(tasks)" not in web:
        problems.append("command admission bypasses the canonical mission schema")
    if "_parse_failure(\"vlm_timeout\"" not in ai:
        problems.append("VLM timeout does not fail closed")
    if 'reason = "release_mismatch"' not in web:
        problems.append("v4 motion/web release mismatch does not fail closed")
    if "ObservationSynchronizer(" not in web:
        problems.append("web runtime has no observation synchronizer")
    if "ExplorationManager(" not in room:
        problems.append("room exploration does not delegate persistent state")
    return problems


def _run(command: list[str], *, cwd: Path) -> bool:
    print("+", " ".join(command), flush=True)
    completed = subprocess.run(command, cwd=cwd, check=False)
    return completed.returncode == 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--architecture-only", action="store_true",
        help="run static ownership/security/deployment checks only")
    args = parser.parse_args(argv)

    problems = architecture_violations(ROOT)
    if problems:
        for problem in problems:
            print(f"ARCHITECTURE FAIL: {problem}", file=sys.stderr)
        return 1
    print("architecture: PASS")
    if args.architecture_only:
        return 0

    commands = [
        [sys.executable, "-m", "compileall", "-q", "ai", "web", "src", "tools", "docker"],
        [sys.executable, "-m", "pytest", "docker", "web", "tools", "src/go2w_bridge/test", "-q"],
    ]
    node = shutil.which("node")
    if node is None:
        print("RELEASE FAIL: node executable not found", file=sys.stderr)
        return 1
    commands.extend([
        [node, "web/test_map_contract.js"],
        [node, "web/test_panel_nav_state.js"],
    ])
    for command in commands:
        if not _run(command, cwd=ROOT):
            return 1
    print("offline release gate: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
