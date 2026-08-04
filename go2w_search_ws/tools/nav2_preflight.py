#!/usr/bin/env python3
"""Read-only preflight for Panel -> Nav2 operation on the NX.

This program never publishes a ROS message and never creates a SportClient.
Run it on the NX after sourcing ROS 2 and the workspace setup files.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Mapping


RECOVERY = {
    "panel_connected": "Start go2w-web and go2w-motion, then wait for fresh /dog_state.",
    "motion_sdk": "Check dog power/network and go2w-motion logs; do not command motion.",
    "mid360_scan": "Check MID360 power, Livox driver, and /scan_mid360 freshness.",
    "fastlio_localization": "Wait for Fast-LIO map->base_link localization or inspect its logs.",
    "map_to_base_link_tf": "Restore the map->odom->base_link TF chain before navigation.",
    "navigate_to_pose_action": "Start/activate Nav2 and verify /navigate_to_pose.",
    "local_costmap_fresh": "Check MID360 costmap source and local_costmap lifecycle state.",
    "global_costmap_fresh": "Check global_costmap lifecycle state and its MID360 obstacle layer.",
    "costmap_bridge_active": "Start costmap-bridge.service so the Panel receives live obstacles.",
    "battery": "Replace or charge the battery before enabling wheel motion.",
    "drive_fault_clear": "Lock mode 6 and zero wheels, then use Panel safe fault reset if offered.",
    "motion_state_stopped": "Use Panel Stand/Stop and wait for STOPPED before preflight.",
    "joint_lock_mode_6": "Do not continue unless sport mode 6 (jointLock) is reported.",
    "wheels_still": "Keep clear, stop the dog, and verify all four wheel speeds are near zero.",
    "navigation_gate": "Resolve the backend navigation reason shown in Panel.",
}


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _check(name: str, ok: bool, detail: Any) -> dict[str, Any]:
    return {
        "name": name,
        "ok": bool(ok),
        "detail": detail,
        "recovery": None if ok else RECOVERY[name],
    }


def evaluate_status(
    status: Mapping[str, Any], probes: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """Evaluate an API snapshot plus read-only ROS probe results."""
    probes = probes or {}
    navigation = status.get("navigation") or {}
    localization = status.get("localization") or {}
    battery = _finite_number(navigation.get("battery_soc"))
    minimum_battery = _finite_number(
        navigation.get("minimum_battery_soc"))
    battery_ok = (
        battery is not None and minimum_battery is not None
        and battery >= minimum_battery)
    wheels = navigation.get("wheel_dq")
    wheel_values = None
    if isinstance(wheels, (list, tuple)) and len(wheels) == 4:
        converted = [_finite_number(value) for value in wheels]
        if all(value is not None for value in converted):
            wheel_values = converted
    mean_wheel_speed = (
        sum(abs(value) for value in wheel_values) / 4.0
        if wheel_values is not None else None)

    def probe(name: str) -> tuple[bool, Any]:
        if name not in probes:
            return False, "not_probed"
        value = probes[name]
        if isinstance(value, Mapping):
            return bool(value.get("ok")), value.get("detail")
        return value is True, value

    tf_ok, tf_detail = probe("map_to_base_link")
    fastlio_ok, fastlio_detail = probe("fastlio_active")
    action_ok, action_detail = probe("navigate_to_pose_action")
    costmap_ok, costmap_detail = probe("local_costmap_fresh")
    global_costmap_ok, global_costmap_detail = probe(
        "global_costmap_fresh")
    bridge_ok, bridge_detail = probe("costmap_bridge_active")
    checks = [
        _check("panel_connected", status.get("connected") is True,
               status.get("connected")),
        _check("motion_sdk", navigation.get("sdk_ready") is True,
               navigation.get("sdk_ready")),
        _check("mid360_scan", navigation.get("nav_scan_fresh") is True,
               navigation.get("nav_scan_fresh")),
        _check(
            "fastlio_localization",
            fastlio_ok and localization.get("healthy") is True,
            {
                "service": fastlio_detail,
                "localization_healthy": localization.get("healthy"),
                "localization_reason": localization.get("reason", "unknown"),
            },
        ),
        _check("map_to_base_link_tf", tf_ok, tf_detail),
        _check("navigate_to_pose_action", action_ok, action_detail),
        _check("local_costmap_fresh", costmap_ok, costmap_detail),
        _check("global_costmap_fresh", global_costmap_ok,
               global_costmap_detail),
        _check("costmap_bridge_active", bridge_ok, bridge_detail),
        _check("battery", battery_ok, {
            "soc": battery, "minimum_soc": minimum_battery}),
        _check("drive_fault_clear", navigation.get("drive_fault") is None,
               navigation.get("drive_fault")),
        _check("motion_state_stopped", status.get("dog_state") == "STOPPED",
               status.get("dog_state")),
        _check("joint_lock_mode_6", navigation.get("sport_mode") == 6,
               navigation.get("sport_mode")),
        _check("wheels_still",
               mean_wheel_speed is not None and mean_wheel_speed < 0.15,
               mean_wheel_speed),
        _check(
            "navigation_gate",
            navigation.get("activatable") is True
            and navigation.get("drive_session") == "parked",
            {
                "activatable": navigation.get("activatable"),
                "drive_session": navigation.get("drive_session"),
                "reason": navigation.get("reason", "unknown"),
            },
        ),
    ]
    return {
        "ok": all(item["ok"] for item in checks),
        "read_only": True,
        "checks": checks,
    }


def _run_probe(command: list[str], timeout: float) -> tuple[bool, str]:
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout,
            check=False)
        output = ((result.stdout or "") + (result.stderr or "")).strip()
        return result.returncode == 0, output[-1000:]
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr = exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
        return False, (stdout + stderr).strip()[-1000:] or "timeout"
    except (FileNotFoundError, OSError) as exc:
        return False, str(exc)


def collect_ros_probes(timeout: float = 4.0) -> dict[str, dict[str, Any]]:
    """Perform bounded ROS reads. No command here can publish robot motion."""
    commands = {
        "action": ["ros2", "action", "list"],
        "tf": ["ros2", "run", "tf2_ros", "tf2_echo", "map", "base_link"],
        "local_costmap": [
            "ros2", "topic", "echo", "/local_costmap/costmap_raw", "--once"],
        "global_costmap": [
            "ros2", "topic", "echo", "/global_costmap/costmap", "--once"],
        "costmap_bridge": [
            "systemctl", "is-active", "costmap-bridge.service"],
        "fastlio": [
            "systemctl", "is-active", "fastlio.service"],
    }
    with ThreadPoolExecutor(max_workers=len(commands)) as executor:
        futures = {
            name: executor.submit(_run_probe, command, timeout)
            for name, command in commands.items()
        }
        results = {name: future.result() for name, future in futures.items()}

    action_ok, action_output = results["action"]
    action_found = action_ok and "/navigate_to_pose" in action_output.splitlines()

    tf_ok, tf_output = results["tf"]
    # tf2_echo normally runs continuously, so timeout is expected after data.
    tf_found = "Translation:" in tf_output and "Rotation:" in tf_output

    costmap_ok, costmap_output = results["local_costmap"]
    costmap_found = costmap_ok and bool(costmap_output)
    global_costmap_ok, global_costmap_output = results["global_costmap"]
    global_costmap_found = global_costmap_ok and bool(global_costmap_output)
    bridge_ok, bridge_output = results["costmap_bridge"]
    fastlio_ok, fastlio_output = results["fastlio"]
    return {
        "fastlio_active": {
            "ok": fastlio_ok,
            "detail": fastlio_output or ("active" if fastlio_ok else "inactive"),
        },
        "map_to_base_link": {"ok": tf_found, "detail": tf_output or "no_tf"},
        "navigate_to_pose_action": {
            "ok": action_found, "detail": action_output or "action_not_listed"},
        "local_costmap_fresh": {
            "ok": costmap_found, "detail": costmap_output or "no_costmap_sample"},
        "global_costmap_fresh": {
            "ok": global_costmap_found,
            "detail": global_costmap_output or "no_global_costmap_sample",
        },
        "costmap_bridge_active": {
            "ok": bridge_ok,
            "detail": bridge_output or ("active" if bridge_ok else "inactive"),
        },
    }


def load_status(url: str, status_file: str | None, timeout: float) -> dict[str, Any]:
    if status_file:
        return json.loads(Path(status_file).read_text(encoding="utf-8"))
    with urllib.request.urlopen(url, timeout=timeout) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("status response must be a JSON object")
    return payload


def _write_atomic(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(payload, encoding="utf-8")
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read-only Panel/Nav2 preflight; sends no robot command")
    parser.add_argument("--url", default="http://127.0.0.1:8000/api/status")
    parser.add_argument("--status-file")
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("--skip-ros-probes", action="store_true")
    parser.add_argument("--json", action="store_true", dest="as_json")
    parser.add_argument("--output", help="atomically write the JSON receipt")
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 0.2 <= args.timeout <= 15.0:
        parser.error("--timeout must be in [0.2, 15.0]")
    try:
        status = load_status(args.url, args.status_file, args.timeout)
        probes = {} if args.skip_ros_probes else collect_ros_probes(args.timeout)
        report = evaluate_status(status, probes)
    except (OSError, ValueError, json.JSONDecodeError, urllib.error.URLError) as exc:
        report = {"ok": False, "read_only": True, "error": str(exc), "checks": []}

    json_payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        _write_atomic(Path(args.output), json_payload)
    if args.as_json:
        print(json_payload, end="")
    else:
        print("Nav2 preflight (READ ONLY)")
        for item in report.get("checks", []):
            marker = "PASS" if item["ok"] else "FAIL"
            print(f"[{marker}] {item['name']}: {item['detail']}")
            if item.get("recovery"):
                print(f"       recovery: {item['recovery']}")
        if report.get("error"):
            print(f"[FAIL] status: {report['error']}")
        print("READY" if report.get("ok") else "NOT READY")
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
