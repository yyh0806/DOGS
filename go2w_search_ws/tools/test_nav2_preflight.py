from pathlib import Path

import time

import nav2_preflight
from nav2_preflight import collect_ros_probes, evaluate_status


def healthy_status():
    return {
        "connected": True,
        "dog_state": "STOPPED",
        "localization": {
            "healthy": True,
            "reason": "ok",
            "age_sec": 0.05,
            "frame_id": "map",
            "child_frame_id": "base_link",
        },
        "navigation": {
            "ready": False,
            "activatable": True,
            "reason": "drive_session_parked",
            "sdk_ready": True,
            "nav_scan_fresh": True,
            "battery_soc": 82.0,
            "minimum_battery_soc": 20.0,
            "drive_fault": None,
            "sport_mode": 6,
            "wheel_dq": [0.01, -0.02, 0.0, 0.03],
            "drive_session": "parked",
        },
    }


def healthy_probes():
    return {
        "fastlio_active": True,
        "map_to_base_link": True,
        "navigate_to_pose_action": True,
        "local_costmap_fresh": True,
        "global_costmap_fresh": True,
        "costmap_bridge_active": True,
    }


def checks_by_name(report):
    return {item["name"]: item for item in report["checks"]}


def test_preflight_accepts_locked_zero_motion_ready_stack():
    report = evaluate_status(healthy_status(), healthy_probes())

    assert report["ok"] is True
    assert all(item["ok"] for item in report["checks"])


def test_preflight_rejects_stack_that_cannot_activate_from_parked():
    status = healthy_status()
    status["navigation"].update({
        "activatable": False,
        "reason": "nav_scan_stale",
    })

    report = evaluate_status(status, healthy_probes())

    assert checks_by_name(report)["navigation_gate"]["ok"] is False
    assert report["ok"] is False


def test_preflight_rejects_stale_fastlio_and_missing_nav2_chain():
    status = healthy_status()
    status["localization"].update({"healthy": False, "reason": "stale"})
    status["navigation"].update({
        "ready": False,
        "reason": "localization_stale",
    })
    probes = healthy_probes()
    probes.update({
        "fastlio_active": False,
        "map_to_base_link": False,
        "navigate_to_pose_action": False,
        "local_costmap_fresh": False,
        "global_costmap_fresh": False,
        "costmap_bridge_active": False,
    })

    report = evaluate_status(status, probes)
    checks = checks_by_name(report)

    assert report["ok"] is False
    assert checks["fastlio_localization"]["ok"] is False
    assert checks["map_to_base_link_tf"]["ok"] is False
    assert checks["navigate_to_pose_action"]["ok"] is False
    assert checks["local_costmap_fresh"]["ok"] is False
    assert checks["global_costmap_fresh"]["ok"] is False
    assert checks["costmap_bridge_active"]["ok"] is False


def test_preflight_rejects_wheel_only_localization_even_when_pose_is_fresh():
    probes = healthy_probes()
    probes["fastlio_active"] = {
        "ok": False,
        "detail": "inactive",
    }

    report = evaluate_status(healthy_status(), probes)

    assert report["ok"] is False
    check = checks_by_name(report)["fastlio_localization"]
    assert check["ok"] is False
    assert check["detail"]["service"] == "inactive"


def test_preflight_rejects_unlocked_or_moving_wheels_even_if_ready_flag_is_true():
    status = healthy_status()
    status["navigation"]["sport_mode"] = 1
    status["navigation"]["wheel_dq"] = [0.4, 0.3, -0.2, 0.5]

    report = evaluate_status(status, healthy_probes())
    checks = checks_by_name(report)

    assert report["ok"] is False
    assert checks["joint_lock_mode_6"]["ok"] is False
    assert checks["wheels_still"]["ok"] is False


def test_preflight_rejects_low_battery_fault_or_nonfinite_wheel_feedback():
    for mutation, failed_check in (
        (("battery_soc", 10.0), "battery"),
        (("drive_fault", "wheel_no_response"), "drive_fault_clear"),
        (("wheel_dq", [0.0, float("nan"), 0.0, 0.0]), "wheels_still"),
    ):
        status = healthy_status()
        status["navigation"][mutation[0]] = mutation[1]
        report = evaluate_status(status, healthy_probes())
        assert checks_by_name(report)[failed_check]["ok"] is False
        assert report["ok"] is False


def test_ros_probe_results_are_required_not_assumed():
    report = evaluate_status(healthy_status(), {})
    checks = checks_by_name(report)

    assert report["ok"] is False
    assert checks["map_to_base_link_tf"]["detail"] == "not_probed"
    assert checks["navigate_to_pose_action"]["detail"] == "not_probed"
    assert checks["local_costmap_fresh"]["detail"] == "not_probed"
    assert checks["global_costmap_fresh"]["detail"] == "not_probed"
    assert checks["costmap_bridge_active"]["detail"] == "not_probed"


def test_ros_probes_share_one_parallel_timeout_window(monkeypatch):
    commands = []

    def fake_run(command, timeout):
        commands.append(tuple(command))
        time.sleep(0.05)
        joined = " ".join(command)
        if "action list" in joined:
            return True, "/navigate_to_pose"
        if "tf2_echo" in joined:
            return False, "Translation: [0, 0, 0]\nRotation: [0, 0, 0, 1]"
        if "systemctl" in joined:
            return True, "active"
        return True, "sample"

    monkeypatch.setattr(nav2_preflight, "_run_probe", fake_run)
    started = time.monotonic()
    probes = collect_ros_probes(timeout=1.0)
    elapsed = time.monotonic() - started

    assert len(commands) == 6
    assert elapsed < 0.16
    assert all(item["ok"] for item in probes.values())


def test_angular_probe_is_explicit_bounded_and_preflight_gated():
    source = Path("tools/probe_angular_response.py").read_text(encoding="utf-8")

    assert 'parser.add_argument("--execute", action="store_true")' in source
    assert "if not args.execute:" in source
    assert "0.0 < abs(args.angular) <= 0.15" in source
    assert "evaluate_status(" in source
    assert "collect_ros_probes(" in source
    assert '"final_joint_lock":' in source


def test_direct_move_probe_reserves_scheduler_margin_and_always_stops():
    source = Path("tools/probe_angular_response.py").read_text(encoding="utf-8")

    assert 'parser.add_argument("--linear-x", type=float, default=0.0)' in source
    assert "0.0 < abs(args.linear_x) <= 0.20" in source
    assert "args.duration <= 0.2" in source
    assert "abs(args.linear_x) * args.duration <= 0.04" in source
    assert "remaining = args.duration - (time.monotonic() - started)" in source
    assert "timeout_sec=min(0.01, remaining)" in source
    assert "node.publish(args.angular, args.linear_x)" in source
    assert "node.publish(0.0, 0.0)" in source
    assert '"command_linear_x": args.linear_x' in source


def test_nav2_deployment_installs_read_only_preflight_and_opt_in_probe():
    source = Path("docker/deploy_nav2_bprime.sh").read_text(encoding="utf-8")

    assert '"$WIN_WS/tools/nav2_preflight.py"' in source
    assert '"$WIN_WS/tools/probe_angular_response.py"' in source
    assert "mkdir -p tools" in source
    assert "cp /tmp/bprime/nav2_preflight.py tools/" in source
    assert "cp /tmp/bprime/probe_angular_response.py tools/" in source
