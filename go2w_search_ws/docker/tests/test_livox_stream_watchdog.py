import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WATCHDOG = ROOT / "tools" / "livox_stream_watchdog.py"
SERVICE = ROOT / "docker" / "livox-mid360-watchdog.service"
DEPLOY = ROOT / "docker" / "deploy_nav2_bprime.sh"


def _load_watchdog_module():
    spec = importlib.util.spec_from_file_location("livox_stream_watchdog", WATCHDOG)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_watchdog_requires_consecutive_stale_polls_after_startup_grace():
    module = _load_watchdog_module()
    policy = module.StreamWatchdogPolicy(
        startup_grace_sec=20.0,
        stale_after_sec=3.0,
        failures_before_restart=3,
        restart_cooldown_sec=20.0,
        started_at=100.0,
    )

    assert policy.should_restart(119.9) is False
    assert policy.should_restart(120.0) is False
    assert policy.should_restart(121.0) is False
    assert policy.should_restart(122.0) is True


def test_fresh_stream_resets_failure_streak_and_stale_stream_restarts_once():
    module = _load_watchdog_module()
    policy = module.StreamWatchdogPolicy(
        startup_grace_sec=2.0,
        stale_after_sec=3.0,
        failures_before_restart=2,
        restart_cooldown_sec=10.0,
        started_at=0.0,
    )

    policy.note_message(2.0)
    assert policy.should_restart(4.9) is False
    assert policy.should_restart(5.1) is False
    policy.note_message(5.2)
    assert policy.should_restart(8.3) is False
    assert policy.should_restart(9.3) is True

    policy.note_restart(9.3)
    assert policy.should_restart(10.0) is False
    assert policy.should_restart(19.2) is False
    assert policy.should_restart(19.3) is False
    assert policy.should_restart(20.3) is True


def test_watchdog_service_is_separate_from_motion_and_can_restart_livox():
    service = SERVICE.read_text(encoding="utf-8")

    assert "After=livox-mid360-driver.service" in service
    assert "Wants=livox-mid360-driver.service" in service
    assert "Requires=livox-mid360-driver.service" not in service
    assert "User=root" in service
    assert (
        "/home/nx/go2w/current/payload/tools/livox_stream_watchdog.py"
        in service
    )
    assert (
        "FASTRTPS_DEFAULT_PROFILES_FILE="
        "/home/nx/go2w/current/payload/docker/fastdds_udp.xml"
        in service
    )
    assert "Restart=always" in service
    assert "go2w-motion" not in service


def test_nav2_deploy_installs_enables_and_verifies_watchdog():
    deploy = DEPLOY.read_text(encoding="utf-8")

    assert '"$WIN_WS/tools/livox_stream_watchdog.py"' in deploy
    assert '"$WIN_WS/docker/livox-mid360-watchdog.service"' in deploy
    assert (
        "sudo -n install -D -o root -g root -m 755 "
        "/tmp/bprime/livox_stream_watchdog.py "
        "/usr/local/lib/go2w/livox_stream_watchdog.py"
    ) in deploy
    assert (
        "sudo -n install -o root -g root -m 644 "
        "/tmp/bprime/livox-mid360-watchdog.service "
        "/etc/systemd/system/livox-mid360-watchdog.service"
    ) in deploy
    assert "sudo -n systemctl enable livox-mid360-watchdog.service" in deploy
    assert "sudo -n systemctl restart livox-mid360-watchdog.service" in deploy
    assert (
        "sudo -n cmp -s /tmp/bprime/livox_stream_watchdog.py "
        "/usr/local/lib/go2w/livox_stream_watchdog.py"
    ) in deploy
    assert (
        "sudo -n cmp -s /tmp/bprime/livox-mid360-watchdog.service "
        "/etc/systemd/system/livox-mid360-watchdog.service"
    ) in deploy
    assert "systemctl is-active --quiet livox-mid360-watchdog.service" in deploy
