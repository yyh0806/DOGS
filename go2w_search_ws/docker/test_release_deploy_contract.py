import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


def text(name):
    return (ROOT / "docker" / name).read_text(encoding="utf-8")


def _bash_path():
    candidates = [
        os.environ.get("GO2W_TEST_BASH"),
        r"C:\Program Files\Git\bin\bash.exe",
        shutil.which("bash"),
    ]
    return next((value for value in candidates if value and Path(value).is_file()), None)


def _heredoc(source, marker):
    opening = f"<<'{marker}'\n"
    start = source.index(opening) + len(opening)
    return source[start:source.index(f"\n{marker}\n", start)] + "\n"


@pytest.mark.parametrize("marker", ("PREFLIGHT", "REMOTE"))
def test_remote_shell_programs_parse_as_bash(marker):
    bash = _bash_path()
    if bash is None:
        pytest.skip("bash is unavailable")
    result = subprocess.run(
        [bash, "-n"],
        input=_heredoc(text("deploy_release.sh"), marker),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_remote_ros_setup_is_scoped_outside_nounset_mode():
    remote = _heredoc(text("deploy_release.sh"), "REMOTE")

    assert "source_ros() {" in remote
    assert "set +u" in remote
    assert "set -u" in remote
    assert remote.count("source /opt/ros/humble/setup.bash") == 1
    assert remote.count("source_ros") >= 3


def test_web_service_uses_persistent_mission_evidence_root():
    service = text("go2w-web.service")
    wrapper = (ROOT / "web" / "start_go2w_web.sh").read_text(
        encoding="utf-8")

    assert "GO2W_MISSION_ROOT=/home/nx/go2w/missions" in service
    assert 'mkdir -p "$GO2W_MISSION_ROOT"' in wrapper


def test_web_service_declares_c13_nominal_annotation_calibration():
    service = text("go2w-web.service")

    assert "GO2W_CAMERA_HFOV_C13_VIS_DEG=77.4" in service
    assert "GO2W_CAMERA_HFOV=77.4" in service
    assert "GO2W_CAMERA_YAW_OFFSET_C13_VIS_DEG=0.0" in service
    assert "GO2W_CAMERA_CALIBRATION_C13_VIS=nominal_centered" in service


def test_lifecycle_probe_has_hard_kill_deadline():
    bringup = text("bringup_slam_nav2.sh")

    assert 'timeout --kill-after=2 "$outer_timeout" python3' in bringup


def test_remote_transaction_derives_service_list_without_space_bearing_ssh_argument():
    source = text("deploy_release.sh")
    invocation = source[source.index('"${SSH[@]}" "$NX_USER@$NX_HOST" bash -s --'):]
    invocation = invocation[:invocation.index("<<'REMOTE'")]
    remote = _heredoc(source, "REMOTE")

    assert '"$services"' not in invocation
    assert 'bootstrap_sport_gateway="${4:-0}"' in remote
    assert 'hardware_env="$5"' in remote
    assert 'control_env="${6:-}"' in remote
    assert 'case "$subsystem" in' in remote
    assert (
        'all) services="go2w-safety-observer.service go2w-motion.service '
        'go2w-sensor.service ' in remote
    )


def test_post_switch_guard_failures_trigger_err_trap_instead_of_explicit_exit():
    remote = _heredoc(text("deploy_release.sh"), "REMOTE")
    switch = remote.index('mv -Tf "$current.next" "$current"')
    commit = remote.index("trap - ERR", switch)
    post_switch = remote[switch:commit]

    assert "detected hardware environment is missing" in post_switch
    assert "exit 1" not in post_switch
    assert "false" in post_switch


def test_rollback_restores_predeployment_active_units_without_previous_release():
    remote = _heredoc(text("deploy_release.sh"), "REMOTE")
    rollback = remote[remote.index("rollback() {"):remote.index("trap rollback ERR")]

    assert "backup_active_state" in remote
    assert "restore_active_state" in remote
    assert 'systemctl is-active "$unit"' in remote
    assert "restore_active_state || true" in rollback
    assert rollback.index("restore_system_state") < rollback.index(
        "restore_active_state")
    assert 'if [ -n "$previous_target" ]' not in rollback[
        rollback.index("restore_system_state"):]


def test_nav_bringup_and_deploy_probes_do_not_use_ros2cli_daemon():
    bringup = text("bringup_slam_nav2.sh")
    deploy = text("deploy_release.sh")
    command_lines = [
        line for line in bringup.splitlines()
        if ("ros2 topic echo" in line or "ros2 lifecycle " in line)
        and not line.strip().startswith('echo "')
    ]

    assert "export ROS2CLI_NO_DAEMON=1" in bringup
    assert "export ROS2CLI_NO_DAEMON=1" in deploy
    assert command_lines
    assert all("--no-daemon" in line for line in command_lines)


def test_release_builder_writes_hash_manifest_and_versioned_artifact():
    source = text("build_release.sh")
    for contract in (
        "release_id", "subsystem", "required_services", "sha256",
        "verification_command", "payload",
    ):
        assert contract in source
    assert "-dirty" in source
    assert '"web": "python3 -m compileall -q ai web"' in source
    assert '"nav": "python3 -m compileall -q src tools' in source
    assert '"all": "python3 -m compileall -q ai web src tools"' in source


def test_release_builder_packages_the_strict_artifact_verifier():
    source = text("build_release.sh")
    assert 'copy_path "tools/verify_release_artifact.py"' in source
    assert 'copy_path "tools/nx_release_probe.py"' in source
    for tool in (
        "diag_sport_requests.py",
        "diag_sport_state.py",
        "diag_wheel_dq.py",
        "nav2_preflight.py",
        "nav2_benchmark.py",
        "capture_map_pose.py",
        "perception_preflight.py",
    ):
        assert f'copy_path "tools/{tool}"' in source
    for module in (
        "__init__.py",
        "config.py",
        "detector.py",
        "locate_anything.py",
        "tracker.py",
        "vlm.py",
    ):
        assert f'copy_path "ai/{module}"' in source


def test_release_builder_packages_stable_gateway_runtime_and_unit():
    source = text("build_release.sh")
    for name in (
        "sport_gateway_protocol.py",
        "sport_gateway_server.py",
        "sport_gateway_client.py",
        "safety_event_recorder.py",
        "nx_safety_observer.py",
        "nx_sport_gateway.py",
    ):
        assert f" {name}" in source or f"{name} " in source
    assert 'copy_path "docker/go2w-sport-gateway.service"' in source
    assert 'copy_path "docker/go2w-safety-observer.service"' in source


def test_motion_release_manages_observer_without_restarting_stable_gateway():
    build = text("build_release.sh")
    deploy = text("deploy_release.sh")

    assert (
        'motion) REQUIRED_SERVICES="go2w-sport-gateway.service,'
        'go2w-safety-observer.service,go2w-motion.service"' in build
    )
    assert (
        'motion) services="go2w-safety-observer.service '
        'go2w-motion.service"' in deploy
    )
    assert 'restart_units="$services"' in deploy
    assert 'restart_units="go2w-sport-gateway.service' not in deploy


def test_packaged_unitree_diagnostics_use_deployment_interface_override():
    for name in ("diag_sport_state.py", "diag_wheel_dq.py"):
        source = (ROOT / "tools" / name).read_text(encoding="utf-8")
        assert 'os.environ.get("DOG_INTERFACE"' in source
        assert 'factory.Init(0, DOG_INTERFACE)' in source


def test_deployer_verifies_before_atomic_symlink_switch_and_rolls_back():
    source = text("deploy_release.sh")
    verify = source.index("sha256")
    switch = source.index('ln -sfn "$release_dir"')
    assert verify < switch
    assert "/home/nx/go2w/releases" in source
    assert "previous_target" in source
    assert "rollback" in source


def test_colcon_build_uses_final_release_prefix_before_current_switch():
    source = text("deploy_release.sh")
    finalize_dir = source.index('mv "$staging" "$release_dir"')
    build = source.index('cd "$release_dir/payload"', finalize_dir)
    colcon = source.index(
        "colcon build --packages-select go2w_nav --symlink-install", build)
    switch = source.index('ln -sfn "$release_dir" "$current.next"', colcon)

    assert finalize_dir < build < colcon < switch
    assert not (
        source.find('cd "$staging/payload"', 0, finalize_dir) >= 0
        and source.find(
            "colcon build --packages-select go2w_nav --symlink-install",
            source.find('cd "$staging/payload"', 0, finalize_dir),
            finalize_dir,
        ) >= 0
    )


def test_existing_release_payload_is_rehashed_before_reuse():
    source = text("deploy_release.sh")
    collision = source.index('if [ -d "$release_dir" ]; then')
    discard_staging = source.index('rm -rf "$staging"', collision)
    block = source[collision:discard_staging]

    assert "hashlib.sha256" in block
    assert 'incoming["sha256"]' in block
    assert "existing release payload mismatch" in block


def test_deployer_validates_locally_before_upload_and_rejects_links_before_extract():
    source = text("deploy_release.sh")
    assert source.index("tools/verify_release_artifact.py") < source.index(
        '"${SCP[@]}" "$ARTIFACT"')
    remote_guard = source.index("unsupported archive member")
    extraction = source.index("archive.extractall(destination)")
    assert remote_guard < extraction
    assert source.count("tools/verify_release_artifact.py") >= 2


def test_motion_restart_requires_explicit_opt_in():
    source = text("deploy_release.sh")
    assert "--allow-motion-restart" in source
    assert "motion_restart_not_authorized" in source


def test_gateway_bootstrap_requires_explicit_supported_maintenance_opt_in():
    source = text("deploy_release.sh")

    assert "--bootstrap-sport-gateway" in source
    assert "gateway_bootstrap_requires_motion_restart_authorization" in source
    assert "gateway_bootstrap_required" in source
    assert "gateway_bootstrapped=1" in source
    assert "gateway_bootstrap_failed_zero_hold" in source


def test_required_gateway_bootstrap_argument_precedes_optional_livox_override():
    source = text("deploy_release.sh")
    invocation = source[
        source.index('bash -s -- "$subsystem"'):
        source.index("<<'PREFLIGHT'")
    ]

    assert invocation.index('"$BOOTSTRAP_SPORT_GATEWAY"') < invocation.index(
        '"$LIVOX_INTERFACE_OVERRIDE"')
    assert 'bootstrap_sport_gateway="${4:-0}"' in source
    assert 'livox_interface_override="${5:-}"' in source


def test_required_transaction_arguments_precede_optional_control_environment():
    source = text("deploy_release.sh")
    start = source.index(
        '"${SSH[@]}" "$NX_USER@$NX_HOST" bash -s --',
        source.index('remote_hardware_env='),
    )
    invocation = source[start:source.index("<<'REMOTE'", start)]

    assert invocation.index('"$BOOTSTRAP_SPORT_GATEWAY"') < invocation.index(
        '"$remote_control_env"')
    assert invocation.index('"$remote_hardware_env"') < invocation.index(
        '"$remote_control_env"')
    assert 'bootstrap_sport_gateway="${4:-0}"' in source
    assert 'hardware_env="$5"' in source
    assert 'control_env="${6:-}"' in source


def test_gateway_bootstrap_requires_read_only_parked_state_gate_before_switch():
    build = text("build_release.sh")
    deploy = text("deploy_release.sh")

    assert 'copy_path "tools/sport_gateway_bootstrap_preflight.py"' in build
    check = deploy.index("tools/sport_gateway_bootstrap_preflight.py")
    switch = deploy.index("backup_system_state", check)
    assert "gateway_bootstrap_state_not_safe" in deploy
    assert deploy.count("tools/sport_gateway_bootstrap_preflight.py") == 2
    assert check < switch


def test_remote_control_token_is_private_before_root_install():
    source = text("deploy_release.sh")
    upload = source.index('"${SCP[@]}" "$TMP/control.env"')
    protect = source.index('chmod 600 \'$remote_control_env\'')
    install = source.index('install -o root -g root -m 0600')
    assert upload < protect < install


def test_deployer_rejects_weak_local_and_existing_nx_tokens():
    source = text("deploy_release.sh")
    local_start = source.index('token="$(sed')
    local_end = source.index("preflight_output=", local_start)
    local = source[local_start:local_end]
    preflight = _heredoc(source, "PREFLIGHT")

    assert '${#token}' in local
    assert "control token must contain at least 32" in local
    assert "existing_control_token" in preflight
    assert '${#existing_control_token}' in preflight
    assert "existing control token is missing or weak" in preflight


def test_first_deploy_can_explicitly_generate_a_non_overwriting_local_token():
    source = text("deploy_release.sh")
    verify = source.index("tools/verify_release_artifact.py")
    generate = source.index("tools/generate_control_token.py", verify)
    preflight = source.index("preflight_output=", generate)

    assert "--generate-control-token-file" in source
    assert "--control-token-file and --generate-control-token-file are mutually exclusive" in source
    assert verify < generate < preflight


def test_read_only_remote_preflight_runs_before_any_upload():
    source = text("deploy_release.sh")
    start = source.index("remote_preflight() {")
    end = source.index("\n}\n\nremote_artifact", start) + 3
    preflight = source[start:end]

    assert end < source.index('"${SCP[@]}" "$ARTIFACT"')
    for required in (
        "sudo -n true",
        "/opt/ros/humble/setup.bash",
        "unitree_sdk2py",
        "available_kb",
        "192.168.123.161",
        "ip route get",
        "GO2W_DOG_INTERFACE=",
        "GO2W_YOLO_MODEL_PATH=",
        "ip route get 192.168.1.160",
        "GO2W_LIVOX_INTERFACE=",
        "import numpy, cv2, torch, ultralytics",
        "yolov8x-worldv2.pt",
        "/etc/go2w/control.env",
        "control_token_supplied",
    ):
        assert required in preflight
    assert "systemctl restart" not in preflight
    assert "systemctl stop" not in preflight


def test_deployer_can_bind_ssh_and_scp_to_the_physical_lan_interface():
    source = text("deploy_release.sh")
    executable_lines = [
        line.strip() for line in source.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]

    assert 'NX_BIND_ADDRESS="${NX_BIND_ADDRESS:-}"' in source
    assert 'SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8)' in source
    assert 'SCP=(scp -q -o BatchMode=yes -o ConnectTimeout=8)' in source
    assert 'SSH+=(-b "$NX_BIND_ADDRESS")' in source
    assert 'SCP+=(-o "BindAddress=$NX_BIND_ADDRESS")' in source
    assert not any(line.startswith("ssh ") for line in executable_lines)
    assert not any(line.startswith("scp ") for line in executable_lines)


def test_detected_hardware_environment_is_installed_before_service_restart():
    source = text("deploy_release.sh")
    preflight = source.index("remote_preflight")
    hardware_upload = source.index('"${SCP[@]}" "$TMP/hardware.env"')
    hardware_install = source.index(
        'install -o root -g root -m 0644 "$hardware_env" /etc/go2w/hardware.env')
    restart = source.index('systemctl restart "$service"', hardware_install)

    assert preflight < hardware_upload < hardware_install < restart
    assert "DOG_INTERFACE=%s" in source
    assert "GO2W_PUBLIC_IP=%s" in source
    assert "GO2W_PANEL_ORIGINS=%s" in source
    assert "GO2W_YOLO_MODEL=%s" in source
    assert "LIVOX_INTERFACE=%s" in source


def test_first_install_rollback_stops_new_services_only_after_switch():
    source = text("deploy_release.sh")

    assert "switched=0" in source
    assert "switched=1" in source
    assert 'if [ "$switched" -eq 1 ]; then' in source
    assert 'systemctl stop "$unit"' in source


def test_nav_rollback_stops_transitive_runtime_before_restoring_files():
    source = text("deploy_release.sh")
    rollback = source.index("rollback() {")
    stop = source.index('systemctl stop "$unit"', rollback)
    restore = source.index("restore_system_state", stop)

    assert "rollback_units=" in source
    for unit in (
        "go2w-slam-nav.service",
        "costmap-bridge.service",
        "livox-mid360-watchdog.service",
        "livox-mid360-driver.service",
        "livox-mid360-net.service",
    ):
        assert unit in source[source.index("rollback_units="):rollback]
    assert stop < restore


def test_nav_service_always_cleans_long_lived_health_supervisor():
    service = (ROOT / "docker" / "go2w-slam-nav.service").read_text(encoding="utf-8")
    assert "ExecStopPost=" in service
    assert "nav-health-supervisor.service" in service


def test_nav_transaction_never_restores_or_restarts_motion_boundary():
    source = text("deploy_release.sh")
    remote = source.split("<<'REMOTE'", 1)[1]
    nav_branch = remote.split('if [ "$subsystem" = "nav" ]; then', 1)[1].split(
        "\nfi", 1
    )[0]
    for forbidden in (
        "go2w-sport-gateway.service",
        "go2w-safety-observer.service",
        "go2w-motion.service",
    ):
        assert forbidden not in nav_branch
    assert 'managed_units="go2w-sensor.service go2w-slam-nav.service' in nav_branch
    assert 'active_state_units="go2w-slam-nav.service' in nav_branch


def test_partial_release_preserves_running_cohort_release_id():
    source = text("deploy_release.sh")
    write = "printf 'GO2W_RELEASE_ID=%s\\n' \"$release_id\""
    location = source.index(write)
    guard = source.rfind(
        'if [ "$subsystem" = "all" ] || ! sudo -n test -s /etc/go2w/release.env; then',
        0, location,
    )
    assert guard != -1 and location - guard < 200
    assert source.index("\nfi", location) > location


def test_atomic_rollback_restores_environment_and_systemd_units():
    source = text("deploy_release.sh")
    backup = source.index("backup_system_state")
    switch = source.index('ln -sfn "$release_dir" "$current.next"')

    assert backup < switch
    for path in (
        "/etc/go2w/release.env",
        "/etc/go2w/hardware.env",
        "/etc/go2w/control.env",
    ):
        assert path in source
    for unit in (
        "go2w-motion.service",
        "go2w-web.service",
        "go2w-slam-nav.service",
        "go2w-sensor.service",
        "costmap-bridge.service",
        "livox-mid360-net.service",
        "livox-mid360-driver.service",
        "livox-mid360-watchdog.service",
    ):
        assert unit in source
    assert '"/etc/systemd/system/$unit"' in source
    assert "restore_system_state" in source
    assert "backup_enable_state" in source
    assert "restore_enable_state" in source
    assert 'sudo -n systemctl daemon-reload' in source
    assert 'sudo -n rm -rf "$system_backup"' in source


def test_successful_deploy_enables_manifest_services_before_commit():
    source = text("deploy_release.sh")
    enable = source.index("sudo -n systemctl enable $enable_services")
    commit = source.index("trap - ERR", enable)

    assert enable < commit
    assert 'sudo -n systemctl is-enabled "$unit"' in source
    assert 'sudo -n systemctl disable "$unit"' in source
    assert (
        'enable_services="go2w-sport-gateway.service '
        'go2w-safety-observer.service go2w-motion.service go2w-web.service '
        'go2w-sensor.service go2w-slam-nav.service"' in source
    )
    assert 'systemctl disable go2w-sensor.service' not in source


def test_atomic_release_installs_costmap_companion_from_current_payload():
    build = text("build_release.sh")
    deploy = text("deploy_release.sh")

    assert 'copy_path "docker/costmap-bridge.service"' in build
    assert (
        'managed_units="go2w-sport-gateway.service '
        'go2w-safety-observer.service go2w-motion.service go2w-web.service '
        'go2w-slam-nav.service go2w-sensor.service costmap-bridge.service '
        'livox-mid360-net.service livox-mid360-driver.service '
        'livox-mid360-watchdog.service"'
        in deploy
    )
    assert deploy.count("for unit in $managed_units") >= 3


def test_full_deploy_never_restarts_active_sport_gateway():
    source = text("deploy_release.sh")
    restart_block = source[
        source.index("for service in $restart_units"):
        source.index("done", source.index("for service in $restart_units")) + 4
    ]

    assert 'gateway_was_active="$(sudo -n systemctl is-active' in source
    assert "go2w-sport-gateway.service" not in restart_block
    assert "test -S /run/go2w-sport-gateway/sport.sock" in source


def test_motion_service_requires_stable_gateway():
    gateway = text("go2w-sport-gateway.service")
    motion = text("go2w-motion.service")

    assert "RuntimeDirectory=go2w-sport-gateway" in gateway
    assert "Restart=always" in gateway
    assert "Before=go2w-motion.service" in gateway
    assert "Requires=go2w-sport-gateway.service" in motion
    assert "After=network-online.target go2w-sport-gateway.service" in motion
    assert "GO2W_SPORT_GATEWAY_SOCKET=/run/go2w-sport-gateway/sport.sock" in motion


def test_atomic_release_contains_mid360_boot_runtime():
    build = text("build_release.sh")
    slam_unit = text("go2w-slam-nav.service")

    for relative in (
        "docker/livox-mid360-net.service",
        "docker/livox-mid360-driver.service",
        "docker/livox-mid360-watchdog.service",
        "tools/livox_stream_watchdog.py",
    ):
        assert f'copy_path "{relative}"' in build
    assert "Wants=livox-mid360-watchdog.service" in slam_unit


def test_nav_and_web_restart_rules_never_include_motion():
    source = text("deploy_release.sh")
    assert 'web) services="go2w-web.service"' in source
    assert 'nav) services="go2w-sensor.service go2w-slam-nav.service"' in source
    assert 'sensor) services="go2w-sensor.service"' in source


def test_full_restart_keeps_sensor_feedback_and_single_nav_odom_owner():
    deploy = text("deploy_release.sh")
    service_line = next(
        line for line in deploy.splitlines()
        if line.strip().startswith("all) services="))
    assert service_line.index("go2w-motion.service") < service_line.index(
        "go2w-sensor.service")
    assert service_line.index("go2w-sensor.service") < service_line.index(
        "go2w-web.service")
    assert service_line.rstrip().endswith("go2w-slam-nav.service\" ;;")

    nav_unit = text("go2w-slam-nav.service")
    sensor_unit = text("go2w-sensor.service")
    assert "Wants=go2w-sensor.service" in nav_unit
    assert "After=go2w-sensor.service" in nav_unit
    assert "Conflicts=go2w-sensor.service" not in nav_unit
    assert "publish_odom_tf:=false" in sensor_unit
    assert "odom_topic:=/wheel_odom" in sensor_unit

    bringup = text("bringup_slam_nav2.sh")
    assert "sudo systemctl stop go2w-sensor.service" not in bringup
    assert "start_transient wheel-odom" not in bringup


def test_persistent_wheel_odom_inherits_commissioned_dog_interface_and_release():
    nav_unit = text("go2w-slam-nav.service")
    sensor_unit = text("go2w-sensor.service")

    assert "EnvironmentFile=-/etc/go2w/hardware.env" in nav_unit
    assert "EnvironmentFile=-/etc/go2w/hardware.env" in sensor_unit
    assert "EnvironmentFile=-/etc/go2w/release.env" in sensor_unit


def test_nav_transient_units_do_not_depend_on_login_user_environment():
    bringup = text("bringup_slam_nav2.sh")

    assert 'RUN_USER="$(id -un)"' in bringup
    assert '-p "User=$RUN_USER"' in bringup
    assert '-p "User=$USER"' not in bringup


def test_nav_restart_applies_current_mid360_units_before_nav():
    source = text("deploy_release.sh")
    assert (
        'restart_units="livox-mid360-net.service '
        'livox-mid360-driver.service livox-mid360-watchdog.service '
        'go2w-sensor.service go2w-slam-nav.service"' in source
    )
    assert (
        'restart_units="go2w-safety-observer.service go2w-motion.service '
        'go2w-sensor.service '
        'go2w-web.service livox-mid360-net.service '
        'livox-mid360-driver.service livox-mid360-watchdog.service '
        'go2w-slam-nav.service"' in source
    )
    assert "for service in $restart_units" in source


def test_dog_facing_services_use_detected_interface_from_hardware_env():
    for name in (
        "go2w-sport-gateway.service",
        "go2w-motion.service",
        "go2w-web.service",
        "go2w-sensor.service",
    ):
        source = text(name)
        lines = [line.strip() for line in source.splitlines()]
        assert "EnvironmentFile=-/etc/go2w/hardware.env" in lines
        assert "DOG_INTERFACE" in source
        assert "/sys/class/net/enxc8a362616c4c/operstate" not in source
        assert "ip link show enxc8a362616c4c" not in source


def test_web_detected_hardware_overrides_bundled_defaults():
    source = text("go2w-web.service")
    hardware = source.index("\nEnvironmentFile=-/etc/go2w/hardware.env\n")

    assert hardware > source.index("\nEnvironment=DOG_INTERFACE=")
    assert hardware > source.index("\nEnvironment=GO2W_PUBLIC_IP=")
    assert hardware > source.index("\nEnvironment=GO2W_YOLO_MODEL=")


def test_full_deploy_requires_read_only_release_and_safe_park_evidence():
    source = text("deploy_release.sh")
    restart = source.index('systemctl restart "$service"')
    probe = source.index('tools/nx_release_probe.py', restart)
    success = source.index("trap - ERR", probe)
    guard = source.rfind('if [ "$subsystem" = "all" ]', restart, probe)

    assert restart < guard < probe < success
    block = source[guard:success]
    assert "--require-sdk-ready" in block
    assert 'validation/${release_id}-deploy.json' in block
    assert "tools/nav2_preflight.py" in block
    assert 'validation/${release_id}-nav2-preflight.json' in block
    assert "tools/perception_preflight.py" in block
    assert 'validation/${release_id}-perception-preflight.json' in block
    assert "for nav_attempt in 1 2 3" in block
    assert 'nav_preflight_ok=1' in block


def test_legacy_nav_deploy_does_not_restart_or_copy_motion_runtime():
    source = text("deploy_nav2_bprime.sh")
    assert "systemctl restart go2w-motion" not in source
    assert "nx_motion_node.py" not in source


def test_legacy_entrypoints_forward_to_atomic_release_tools_before_old_body():
    expected_subsystem = {
        "deploy_nx.sh": "motion",
        "deploy_nx_web.sh": "web",
        "deploy_nav2_bprime.sh": "nav",
    }
    for name, subsystem in expected_subsystem.items():
        source = text(name)
        marker = "# LEGACY IMPLEMENTATION BELOW IS UNREACHABLE"
        assert marker in source
        wrapper, _legacy = source.split(marker, 1)
        assert f'build_release.sh" {subsystem}' in wrapper
        assert 'deploy_release.sh" "$artifact" "$@"' in wrapper
