import ast
from pathlib import Path
import subprocess
import sys


NODE = Path(__file__).resolve().parents[1] / "go2w_bridge" / "nx_motion_node.py"
GATEWAY = Path(__file__).resolve().parents[1] / "go2w_bridge" / "nx_sport_gateway.py"


def parse_node():
    source = NODE.read_text(encoding="utf-8")
    return source, ast.parse(source)


def test_node_has_one_motion_machine_and_no_legacy_state_owners():
    source, tree = parse_node()
    classes = {
        item.name for item in tree.body if isinstance(item, ast.ClassDef)}

    assert "NxMotionNode" in classes
    assert "Go2WStateModel" not in classes
    assert "DriveSessionModel" not in classes
    assert "DriveStateModel" not in classes
    assert "Go2WMotionMachine(" in source
    assert source.count("Go2WMotionMachine(") == 1
    assert "STAND_UNCONFIRMED" not in source
    assert "BALANCE_UNCONFIRMED" not in source


def test_ros_callbacks_only_enqueue_actor_events():
    _, tree = parse_node()
    node = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "NxMotionNode")
    methods = {
        item.name: ast.unparse(item)
        for item in node.body if isinstance(item, ast.FunctionDef)
    }
    for name in (
        "_on_drive_feedback", "_on_cmd_vel", "_on_nav_cmd_vel",
        "_on_motion_session", "_on_cmd_pose", "_on_scan",
    ):
        body = methods[name]
        assert "self._enqueue(" in body, name
        assert "self._controller." not in body, name
        assert "self._sport" not in body, name
        assert ".execute(" not in body, name


def test_actor_starts_before_ros_command_subscriptions():
    source, tree = parse_node()
    init = next(
        item for item in ast.walk(tree)
        if isinstance(item, ast.FunctionDef) and item.name == "__init__")
    body = ast.unparse(init)

    assert "self._actor_thread.start()" in body
    assert body.index("self._actor_thread.start()") < body.index(
        "self.create_subscription(Twist, '/cmd_vel'")
    assert "SportGatewayClient(" in source
    assert "UnitreeSportAdapter(" not in source
    assert "SportClient(" not in source
    assert "MotionSwitcherClient(" not in source


def test_only_stable_gateway_constructs_the_leased_sport_client():
    node_source = NODE.read_text(encoding="utf-8")
    gateway_source = GATEWAY.read_text(encoding="utf-8")

    assert "SportClient(enableLease=True)" not in node_source
    assert gateway_source.count("SportClient(enableLease=True)") == 1


def test_motion_policy_retries_until_gateway_mode_check_is_healthy():
    source, tree = parse_node()
    initialize = next(
        item for item in ast.walk(tree)
        if isinstance(item, ast.FunctionDef)
        and item.name == "_try_initialize_sdk"
    )
    body = ast.unparse(initialize)

    assert "initialized.code != 0" in body
    assert "initialized.motion_service != 'ai-w'" in body
    assert body.index("initialized.code != 0") < body.index(
        "self._controller.attach_adapter")
    assert "adapter.close()" in body


def test_startup_and_shutdown_have_no_automatic_unload_command():
    source, tree = parse_node()
    assert "Damp" not in source
    assert "RecoveryStand" not in source
    destroy = next(
        item for item in ast.walk(tree)
        if isinstance(item, ast.FunctionDef) and item.name == "destroy_node")
    destroy_source = ast.unparse(destroy)
    assert "self._enqueue('shutdown'" in destroy_source
    assert "StandUp" not in destroy_source
    assert "BalanceStand" not in destroy_source


def test_shutdown_event_is_consumed_before_actor_exits():
    _, tree = parse_node()
    actor = next(
        item for item in ast.walk(tree)
        if isinstance(item, ast.FunctionDef) and item.name == "_actor_loop")
    actor_source = ast.unparse(actor)

    assert "while rclpy.ok():" in actor_source
    assert "and (not self._shutdown_enqueued)" not in actor_source
    assert "if kind == 'shutdown':" in actor_source
    assert "self._controller.shutdown()" in actor_source


def test_pure_motion_modules_support_nx_direct_file_deployment():
    module_dir = str(NODE.parent)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; "
                f"sys.path.insert(0, {module_dir!r}); "
                "import motion_types, motion_machine, motion_protocol, "
                "motion_safety, motion_controller, unitree_sport_adapter, "
                "sport_gateway_protocol, sport_gateway_client"
            ),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
