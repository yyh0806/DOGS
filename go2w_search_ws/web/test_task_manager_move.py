"""Task 4 静态契约: nx_web_server.py 含 move_relative 集成点.

动态执行测试 (mock TaskManager 跑 _execute_move_relative) 依赖完整 ROS 栈
实例化 TaskManager, Windows 无 ROS 环境不可靠。集成契约用源码断言验证,
动态端到端验收在 NX 部署后执行 (见 plan §部署与验收).
"""

import ast
from pathlib import Path
import textwrap

import pytest

SRC = Path(__file__).resolve().parent / "nx_web_server.py"


@pytest.fixture(scope="module")
def source() -> str:
    return SRC.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def source_tree(source):
    return ast.parse(source)


def method_source(source, source_tree, class_name, method_name):
    cls = next(node for node in source_tree.body
               if isinstance(node, ast.ClassDef) and node.name == class_name)
    method = next(node for node in cls.body
                  if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                  and node.name == method_name)
    return ast.get_source_segment(source, method)


def compares_direction(node, expected):
    return (
        isinstance(node, ast.Compare)
        and isinstance(node.left, ast.Name)
        and node.left.id == "direction"
        and len(node.ops) == 1
        and isinstance(node.ops[0], ast.Eq)
        and len(node.comparators) == 1
        and isinstance(node.comparators[0], ast.Constant)
        and node.comparators[0].value == expected
    )


def test_source_dispatches_move_relative(source):
    """dispatch 链含 move_relative 分支."""
    assert 'task.type == "move_relative"' in source
    assert "self._execute_move_relative(task)" in source


def test_source_has_set_point_nav_method(source):
    """TaskManager 暴露 set_point_nav 注入器."""
    assert "def set_point_nav(self, port):" in source
    assert "self._point_nav = port" in source


def test_source_admit_branches_on_task_type(source):
    """_admit_command_result 按 first_type 分流到 canonicalize_move_tasks."""
    assert 'first_type == "move_relative"' in source
    assert "canonicalize_move_tasks(tasks)" in source


def test_source_imports_move_executor(source):
    """顶部 import 执行器纯函数."""
    assert "from nx_move_executor import" in source
    assert "compute_linear_target" in source
    assert "compute_angular_target_yaw" in source
    assert "run_angular_turn" in source
    assert "run_linear_translation" in source


def test_source_defines_execute_move_relative_and_await(source):
    """定义执行器与终态轮询方法."""
    assert "def _execute_move_relative(self, task):" in source
    assert "def _await_point_nav_terminal(self, task" in source
    # linear 走 point_nav, angular 走 cmd_vel+odom
    assert "self._point_nav.submit(" in source
    assert "run_angular_turn(" in source
    assert 'ws_broadcast({"type": "move_result"' in source


def test_source_main_injects_point_nav(source):
    """main() 把 point_nav 注入 task_mgr."""
    assert "task_mgr.set_point_nav(point_nav)" in source


def test_source_does_not_reuse_move_type_for_relative(source):
    """search_area 的 move waypoint 执行与 move_relative 必须分离."""
    assert 'task.type == "move"' not in source.replace(
        'task.type == "move_relative"', '')


def test_forward_relative_move_still_submits_point_navigation(source, source_tree):
    method = method_source(source, source_tree, "TaskManager",
                           "_execute_move_relative")
    method = textwrap.dedent(method)
    tree = ast.parse(method)
    forward_branch = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and compares_direction(node.test, "forward")
    )
    branch_source = ast.get_source_segment(method, forward_branch)
    assert "compute_linear_target" in branch_source
    assert "self._point_nav.submit" in branch_source


def test_backward_relative_move_uses_closed_loop_reverse_not_point_nav(source,
                                                                       source_tree):
    method = method_source(source, source_tree, "TaskManager",
                           "_execute_move_relative")
    method = textwrap.dedent(method)
    tree = ast.parse(method)
    reverse_branch = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and compares_direction(node.test, "backward")
    )
    branch_source = ast.get_source_segment(method, reverse_branch)
    assert "run_linear_translation" in branch_source
    assert "directional_clearance" in branch_source
    assert "180.0" in branch_source and "30.0" in branch_source
    assert "manual=False" in branch_source
    assert "self._point_nav.submit" not in branch_source
    assert "get_localization_health" in branch_source
    assert "self.robot.stop_move" in branch_source
    assert 'start_yaw=float(health["yaw"])' in branch_source
    assert "clearance_margin_m=_REVERSE_CLEARANCE_M" in branch_source


def test_angular_relative_move_uses_nav_owned_velocity_channel(source,
                                                                source_tree):
    """任务仲裁以 nav owner 激活后，闭环转向必须发布到同一个 nav 通道。"""
    method = method_source(source, source_tree, "TaskManager",
                           "_execute_move_relative")
    tree = ast.parse(textwrap.dedent(method))
    send_cmd = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "send_cmd"
    )
    move_call = next(
        node for node in ast.walk(send_cmd)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "move"
    )
    manual = next(
        keyword.value for keyword in move_call.keywords
        if keyword.arg == "manual"
    )
    assert isinstance(manual, ast.Constant)
    assert manual.value is False


def test_all_product_voice_tasks_are_owned_by_navigation_arbiter(source,
                                                                  source_tree):
    """产品语音入口统一经 task arbiter 激活 nav owner，不得直通 manual。"""
    admission = method_source(source, source_tree, "TaskManager",
                              "_admit_command_result")
    add_list = method_source(source, source_tree, "TaskManager", "add_list")
    executor = method_source(source, source_tree, "TaskManager",
                             "_execute_move_relative")

    assert "self.add_list(" in admission
    assert "self._navigation_arbiter.start_tasks(" in add_list
    assert "task_mgr.set_navigation_arbiter(navigation_arbiter)" in source
    assert "manual=True" not in executor


def test_bridge_directional_clearance_uses_scan_snapshot_and_age(source,
                                                                  source_tree):
    method = method_source(source, source_tree, "NxRobotBridge",
                           "directional_clearance")
    assert "get_scan_snapshot" in method
    assert "max_age_sec" in method
    assert "directional_clearance_from_scan" in method


def test_bridge_move_guards_autonomous_forward_and_reverse(source, source_tree):
    method = method_source(source, source_tree, "NxRobotBridge", "move")
    assert "vx > 0.0" in method
    assert "vx < 0.0" in method
    assert "directional_clearance(180.0" in method
    assert "rear <= _REVERSE_CLEARANCE_M" in method
    assert "manual" in method


def test_server_defines_one_shared_reverse_clearance_limit(source):
    assert 'GO2W_REVERSE_CLEARANCE", "0.55"' in source
    assert "sanitize_clearance_margin" in source
    assert source.count("clearance_margin_m=_REVERSE_CLEARANCE_M") == 1
    assert source.count("rear <= _REVERSE_CLEARANCE_M") == 1


def test_scan_freshness_uses_monotonic_receipt_age(source, source_tree):
    init = method_source(source, source_tree, "NxWebNode", "__init__")
    on_scan = method_source(source, source_tree, "NxWebNode", "_on_scan")
    snapshot = method_source(source, source_tree, "NxWebNode",
                             "get_scan_snapshot")
    assert "_scan_received_monotonic" in init
    assert "_scan_received_monotonic" in on_scan
    assert "time.monotonic()" in on_scan
    assert "_scan_received_monotonic" in snapshot
    assert "time.monotonic()" in snapshot
    assert "time.time() - timestamp" not in snapshot
