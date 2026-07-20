"""Task 4 静态契约: nx_web_server.py 含 move_relative 集成点.

动态执行测试 (mock TaskManager 跑 _execute_move_relative) 依赖完整 ROS 栈
实例化 TaskManager, Windows 无 ROS 环境不可靠。集成契约用源码断言验证,
动态端到端验收在 NX 部署后执行 (见 plan §部署与验收).
"""

from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parent / "nx_web_server.py"


@pytest.fixture(scope="module")
def source() -> str:
    return SRC.read_text(encoding="utf-8")


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
