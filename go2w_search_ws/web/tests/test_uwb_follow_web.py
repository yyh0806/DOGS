"""UWB 跟随 web 集成契约测试 (功能 B-2)。

两层验证:
1. 功能层: 用 ROS mock 导入 nx_web_server, 经 build_uwb_follow_controller
   工厂注入替身, 全链路 (tick → arbiter manual 所有权 → robot.move) 验证
   接线正确 (manual=True 通道、闸门闭包、WS 广播类型)。
2. 源码契约层: 端点注册 / audited_paths / /api/stop、/api/e_stop 停跟随联动 /
   /api/status 汇聚 / 前端 panel.html 按钮与 WS 分支 (沿用
   test_panel_navigation_contract.py 的静态契约风格)。
"""
import sys
import types
from pathlib import Path

import pytest

_WEB = Path(__file__).resolve().parents[1]
if str(_WEB) not in sys.path:
    sys.path.insert(0, str(_WEB))


# 测试环境无 rclpy/ROS 消息: 注入最小 mock (nx_web_server 顶层 import 需要,
# 与 test_plan_executor.py 同款模式)
def _install_ros_mocks():
    class _Node:
        pass

    if "rclpy" not in sys.modules:
        _rclpy = types.ModuleType("rclpy")
        _rclpy.init = lambda *a, **k: None
        _rclpy.ok = lambda: True
        _rclpy.shutdown = lambda *a, **k: None
        _rclpy.spin = lambda *a, **k: None
        sys.modules["rclpy"] = _rclpy
    _node_mod = types.ModuleType("rclpy.node")
    _node_mod.Node = _Node
    sys.modules["rclpy.node"] = _node_mod
    _qos_mod = types.ModuleType("rclpy.qos")
    _qos_mod.qos_profile_sensor_data = lambda: None
    sys.modules["rclpy.qos"] = _qos_mod
    for _pkg, _mods in {
        "geometry_msgs.msg": {"Twist": type("Twist", (), {})},
        "nav_msgs.msg": {"Odometry": type("Odometry", (), {})},
        "sensor_msgs.msg": {
            "Imu": type("Imu", (), {}),
            "LaserScan": type("LaserScan", (), {}),
            "PointCloud2": type("PointCloud2", (), {}),
        },
        "std_msgs.msg": {"String": type("String", (), {})},
    }.items():
        _m = types.ModuleType(_pkg)
        for _name, _cls in _mods.items():
            setattr(_m, _name, _cls)
        sys.modules[_pkg] = _m
        _parent = _pkg.rsplit(".", 1)[0]
        sys.modules.setdefault(_parent, types.ModuleType(_parent))


_install_ros_mocks()

import nx_web_server as nws  # noqa: E402
import nx_uwb_follow  # noqa: E402


class FakeRobot:
    """directional_clearance / move 替身 (签名与 NxRobotBridge 对齐)。"""

    def __init__(self, clearance=5.0):
        self.clearance = clearance
        self.moves = []

    def directional_clearance(self, center_deg, half_fov_deg):
        return self.clearance

    def move(self, vx, vy, vyaw, manual=False):
        self.moves.append((vx, vy, vyaw, manual))

    def stop_move(self):
        self.moves.append((0.0, 0.0, 0.0, False))


class FakeArbiter:
    def __init__(self):
        self.acquires = []
        self.releases = []

    def run_manual_action(self, reason, action):
        self.acquires.append(reason)
        action()
        return {"ok": True}

    def release_manual(self, reason="release"):
        self.releases.append(reason)
        return {"ok": True}


class FakeSource:
    def __init__(self, range_m=3.0):
        self.range_m = range_m

    def get_fix(self):
        return {"ok": True, "range_m": self.range_m, "age_sec": 0.05}


# ============================================================================
# 1) 功能层: 工厂全链路
# ============================================================================

def _build(clearance=5.0, range_m=3.0):
    robot = FakeRobot(clearance)
    arbiter = FakeArbiter()
    broadcast = []

    def _broadcast(message, force=False):
        broadcast.append(message)

    controller = nws.build_uwb_follow_controller(
        robot, arbiter, source=FakeSource(range_m), broadcast=_broadcast)
    return controller, robot, arbiter, broadcast


def test_factory_wires_manual_channel_and_ownership():
    controller, robot, arbiter, broadcast = _build()
    assert controller.start()["ok"] is True
    snap = controller.tick()
    assert snap["state"] == "following"
    # 速度经 robot.move(manual=True) 直发 (闸门已在控制器内先行)
    assert robot.moves, "velocity must be published through robot.move"
    vx, vy, wz, manual = robot.moves[-1]
    assert vx > 0 and manual is True
    # 所有权走 arbiter manual 通道
    assert arbiter.acquires == ["uwb_follow"]
    # WS 广播类型 uwb_follow
    assert broadcast and all(m["type"] == "uwb_follow" for m in broadcast)
    assert broadcast[-1]["data"]["state"] == "following"


def test_factory_probe_closure_uses_directional_clearance():
    controller, robot, arbiter, broadcast = _build(clearance=0.3)
    controller.start()
    snap = controller.tick()
    assert snap["state"] == "obstacle_hold"
    assert snap["gate"]["blocked"] is True
    # 闸门挡: 不发布平移速度
    if robot.moves:
        vx, vy, wz, manual = robot.moves[-1]
        assert vx == 0.0 and vy == 0.0


def test_factory_stop_releases_ownership():
    controller, robot, arbiter, broadcast = _build()
    controller.start()
    controller.tick()
    controller.stop("operator_stop")
    assert arbiter.releases, "stop must release manual ownership"
    assert controller.get_state()["state"] == "idle"


def test_load_uwb_follow_source_returns_none_without_bridge(monkeypatch):
    # 开发机无 nx_uwb_bridge → None (start 拒绝, 不抛)
    monkeypatch.delenv("GO2W_UWB_FOLLOW_DISABLE", raising=False)
    assert nws._load_uwb_follow_source() is None
    monkeypatch.setenv("GO2W_UWB_FOLLOW_DISABLE", "1")
    assert nws._load_uwb_follow_source() is None


def test_load_uwb_follow_source_uses_bridge_factory(monkeypatch):
    calls = []

    class _Bridge:
        def get_fix(self):
            return {"ok": True, "range_m": 2.0}

    fake_module = types.ModuleType("nx_uwb_bridge")
    fake_module.get_follow_fix_source = lambda: (calls.append(1) or _Bridge())
    monkeypatch.setitem(sys.modules, "nx_uwb_bridge", fake_module)
    source = nws._load_uwb_follow_source()
    assert source is not None and calls == [1]
    assert source.get_fix()["range_m"] == 2.0


def test_uwb_follow_loop_stops_on_event():
    import threading
    controller, robot, arbiter, broadcast = _build()
    stop_event = threading.Event()
    thread = threading.Thread(
        target=nws._uwb_follow_loop, args=(controller, stop_event), daemon=True)
    controller.start()
    thread.start()
    stop_event.set()
    thread.join(timeout=2.0)
    assert not thread.is_alive()


# ============================================================================
# 2) 源码契约层 (端点注册 / 联动 / 前端)
# ============================================================================

_SERVER = (_WEB / "nx_web_server.py").read_text(encoding="utf-8")
_PANEL = (_WEB / "static" / "panel.html").read_text(encoding="utf-8")


def test_server_registers_uwb_follow_endpoints():
    for path in ("/api/uwb_follow/start", "/api/uwb_follow/stop",
                 "/api/uwb_follow/status", "/api/uwb_follow/params"):
        assert path in _SERVER, f"missing endpoint {path}"


def test_endpoints_are_audited_control_requests():
    assert '"/api/uwb_follow/start"' in _SERVER
    assert '"/api/uwb_follow/stop"' in _SERVER
    assert '"/api/uwb_follow/params"' in _SERVER


def test_stop_and_estop_stop_follow_first():
    # /api/stop 与 /api/e_stop 分支内必须先 uwb_follow.stop 再 arbiter 停车
    stop_seg = _SERVER.split("elif p.path == '/api/stop':", 1)[1].split(
        "elif p.path", 1)[0]
    assert "uwb_follow.stop(" in stop_seg
    estop_seg = _SERVER.split("elif p.path == '/api/e_stop':", 1)[1].split(
        "elif p.path", 1)[0]
    assert "uwb_follow.stop(" in estop_seg


def test_status_snapshot_includes_uwb_follow():
    assert '"uwb_follow": uwb_follow.get_state() if uwb_follow else {}' in _SERVER


def test_ws_broadcast_type_registered():
    assert '{"type": "uwb_follow", "data": state}' in _SERVER


def test_main_construction_and_shutdown_wiring():
    assert "build_uwb_follow_controller(" in _SERVER
    assert "_uwb_follow_loop" in _SERVER
    assert 'uwb_follow.stop("web_shutdown")' in _SERVER
    assert "global robot, task_mgr, node, point_nav, navigation_gateway" in _SERVER


def test_panel_has_follow_button_and_ws_branch():
    assert 'id="uwbFollowBtn"' in _PANEL
    assert "toggleUwbFollow()" in _PANEL
    assert "/api/uwb_follow/start" in _PANEL
    assert "/api/uwb_follow/stop" in _PANEL
    assert "data.type === 'uwb_follow'" in _PANEL
    assert "'uwb_follow'," in _PANEL  # STATUS_OVERLAP_WS_TYPES


def test_controller_importable_and_ros_free():
    import inspect
    source = inspect.getsource(nx_uwb_follow)
    for forbidden in ("import rclpy", "from rclpy", "import rospy",
                      "nav_msgs", "geometry_msgs"):
        assert forbidden not in source, f"nx_uwb_follow must stay ROS-free: {forbidden}"
