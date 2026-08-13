"""多步计划执行器契约测试 (Task plan_id + 失败中止)。"""
import sys
import types
from pathlib import Path

import pytest

_WEB = Path(__file__).resolve().parents[1]
if str(_WEB) not in sys.path:
    sys.path.insert(0, str(_WEB))

# 测试环境无 rclpy/ROS 消息: 注入最小 mock (nx_web_server 顶层 import 需要)
def _install_ros_mocks():
    class _Node:
        pass

    def _qos_profile_sensor_data():
        return None

    class _Msg:
        pass

    class _Twist(_Msg):
        linear = None
        angular = None

    class _Odometry(_Msg):
        pose = None

    class _Imu(_Msg):
        pass

    class _LaserScan(_Msg):
        pass

    class _PointCloud2(_Msg):
        pass

    class _String(_Msg):
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
    _qos_mod.qos_profile_sensor_data = _qos_profile_sensor_data
    sys.modules["rclpy.qos"] = _qos_mod
    for _pkg, _mods in {
        "geometry_msgs.msg": {"Twist": _Twist},
        "nav_msgs.msg": {"Odometry": _Odometry},
        "sensor_msgs.msg": {"Imu": _Imu, "LaserScan": _LaserScan,
                            "PointCloud2": _PointCloud2},
        "std_msgs.msg": {"String": _String},
    }.items():
        _m = types.ModuleType(_pkg)
        for _name, _cls in _mods.items():
            setattr(_m, _name, _cls)
        sys.modules[_pkg] = _m
        _parent = _pkg.rsplit(".", 1)[0]
        sys.modules.setdefault(_parent, types.ModuleType(_parent))


_install_ros_mocks()

import nx_web_server as nws  # noqa: E402


class _FakeRobot:
    def stop_move(self):
        pass


class _FakeMgr(nws.TaskManager):
    """免 navigation_arbiter/worker 的裸 TaskManager (只测计划队列逻辑)。"""

    def __init__(self):
        super().__init__(_FakeRobot(), vlm_engine=None, detector=None,
                         room_orchestrator=None)
        self._navigation_arbiter = None


def test_add_plan_assigns_plan_id_and_index():
    mgr = _FakeMgr()
    t1 = nws.Task("go_landmark", {"landmark": "大门"})
    t2 = nws.Task("fetch", {"pickup": "大门", "object": "咖啡"})
    r = mgr.add_plan([t1, t2])
    assert r["ok"] is True
    assert t1.plan_id == t2.plan_id
    assert t1.plan_index == 0 and t2.plan_index == 1
    # 优先级递减 (先步更高)
    assert t1.priority > t2.priority


def test_cancel_plan_pending_aborts_remaining():
    mgr = _FakeMgr()
    t1 = nws.Task("go_landmark", {"landmark": "大门"})
    t2 = nws.Task("fetch", {"pickup": "大门", "object": "咖啡"})
    mgr.add_plan([t1, t2])
    t1.status = "failed"  # 模拟第 1 步失败
    cancelled = mgr._cancel_plan_pending(t1.plan_id)
    assert cancelled == 1
    assert t2.status == "cancelled"
    assert t2.result == "plan_aborted"


def test_cancel_plan_does_not_touch_other_plans():
    mgr = _FakeMgr()
    a1 = nws.Task("go_landmark", {"landmark": "大门"})
    a2 = nws.Task("fetch", {"pickup": "大门", "object": "咖啡"})
    b1 = nws.Task("follow", {"target": "人"})
    mgr.add_plan([a1, a2])
    mgr.add_plan([b1])
    a1.status = "failed"
    mgr._cancel_plan_pending(a1.plan_id)
    assert a2.status == "cancelled"
    assert b1.status == "pending"  # 其他计划不受影响


def test_task_to_dict_includes_plan_fields():
    t = nws.Task("fetch", {"object": "咖啡"}, plan_id="p1")
    t.plan_index = 2
    d = t.to_dict()
    assert d["plan_id"] == "p1" and d["plan_index"] == 2
