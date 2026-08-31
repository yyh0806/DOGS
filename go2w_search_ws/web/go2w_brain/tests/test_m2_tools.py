"""M2 工具集测试: 规划/航线/标定/取消 + 干跑 + POST 契约 (本地 HTTP 假服务器)。

lake_plan 相关用例使用 lake_plan/tests 的入库瓦片夹具 (封闭, 零网络)。
"""
from __future__ import annotations

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

# lake_plan 夹具 (在导入 lake_plan 前设置即可: config 每次调用读环境)
_LAKE_FIXTURES = (Path(__file__).resolve().parents[2]
                  / "lake_plan" / "tests" / "fixtures")
os.environ["GO2W_LAKE_CACHE_DIR"] = str(_LAKE_FIXTURES / "tiles")
os.environ["GO2W_LAKE_OFFLINE"] = "1"

from go2w_brain.platform import MockAdapter, NxHttpAdapter  # noqa: E402
from go2w_brain.registry import ToolRegistry  # noqa: E402
from go2w_brain.tools import BUILTIN_TOOLS  # noqa: E402

TOOLS = {t.name: t for t in BUILTIN_TOOLS}


@pytest.fixture
def mock_platform():
    return MockAdapter()


@pytest.fixture
def gate():
    from go2w_brain.dispatcher import DispatchGate, require_mission_lock
    registry = ToolRegistry()
    for tool in BUILTIN_TOOLS:
        registry.register(tool)
    gate = DispatchGate(registry)
    gate.register_precondition("mission_lock", require_mission_lock)
    return gate


def _ctx(platform, config=None, mission_lock="m-test"):
    return {"platform": platform, "log": _NullLog(), "config": config,
            "mission_lock": mission_lock}


class _NullLog:
    def append(self, *a, **k):
        return None


# ---------- plan_lake_loop ----------

def test_plan_lake_loop_with_fixture_cache(mock_platform):
    result = TOOLS["plan_lake_loop"].execute({}, _ctx(mock_platform))
    assert result["ok"], result.get("reason")
    # 摘要契约: LLM 只见计数/统计/样本, 不见全量数组 (上下文经济)
    assert result["waypoint_count"] > 8
    assert result["closed"] is True
    assert 8.0 < result["length_km"] < 20.0
    assert len(result["first_waypoints"]) == 3
    assert result["lake"]["dist_km"] < 2.5  # mock GPS=蠡湖中心 → 选蠡湖本身


def test_plan_lake_loop_no_gps_no_center(mock_platform):
    snapshot = mock_platform.snapshot()
    mock_platform._base["gps"] = {"available": False}
    result = TOOLS["plan_lake_loop"].execute({}, _ctx(mock_platform))
    assert result["ok"] is False
    assert result["reason"] == "no_gps_and_no_explicit_center"
    mock_platform._base["gps"] = snapshot["gps"]


# ---------- follow_route / cancel / calibrate ----------

_WPS = [{"lat": 31.51, "lon": 120.26}, {"lat": 31.52, "lon": 120.27},
        {"lat": 31.51, "lon": 120.26}]


def test_follow_route_dispatch_requires_mission_lock(gate):
    ok, reason, _ = gate.check("follow_route", {"waypoints": _WPS}, {})
    assert not ok and reason == "mission_lock_required"


def test_plan_then_follow_from_plan_reference(mock_platform):
    """引用传递: plan_lake_loop 摘要 → follow_route(from_plan) 全量受理。"""
    plan_store: dict = {}
    ctx = _ctx(mock_platform)
    ctx["plan_store"] = plan_store
    plan = TOOLS["plan_lake_loop"].execute({}, ctx)
    assert plan["ok"] and plan["waypoint_count"] > 8
    assert "waypoints" not in plan  # LLM 只见摘要, 不见全量数组
    assert "last_route" in plan_store
    follow = TOOLS["follow_route"].execute({"from_plan": True}, ctx)
    assert follow["ok"] and follow["waypoint_total"] == plan["waypoint_count"]
    assert mock_platform.calls[-1][0] == "submit_gps_route"


def test_follow_from_plan_without_plan(mock_platform):
    follow = TOOLS["follow_route"].execute(
        {"from_plan": True}, _ctx(mock_platform))
    assert follow["ok"] is False
    assert follow["reason"] == "no_plan_in_session"


def test_follow_route_wet_on_mock(gate, mock_platform):
    ok, reason, tool = gate.check("follow_route", {"waypoints": _WPS},
                                  _ctx(mock_platform))
    assert ok, reason
    result = tool.execute({"waypoints": _WPS}, _ctx(mock_platform))
    assert result["ok"] and result["active"]
    assert mock_platform.calls[-1][0] == "submit_gps_route"
    state = mock_platform.gps_state()
    assert state["active"] and state["waypoint_total"] == 3


def test_follow_route_dry_never_submits(mock_platform):
    class Cfg:
        dry_run = True
    result = TOOLS["follow_route"].execute(
        {"waypoints": _WPS}, _ctx(mock_platform, config=Cfg()))
    assert result["ok"] and result["dry"] is True
    assert not any(c[0] == "submit_gps_route" for c in mock_platform.calls)


def test_follow_route_rejects_bad_waypoints(gate, mock_platform):
    ok, reason, tool = gate.check(
        "follow_route", {"waypoints": [{"lat": 1.0, "lng": 2.0}]},
        _ctx(mock_platform))
    assert ok  # 顶层 schema 通过 (array), 字段校验在工具内
    result = tool.execute({"waypoints": [{"lat": 1.0, "lng": 2.0}]},
                          _ctx(mock_platform))
    assert result["ok"] is False and "invalid_waypoint" in result["reason"]


def test_cancel_route_always_dispatchable(gate, mock_platform):
    ok, reason, tool = gate.check("cancel_route", {}, {})  # 无任务锁也放行
    assert ok
    result = tool.execute({}, _ctx(mock_platform, mission_lock=None))
    assert result["ok"]
    assert mock_platform.calls[-1][0] == "cancel_gps_route"


def test_calibrate_heading_validates(gate):
    ok, reason, _ = gate.check("calibrate_heading", {}, _ctx(None))
    assert not ok and "missing_required" in reason


# ---------- NxHttpAdapter POST 契约 (本地假服务器) ----------

class _FakeNX(BaseHTTPRequestHandler):
    scenarios = {}  # path -> (status, payload)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length) or b"{}")
        auth = self.headers.get("Authorization", "")
        status, payload = self.scenarios.get(
            self.path, (202, {"ok": True}))
        if auth != "Bearer tok-1":
            status, payload = 401, {"ok": False, "reason": "unauthorized"}
        if self.path == "/api/gps/route":
            payload = dict(payload, _seen_waypoints=len(
                body.get("waypoints") or []))
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(payload).encode())

    def log_message(self, *args):
        pass


@pytest.fixture
def fake_nx():
    server = HTTPServer(("127.0.0.1", 0), _FakeNX)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{server.server_port}"
    server.shutdown()


def test_nx_adapter_submit_contract(fake_nx):
    _FakeNX.scenarios = {"/api/gps/route": (202, {"ok": True, "active": True})}
    adapter = NxHttpAdapter(fake_nx, timeout=3.0, control_token="tok-1")
    result = adapter.submit_gps_route(_WPS)
    assert result["ok"] and result["_seen_waypoints"] == 3


def test_nx_adapter_rejection_passes_reason(fake_nx):
    _FakeNX.scenarios = {"/api/gps/route": (
        409, {"ok": False, "reason": "heading_not_calibrated"})}
    adapter = NxHttpAdapter(fake_nx, timeout=3.0, control_token="tok-1")
    result = adapter.submit_gps_route(_WPS)
    assert result["ok"] is False
    assert result["http_status"] == 409
    assert result["reason"] == "heading_not_calibrated"


def test_nx_adapter_requires_token(fake_nx):
    _FakeNX.scenarios = {}
    adapter = NxHttpAdapter(fake_nx, timeout=3.0, control_token="")
    result = adapter.calibrate_heading(12.0)
    assert result["ok"] is False and result["http_status"] == 401


def test_nx_adapter_unreachable():
    adapter = NxHttpAdapter("http://127.0.0.1:1", timeout=0.3,
                            control_token="t")
    result = adapter.cancel_gps_route()
    assert result["ok"] is False
    assert str(result["reason"]).startswith("unreachable:")
