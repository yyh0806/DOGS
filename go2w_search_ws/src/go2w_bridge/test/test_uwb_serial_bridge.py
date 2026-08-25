"""UWB 串口桥测试:mock 注入路径、串口降级、回调分发(B-1)。

无 ROS / 无 pyserial / 无硬件环境即可全绿(工单要求)。
"""

import threading
import time

import pytest

from go2w_bridge.nooploop_uwb_protocol import AoaNodeFrame, TagFrame, build_tag_frame
from go2w_bridge.uwb_serial_bridge import (
    MockUwbSource,
    UwbBridgeConfig,
    UwbSerialBridge,
)


def test_config_rejects_invalid_values():
    with pytest.raises(ValueError):
        UwbBridgeConfig(mode="wifi")
    with pytest.raises(ValueError):
        UwbBridgeConfig(baudrate=0)
    with pytest.raises(ValueError):
        UwbBridgeConfig(mock_hz=0)


def test_no_port_falls_back_to_mock_with_reason():
    bridge = UwbSerialBridge(UwbBridgeConfig(port=None, mode="auto"))
    assert bridge.open() == "mock"
    assert bridge.status()["source"] == "mock"
    assert bridge.status()["fallback_reason"]


def test_serial_failure_falls_back_to_mock():
    # 本机无 pyserial / 端口不存在:两种失败都必须降级而非抛出
    bridge = UwbSerialBridge(
        UwbBridgeConfig(port="COM_INVALID_99", mode="serial")
    )
    assert bridge.open() == "mock"
    status = bridge.status()
    assert status["fallback_reason"]
    assert "COM_INVALID_99" in status["fallback_reason"] or "pyserial" in status["fallback_reason"]


def test_explicit_mock_mode_never_touches_serial():
    bridge = UwbSerialBridge(UwbBridgeConfig(mode="mock"))
    assert bridge.open() == "mock"
    assert bridge.status()["fallback_reason"] == "mode=mock"


def test_inject_bytes_routes_through_real_parser():
    bridge = UwbSerialBridge()
    frames = bridge.inject_tag(tag_id=5, distance_m=4.2, angle_deg=25.0)
    assert len(frames) == 1
    assert isinstance(frames[0], AoaNodeFrame)
    node = frames[0].find_node(5)
    assert node is not None
    assert node.distance_m == pytest.approx(4.2, abs=1e-3)
    assert node.angle_deg == pytest.approx(25.0, abs=0.02)


def test_inject_tag_fires_on_tag_and_updates_latest():
    received = []
    bridge = UwbSerialBridge(
        UwbBridgeConfig(followed_tag_id=7), on_tag=lambda tag_id, payload: received.append((tag_id, payload))
    )
    bridge.inject_tag(tag_id=7, distance_m=1.5, angle_deg=-10.0)
    bridge.inject_tag(tag_id=9, distance_m=9.9)  # 非跟随标签:被过滤
    assert [tag_id for tag_id, _ in received] == [7]
    payload = received[0][1]
    assert payload["distance_m"] == pytest.approx(1.5, abs=1e-3)
    latest = bridge.latest()
    assert latest["id"] == 7
    assert latest["distance_m"] == pytest.approx(1.5, abs=1e-3)
    # 快照缓存所有标签(过滤只作用于 on_tag 回调),便于后续切换跟随目标
    assert bridge.latest(tag_id=9)["distance_m"] == pytest.approx(9.9, abs=1e-3)


def test_on_tag_without_filter_receives_all_tags():
    received = []
    bridge = UwbSerialBridge(on_tag=lambda tag_id, payload: received.append(tag_id))
    bridge.inject_tag(tag_id=3, distance_m=2.0)
    bridge.inject_tag(tag_id=4, distance_m=2.5)
    assert received == [3, 4]


def test_on_frame_receives_tag_frames_too():
    frames = []
    bridge = UwbSerialBridge(on_frame=frames.append)
    bridge.inject_bytes(build_tag_frame(tag_id=2, distances_m=(2.5,) + (0.0,) * 7))
    assert len(frames) == 1
    assert isinstance(frames[0], TagFrame)
    snapshot = bridge.latest(tag_id=2)
    assert snapshot["distance_m"] == pytest.approx(2.5, abs=1e-3)


def test_callback_exception_does_not_break_bridge():
    calls = []

    def bad_callback(frame):
        calls.append(frame)
        raise RuntimeError("boom")

    bridge = UwbSerialBridge(on_frame=bad_callback)
    frames = bridge.inject_tag(tag_id=1, distance_m=1.0)
    assert len(frames) == 1  # 解析结果照常返回
    assert len(calls) == 1
    assert bridge.latest(tag_id=1) is not None  # 快照先于回调写入


def test_mock_source_generates_protocol_frames():
    source = MockUwbSource(tag_id=8, hz=100.0, scenario=lambda t: (2.0 + 0.5 * t, 45.0))
    raw = source.next_frame_bytes()
    bridge = UwbSerialBridge()
    (frame,) = bridge.inject_bytes(raw)
    node = frame.find_node(8)
    assert node is not None
    assert node.angle_deg == pytest.approx(45.0, abs=0.02)


def test_run_for_mock_scenario_updates_latest():
    bridge = UwbSerialBridge(
        UwbBridgeConfig(mode="mock", mock_hz=200.0, followed_tag_id=1)
    )
    parsed = bridge.run_for(0.1)
    assert parsed >= 5  # 200Hz * 0.1s 的下界(保守)
    latest = bridge.latest()
    assert latest is not None
    assert 2.9 <= latest["distance_m"] <= 3.1  # 默认场景恒 3.0m


def test_threaded_start_and_stop():
    got_frame = threading.Event()
    frames = []

    def on_frame(frame):
        frames.append(frame)
        got_frame.set()

    bridge = UwbSerialBridge(
        UwbBridgeConfig(mode="mock", mock_hz=100.0), on_frame=on_frame
    )
    assert bridge.start() == "mock"
    try:
        assert got_frame.wait(timeout=5.0), "mock 线程 5s 内未产出任何帧"
    finally:
        bridge.stop()
    assert bridge.status()["stats"]["frames_parsed"] >= 1


def test_status_reports_parser_stats():
    bridge = UwbSerialBridge()
    bridge.inject_tag(tag_id=1, distance_m=1.0)
    bridge.inject_bytes(b"\xFF\xFE\x55\x99")  # 纯噪声
    status = bridge.status()
    assert status["stats"]["frames_parsed"] == 1
    assert status["stats"]["dropped_noise_bytes"] >= 2
    assert status["pending_bytes"] == 0


def test_bridge_module_has_no_ros_dependency():
    """桥模块源码级禁止 ROS;pyserial 只允许函数内懒加载(降级前提)。"""
    import ast
    from pathlib import Path

    import go2w_bridge.uwb_serial_bridge as module

    tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
    module_level = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            module_level.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            module_level.add(node.module.split(".")[0])
    assert "rclpy" not in module_level and "rclcpp" not in module_level
    assert "serial" not in module_level  # 串口只能懒加载,保证无 pyserial 可导入
    # 懒加载契约:_try_open_serial 中 import serial 必须被 except ImportError 包住
    source = Path(module.__file__).read_text(encoding="utf-8")
    assert "import serial" in source and "except ImportError" in source
