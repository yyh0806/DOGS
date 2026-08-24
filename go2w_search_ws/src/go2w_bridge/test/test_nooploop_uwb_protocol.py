"""Nooploop NLink UWB 协议解析器字节流回归测试(B-1 完成标准)。

全部用例只依赖纯逻辑模块:无 ROS、无 pyserial、无硬件。
帧常量/布局逐字节对照官方 nlink_unpack C 库。
"""

import pytest

from go2w_bridge.nooploop_uwb_protocol import (
    AOA_FRAME_FIXED,
    AOA_NODE_FRAME_MARK,
    AOA_NODE_SIZE,
    AoaNodeFrame,
    AoaNodeMeasurement,
    AoaNodeSpec,
    MAX_FRAME_BYTES_DEFAULT,
    NODE_FRAME_FIXED,
    NODE_FRAME_MARK,
    NLinkStreamParser,
    ProtocolError,
    TAG_FRAME_MARK,
    TAG_FRAME_SIZE,
    TagFrame,
    build_aoa_node_frame,
    build_node_frame,
    build_tag_frame,
    compute_checksum,
    pack_int24,
    unpack_int24,
)


def make_tag_bytes(**overrides) -> bytes:
    kwargs = dict(
        tag_id=7,
        role=2,
        position_m=(1.234, -2.345, 0.567),
        velocity_mps=(0.12, -0.34, 0.0),
        distances_m=(3.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        imu_gyro=(0.01, -0.02, 0.03),
        imu_acc=(0.1, 0.2, 9.8),
        angle_deg=(1.5, -2.5, 179.99),
        quaternion=(1.0, 0.0, 0.0, 0.0),
        local_time_ms=123456,
        system_time_ms=654321,
        eop=(0.5, 0.25, 0.75),
        voltage_v=3.65,
    )
    kwargs.update(overrides)
    return build_tag_frame(**kwargs)


def make_aoa_bytes(**overrides) -> bytes:
    kwargs = dict(
        nodes=[
            AoaNodeSpec(id=1, role=2, distance_m=2.75, angle_deg=30.0, fp_rssi_dbm=-55.0, rx_rssi_dbm=-68.0),
            AoaNodeSpec(id=2, role=2, distance_m=5.125, angle_deg=-15.0, fp_rssi_dbm=-70.0, rx_rssi_dbm=-80.0),
        ],
        device_role=3,
        device_id=0,
        local_time_ms=999,
        system_time_ms=8888,
        voltage_v=3.7,
    )
    kwargs.update(overrides)
    return build_aoa_node_frame(**kwargs)


def make_node_frame(**overrides) -> bytes:
    kwargs = dict(entries=[(2, 9, bytes((0xDE, 0xAD)))], device_role=3, device_id=5)
    kwargs.update(overrides)
    return build_node_frame(**kwargs)


# --------------------------------------------------------------- 帧常量契约

def test_frame_constants_match_official_nlink_unpack():
    """帧标识/定长/节点长度必须与官方 C 库一致(升级固件时的护栏)。"""
    assert TAG_FRAME_MARK == 0x01
    assert NODE_FRAME_MARK == 0x02
    assert AOA_NODE_FRAME_MARK == 0x07
    assert TAG_FRAME_SIZE == 128
    assert AOA_FRAME_FIXED == 21
    assert NODE_FRAME_FIXED == 11
    assert AOA_NODE_SIZE == 11


def test_checksum_matches_official_algorithm():
    # NLINK_VerifyCheckSum: 除末字节外累加取低 8 位
    assert compute_checksum(b"\x01\x02\x03") == 6
    frame = make_aoa_bytes()
    assert compute_checksum(frame[:-1]) == frame[-1]


def test_int24_roundtrip_negative_and_positive():
    for value in (0, 1, -1, 1234, -8388608, 8388607, -4194304):
        packed = pack_int24(value)
        assert len(packed) == 3
        assert unpack_int24(packed) == value


def test_int24_negative_encoding_example():
    # -1.234 m 位置 => 原始 -1234 => 小端补码 2E FB FF(0xFFFB2E)
    assert pack_int24(-1234) == bytes((0x2E, 0xFB, 0xFF))
    assert unpack_int24(b"\x2E\xFB\xFF") == -1234


# ---------------------------------------------------------------- 解码正确性

def test_tag_frame_decode_all_fields():
    (frame,) = NLinkStreamParser().feed(make_tag_bytes())
    assert isinstance(frame, TagFrame)
    assert frame.id == 7
    assert frame.role == 2
    assert frame.role_name_ == "tag"
    assert frame.position_m == pytest.approx((1.234, -2.345, 0.567), abs=1e-3)
    assert frame.velocity_mps == pytest.approx((0.12, -0.34, 0.0), abs=1e-4)
    assert frame.distances_m[0] == pytest.approx(3.25, abs=1e-3)
    assert frame.distances_m[1:] == (0.0,) * 7
    assert frame.imu_gyro == pytest.approx((0.01, -0.02, 0.03), abs=1e-6)
    assert frame.imu_acc == pytest.approx((0.1, 0.2, 9.8), abs=1e-6)
    assert frame.angle_deg == pytest.approx((1.5, -2.5, 179.99), abs=0.02)
    assert frame.quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert frame.local_time_ms == 123456
    assert frame.system_time_ms == 654321
    assert frame.eop == pytest.approx((0.5, 0.25, 0.75), abs=0.01)
    assert frame.voltage_v == pytest.approx(3.65, abs=0.001)
    assert frame.raw == make_tag_bytes()


def test_aoa_node_frame_decode_multiple_nodes():
    (frame,) = NLinkStreamParser().feed(make_aoa_bytes())
    assert isinstance(frame, AoaNodeFrame)
    assert frame.device_role == 3
    assert frame.device_role_name == "console"
    assert frame.device_id == 0
    assert frame.local_time_ms == 999
    assert frame.system_time_ms == 8888
    assert len(frame.nodes) == 2
    first, second = frame.nodes
    assert isinstance(first, AoaNodeMeasurement)
    assert (first.id, first.role) == (1, 2)
    assert first.distance_m == pytest.approx(2.75, abs=1e-3)
    assert first.angle_deg == pytest.approx(30.0, abs=0.02)
    assert first.fp_rssi_dbm == pytest.approx(-55.0, abs=0.6)
    assert second.distance_m == pytest.approx(5.125, abs=1e-3)
    assert second.angle_deg == pytest.approx(-15.0, abs=0.02)
    assert frame.find_node(2) is second
    assert frame.find_node(42) is None
    assert frame.to_dict()["nodes"][0]["role_name"] == "tag"


def test_node_tlv_frame_decode():
    payload = bytes((0xDE, 0xAD, 0xBE, 0xEF))
    raw = build_node_frame(entries=[(2, 9, payload), (1, 3, b"")], device_role=3, device_id=5)
    (frame,) = NLinkStreamParser().feed(raw)
    assert frame.to_dict()["kind"] == "node_frame"
    assert frame.device_role == 3
    assert frame.device_id == 5
    assert len(frame.nodes) == 2
    assert frame.nodes[0].id == 9
    assert frame.nodes[0].data == payload
    assert frame.nodes[1].data == b""


def test_zero_node_aoa_frame_is_valid():
    raw = build_aoa_node_frame(nodes=[])
    (frame,) = NLinkStreamParser().feed(raw)
    assert frame.nodes == ()


# --------------------------------------------------------------- 字节流鲁棒性

def test_feed_byte_by_byte_yields_full_frames():
    stream = make_aoa_bytes() + make_tag_bytes() + make_node_frame(entries=[(2, 1, b"xy")])
    parser = NLinkStreamParser()
    frames = []
    for single in stream:
        frames.extend(parser.feed(bytes((single,))))
    assert len(frames) == 3
    assert isinstance(frames[0], AoaNodeFrame)
    assert isinstance(frames[1], TagFrame)
    assert frames[2].to_dict()["kind"] == "node_frame"


@pytest.mark.parametrize("split", [1, 2, 3, 4, 11, 21, 32, 63, 64, 127, 128, 129])
def test_every_split_point_of_tag_frame_stream(split):
    stream = b"\xAA" * 5 + make_tag_bytes()  # 前置噪声 + 定长帧
    parser = NLinkStreamParser()
    frames = parser.feed(stream[:split])
    frames += parser.feed(stream[split:])
    assert len(frames) == 1
    assert isinstance(frames[0], TagFrame)
    assert frames[0].id == 7


def test_partial_frame_waits_for_more_bytes():
    parser = NLinkStreamParser()
    aoa = make_aoa_bytes()
    assert parser.feed(aoa[:-3]) == []
    assert parser.pending_bytes >= len(aoa) - 3 - 1  # 至多丢弃了开头 0x55 前 noise
    (frame,) = parser.feed(aoa[-3:])
    assert isinstance(frame, AoaNodeFrame)
    assert frame.nodes[0].distance_m == pytest.approx(2.75, abs=1e-3)


def test_noise_between_frames_is_dropped_and_counted():
    parser = NLinkStreamParser()
    stream = b"\x00\xFFgarbage" + make_aoa_bytes() + b"\x77\x88" + make_aoa_bytes()
    frames = parser.feed(stream)
    assert len(frames) == 2
    assert all(isinstance(f, AoaNodeFrame) for f in frames)
    assert parser.stats.frames_parsed == 2
    assert parser.stats.dropped_noise_bytes >= len(b"\x00\xFFgarbage") + len(b"\x77\x88")


def test_checksum_corruption_is_rejected_and_stream_recovers():
    parser = NLinkStreamParser()
    broken = bytearray(make_aoa_bytes())
    broken[9] ^= 0x40  # 破坏载荷(帧尾校验和不再匹配)
    frames = parser.feed(bytes(broken))
    assert frames == []
    assert parser.stats.checksum_errors >= 1
    # 同一缓冲区后紧跟完好帧:必须恢复解析
    (frame,) = parser.feed(make_aoa_bytes())
    assert isinstance(frame, AoaNodeFrame)


def test_isolated_sync_byte_with_unknown_mark_is_skipped():
    parser = NLinkStreamParser()
    stream = b"\x55\x99\x55\x98" + make_aoa_bytes()
    frames = parser.feed(stream)
    assert len(frames) == 1
    assert parser.stats.frames_parsed == 1


def test_oversized_length_field_is_skipped_without_deadlock():
    parser = NLinkStreamParser()
    bomb = bytes((0x55, AOA_NODE_FRAME_MARK, 0xFF, 0xFF))  # 声称 65535B 的假帧
    (frame,) = parser.feed(bomb + make_aoa_bytes())
    assert isinstance(frame, AoaNodeFrame)
    assert parser.pending_bytes < MAX_FRAME_BYTES_DEFAULT


def test_length_below_minimum_is_skipped():
    parser = NLinkStreamParser()
    bogus = bytes((0x55, AOA_NODE_FRAME_MARK, 0x05, 0x00)) + make_aoa_bytes()
    frames = parser.feed(bogus)
    assert len(frames) == 1
    assert parser.stats.checksum_errors == 0


def test_aoa_count_mismatch_counts_as_malformed():
    # 校验和自洽但节点数与帧长矛盾(伪造:改 count 后重算校验和)
    frame = bytearray(make_aoa_bytes())
    frame[20] = 3  # 声称 3 个节点,实际数据只有 2 个
    frame[-1] = compute_checksum(bytes(frame[:-1]))
    parser = NLinkStreamParser()
    assert parser.feed(bytes(frame)) == []
    assert parser.stats.malformed_frames == 1
    # 后续好帧不受影响
    assert len(parser.feed(make_tag_bytes())) == 1


def test_node_tlv_overrun_is_malformed():
    frame = bytearray(build_node_frame(entries=[(2, 1, b"12345678")]))
    frame[10] = 2  # 声称 2 个节点,TLV 越界
    frame[-1] = compute_checksum(bytes(frame[:-1]))
    parser = NLinkStreamParser()
    assert parser.feed(bytes(frame)) == []
    assert parser.stats.malformed_frames == 1


def test_stats_counters_and_reset():
    parser = NLinkStreamParser()
    parser.feed(b"\xFF" * 3)
    parser.feed(make_tag_bytes())
    parser.feed(make_aoa_bytes())
    parser.feed(make_node_frame(entries=[(1, 1, b"z")]))
    broken = bytearray(make_aoa_bytes())
    broken[6] ^= 0xFF
    parser.feed(bytes(broken))
    stats = parser.stats.to_dict()
    assert stats["frames_parsed"] == 3
    assert stats["tag_frames"] == 1
    assert stats["aoa_frames"] == 1
    assert stats["node_frames"] == 1
    assert stats["checksum_errors"] == 1
    assert stats["dropped_noise_bytes"] >= 3
    parser.reset()
    assert parser.stats.to_dict()["frames_parsed"] == 0
    assert parser.pending_bytes == 0


def test_many_frames_single_feed():
    parser = NLinkStreamParser()
    stream = b"".join(make_aoa_bytes(local_time_ms=i) for i in range(100))
    frames = parser.feed(stream)
    assert len(frames) == 100
    assert [f.local_time_ms for f in frames] == list(range(100))


# ------------------------------------------------------------------ 编码器

def test_builder_checksum_is_consistent():
    for raw in (make_tag_bytes(), make_aoa_bytes(), build_node_frame(entries=[(2, 1, b"abc")])):
        assert compute_checksum(raw[:-1]) == raw[-1]


def test_builder_rejects_bad_distances_length():
    with pytest.raises(ProtocolError):
        build_tag_frame(distances_m=(1.0, 2.0))


def test_parser_rejects_max_frame_bytes_below_tag_frame():
    with pytest.raises(ValueError):
        NLinkStreamParser(max_frame_bytes=64)


def _module_level_imports(path) -> set:
    """静态收集模块级 import 的顶层包名(AST,不受测试会话污染)。"""
    import ast
    from pathlib import Path

    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    names = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            names.add(node.module.split(".")[0])
    return names


def test_module_has_no_ros_dependency():
    """解析器必须纯逻辑:源码级禁止 ROS/pyserial 依赖(工单硬性要求)。"""
    import go2w_bridge.nooploop_uwb_protocol as module

    imports = _module_level_imports(module.__file__)
    forbidden = {"rclpy", "rclcpp", "rospy", "serial", "serial_asyncio"}
    assert imports & forbidden == set(), f"parser pulled in {imports & forbidden}"
    assert set(imports) <= {"__future__", "dataclasses", "struct", "typing"}


def test_importing_parser_in_fresh_interpreter_pulls_no_ros_or_serial():
    """子进程侧证:干净解释器导入解析器不得传递拉入 ROS/pyserial。

    不用 importlib.reload:它会在共享 pytest 会话里重制模块身份,
    污染后续测试的 isinstance 判断(实测教训)。
    """
    import os
    import subprocess
    import sys

    package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    code = (
        "import sys; import go2w_bridge.nooploop_uwb_protocol; "
        "leaked = [m for m in sys.modules "
        "if m.split('.')[0] in {'rclpy', 'rclcpp', 'rospy', 'serial'}]; "
        "assert not leaked, leaked"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=package_root,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr
