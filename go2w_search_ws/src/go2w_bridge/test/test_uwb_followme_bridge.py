def test_followme_protocol_end_to_end_snapshot():
    """Follow-Me 帧注入 → 桥 latest() 统一快照 → 适配层 fix 合同。

    真机链路: FMM-A01 基站串口(921600) → FollowMeStreamParser → 快照 →
    nx_uwb_bridge._FollowFixSource.get_fix()。此测试锁三层衔接。
    """
    from go2w_bridge.followme_protocol import build_spherical_frame
    from go2w_bridge.uwb_serial_bridge import UwbBridgeConfig, UwbSerialBridge

    bridge = UwbSerialBridge(UwbBridgeConfig(
        mode="mock", protocol="followme", followed_tag_id=1))
    frame = build_spherical_frame(dis_m=4.2, azimuth_deg=15.0, elevation_deg=0.0)
    bridge.inject_bytes(frame)

    snap = bridge.latest(tag_id=1)
    assert snap is not None
    assert snap["kind"] == "followme_spherical"
    assert abs(snap["distance_m"] - 4.2) < 1e-6
    assert abs(snap["angle_deg"] - 15.0) < 1e-6
    assert snap["received_monotonic"] > 0.0

    status = bridge.status()
    assert status["protocol"] == "followme"
    assert status["followme_frames"] >= 1

    # 适配层 fix 合同 (与 web 层同一转换)
    import sys, os
    sys.path.insert(0, os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "web"))
    import nx_uwb_bridge as nb  # noqa: E402

    source = nb._FollowFixSource(bridge, angle_offset_deg=0.0)
    fix = source.get_fix()
    assert fix["ok"] is True
    assert abs(fix["range_m"] - 4.2) < 1e-6
    import math
    assert abs(fix["azimuth_rad"] - math.radians(15.0)) < 1e-9
    assert fix["age_sec"] < 1.0


def test_protocol_auto_locks_on_first_valid_frame():
    """auto 模式: 喂 Follow-Me 帧后锁定 followme, 再喂 NLink 帧不解析。"""
    from go2w_bridge.followme_protocol import build_spherical_frame
    from go2w_bridge.uwb_serial_bridge import UwbBridgeConfig, UwbSerialBridge
    from go2w_bridge.nooploop_uwb_protocol import (
        AoaNodeSpec, ROLE_CONSOLE, build_aoa_node_frame)

    bridge = UwbSerialBridge(UwbBridgeConfig(
        mode="mock", protocol="auto", followed_tag_id=1))
    fm = build_spherical_frame(dis_m=2.0, azimuth_deg=0.0, elevation_deg=0.0)
    nl = build_aoa_node_frame(
        nodes=[AoaNodeSpec(id=1, distance_m=9.9, angle_deg=0.0)],
        device_role=ROLE_CONSOLE, device_id=0)
    bridge.inject_bytes(fm)
    bridge.inject_bytes(nl)  # 锁定后 NLink 帧不应产出
    assert bridge.status()["protocol"] == "followme"
    snap = bridge.latest(tag_id=1)
    assert snap is not None
    # Follow-Me 帧生效 (2.0m), NLink 帧 (9.9m) 未覆盖快照
    assert abs(snap["distance_m"] - 2.0) < 1e-6

