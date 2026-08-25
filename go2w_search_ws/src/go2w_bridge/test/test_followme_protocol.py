"""Follow-Me 协议解析器测试 — 向量全部取自官方协议 PDF V1.0 示例。

uwb_docs/Follow-Me_Protocol_V1.0_en.pdf:
  §1.1.1 Uplink-Frame Example (MSG_SPHERICAL_RESULT, anchor)
  §2.2.2 MSG_RESULT example
  §2.6.2 MSG_DIS example
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from go2w_bridge.followme_protocol import (  # noqa: E402
    FollowMeStreamParser,
    MSG_DIS,
    MSG_RESULT,
    MSG_SPHERICAL_RESULT,
    build_spherical_frame,
    crc16_modbus,
)

# PDF §1.1.1: 官方 Uplink 示例帧 (AA 06 ... 35 DE)
OFFICIAL_FRAME = bytes.fromhex(
    "AA060483E3A165449E16002A1415CD5B070000000BCDCC0C4033335340CDCC8C40" "35DE"
)


def test_official_frame_parses():
    parser = FollowMeStreamParser()
    frames = parser.feed(OFFICIAL_FRAME)
    assert len(frames) == 1
    f = frames[0]
    assert f.role_name == "anchor"
    assert f.frame_cnt == 4
    assert f.uid == "83e3a165449e"
    assert len(f.messages) == 1
    m = f.messages[0]
    assert m["msg_id"] == MSG_SPHERICAL_RESULT
    assert abs(m["distance_m"] - 2.2) < 1e-6
    assert abs(m["azimuth_deg"] - 3.3) < 1e-6
    assert abs(m["elevation_deg"] - 4.4) < 1e-6
    assert m["local_time_us"] == 123456789
    assert m["cnt"] == 11


def test_crc_modbus_matches_official():
    # 帧体 = 去掉末 2 字节 CRC; 在线 CRC = 35 DE (LE) = 0xDE35
    calc = crc16_modbus(OFFICIAL_FRAME[:-2])
    assert calc == 0xDE35


def test_roundtrip_build_and_parse():
    frame = build_spherical_frame(
        dis_m=5.67, azimuth_deg=-42.0, elevation_deg=1.25,
        local_time_us=987654321, cnt=77)
    parser = FollowMeStreamParser()
    frames = parser.feed(frame)
    assert len(frames) == 1
    m = frames[0].messages[0]
    assert abs(m["distance_m"] - 5.67) < 1e-6
    assert abs(m["azimuth_deg"] - (-42.0)) < 1e-6
    assert abs(m["elevation_deg"] - 1.25) < 1e-6
    assert m["local_time_us"] == 987654321
    assert m["cnt"] == 77


def test_corrupted_crc_drops_frame_but_recovers_next():
    good1 = build_spherical_frame(dis_m=1.0, azimuth_deg=0.0, elevation_deg=0.0)
    good2 = build_spherical_frame(dis_m=2.0, azimuth_deg=10.0, elevation_deg=0.0,
                                  frame_cnt=5)
    bad = bytearray(good1)
    bad[14] ^= 0xFF  # 破坏 dis 字段 → CRC 失配
    parser = FollowMeStreamParser()
    frames = parser.feed(bytes(bad) + good2)
    assert len(frames) == 1
    assert abs(frames[0].messages[0]["distance_m"] - 2.0) < 1e-6
    assert parser.crc_errors == 1


def test_split_feed_across_chunks():
    frame = build_spherical_frame(dis_m=3.3, azimuth_deg=90.0, elevation_deg=0.0)
    parser = FollowMeStreamParser()
    out = []
    for i in range(0, len(frame), 3):  # 3 字节一喂
        out.extend(parser.feed(frame[i:i + 3]))
    assert len(out) == 1
    assert abs(out[0].messages[0]["azimuth_deg"] - 90.0) < 1e-6


def test_noise_before_frame_resyncs():
    frame = build_spherical_frame(dis_m=4.0, azimuth_deg=0.0, elevation_deg=0.0)
    parser = FollowMeStreamParser()
    frames = parser.feed(b"\x00\xffUW\xaa" + frame)  # 噪声里混入假 0xAA
    assert len(frames) == 1
    assert abs(frames[0].messages[0]["distance_m"] - 4.0) < 1e-6


def test_msg_dis_official_example():
    # PDF §2.6.2: 2E 0D 15CD5B070000007B A4709D3F 63
    # 构成完整帧: 手册只给消息不给整帧, 用编码器前缀拼
    from go2w_bridge.followme_protocol import crc16_modbus as _crc
    import struct
    msg = bytes([MSG_DIS, 0x0D]) + bytes.fromhex("15CD5B070000007BA4709D3F63")
    header = bytearray([0xAA, 0x06, 0x01]) + b"\x01\x02\x03\x04\x05\x06"
    header.extend(struct.pack("<H", len(msg)))
    frame = bytes(header) + msg
    frame += struct.pack("<H", _crc(frame))
    parser = FollowMeStreamParser()
    frames = parser.feed(frame)
    m = frames[0].messages[0]
    assert m["msg_id"] == MSG_DIS
    assert abs(m["distance_m"] - 1.23) < 1e-6
    assert m["prr_percent"] == 99
    assert m["local_time_us"] == 123456789
    assert m["cnt"] == 123


def test_non_followme_stream_does_not_wedge():
    # 一直喂 NLink 0x55 流不应死循环/爆缓冲
    parser = FollowMeStreamParser()
    nlink_noise = bytes([0x55, 0x02, 0x10, 0x00]) * 200
    assert parser.feed(nlink_noise) == []
    assert parser.pending_bytes <= parser._max_buffer


def test_tag_role_parsed():
    # Role=1 (Tag) 帧: func_field = 0x06<<4 | 1 = 0x61
    from go2w_bridge.followme_protocol import ROLE_TAG
    frame = build_spherical_frame(
        dis_m=1.5, azimuth_deg=0.0, elevation_deg=0.0, role=ROLE_TAG)
    parser = FollowMeStreamParser()
    frames = parser.feed(frame)
    assert frames[0].role_name == "tag"


def test_result_message_xyz_official():
    # PDF §2.2.2 MSG_RESULT 消息数据
    import struct
    from go2w_bridge.followme_protocol import crc16_modbus as _crc
    msg = bytes.fromhex(
        "1D2615CD5B070000000B0BAF353FD906B7BEBD26583EFBFFE4FFF9FF"
        "040002000200310023000900")
    header = bytearray([0xAA, 0x06, 0x02]) + b"\x11\x22\x33\x44\x55\x66"
    header.extend(struct.pack("<H", len(msg)))
    frame = bytes(header) + msg + struct.pack("<H", _crc(bytes(header) + msg))
    parser = FollowMeStreamParser()
    frames = parser.feed(frame)
    m = frames[0].messages[0]
    assert m["msg_id"] == MSG_RESULT
    assert abs(m["pos_m"][0] - 0.709702) < 1e-5
    assert abs(m["pos_m"][1] - (-0.357474)) < 1e-5
    assert abs(m["vel_mps"][0] - (-0.05)) < 1e-9
