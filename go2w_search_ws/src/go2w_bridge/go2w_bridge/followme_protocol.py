"""Nooploop Follow-Me 协议解析器 (真机硬件: FMM-A01 基站 + FMM-T01 标签)。

2026-08-24 硬件确认后新增。Follow-Me 是与 LinkTrack NLink 完全不同的新协议:
  - 帧头 0xAA (NLink 是 0x55), 校验 CRC-16/Modbus (NLink 是累加和低 8 位),
    帧内是"消息"结构 (一帧可含多条消息)。
协议来源: Follow-Me Protocol V1.0 (官方 PDF, 已下载存 uwb_docs/)。

== 帧格式 (Uplink, 基站→主机) ==
  AA [FuncField] [FrameCNT u8] [UID 6B] [PayloadSize u16 LE] [Messages...] [CRC16 LE]
  FuncField = SystemType(高4位, 固定 0x06) | Role(低4位, Anchor=0 Tag=1)
  CRC-16/Modbus 覆盖 帧头→Payload 末 (不含 CRC 自身), 小端在线。

== 消息 (与跟随相关的) ==
  MSG_SPHERICAL_RESULT 0x2A (基站→主机, 我们的主消费):
      local_time u56(us) + cnt u8 + dis f32(m) + azimuth f32(deg) + elevation f32(deg)
  MSG_RESULT 0x1D (基站输出直角坐标 xyz + 速度, 备用)
  MSG_DIS 0x2E (纯距离 + PRR)

== 角度约定 (用户手册 §3) ==
  基站指示灯/按键方向 = X+ = 方位角 0°, 航空插头方向 = ±180°, Z 轴朝上,
  右手系 → 方位角逆时针为正。安装要求 X+ 朝狗头正前方时, 方位角与 REP-103
  的 "+左" 约定一致, 无需符号翻转 (装反 180° 时用 GO2W_UWB_ANGLE_OFFSET_DEG=180 补偿)。

纯逻辑零 ROS/pyserial 依赖; 测试向量取自协议 PDF 的官方示例帧。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Dict, List, Optional

FRAME_HEADER = 0xAA
SYSTEM_TYPE_FOLLOW_ME = 0x06
ROLE_ANCHOR = 0
ROLE_TAG = 1

MSG_RESULT = 0x1D
MSG_PREV_RESULT = 0x1E
MSG_SPHERICAL_RESULT = 0x2A
MSG_PREV_SPHERICAL_RESULT = 0x2B
MSG_DATA_USER_TO_USER = 0x24
MSG_DIS = 0x2E

# 帧固定前缀: AA + FuncField + CNT + UID(6) + PayloadSize(2) = 11 字节
_FRAME_PREFIX_LEN = 11
_MAX_PAYLOAD = 242  # 协议规定 payload 上限


def crc16_modbus(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/Modbus: poly 0x8005, init 0xFFFF, reflect in/out, xorout 0。

    逐位实现 (查表法对 25Hz 小帧无性能意义, 位实现零状态便于增量校验)。
    """
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001  # 0x8005 反射多项式
            else:
                crc >>= 1
    return crc & 0xFFFF


@dataclass(frozen=True)
class SphericalMeasurement:
    """MSG_SPHERICAL_RESULT 的一次解算 (基站观测标签)。"""

    distance_m: float
    azimuth_deg: float
    elevation_deg: float
    local_time_us: int
    cnt: int

    def to_dict(self) -> dict:
        return {
            "kind": "followme_spherical",
            "distance_m": self.distance_m,
            "azimuth_deg": self.azimuth_deg,
            "elevation_deg": self.elevation_deg,
            "local_time_us": self.local_time_us,
            "cnt": self.cnt,
        }


@dataclass(frozen=True)
class FollowMeFrame:
    """一条完整 Uplink 帧 (可能携带多条消息)。"""

    role: int
    role_name: str
    frame_cnt: int
    uid: str
    messages: List[dict] = field(default_factory=list)
    raw: bytes = b""

    def to_dict(self) -> dict:
        return {
            "role": self.role,
            "role_name": self.role_name,
            "frame_cnt": self.frame_cnt,
            "uid": self.uid,
            "messages": self.messages,
        }


def _role_name(role: int) -> str:
    return "anchor" if role == ROLE_ANCHOR else "tag"


def _parse_message(msg_id: int, payload: bytes) -> Optional[dict]:
    """按协议字段表解一条消息; 未知/长度不符返回 None (帧 CRC 已保证字节
    完整性, 长度不符说明固件版本与本解析器不匹配, 丢弃优于猜测)。"""
    if msg_id == MSG_SPHERICAL_RESULT:
        # local_time u56 + cnt u8 + dis f32 + azimuth f32 + elevation f32 = 20
        # u56+u8 打包进 u64 时 local_time 占低 56 位, cnt 占高 8 位
        # (官方示例: 15CD5B070000000B → time=0x075BCD15=123456789us, cnt=0x0B=11)
        if len(payload) < 20:
            return None
        time_cnt = struct.unpack_from("<Q", payload, 0)[0]
        local_time_us = time_cnt & 0x00FFFFFFFFFFFFFF
        cnt = time_cnt >> 56
        dis, azimuth, elevation = struct.unpack_from("<fff", payload, 8)
        return SphericalMeasurement(
            distance_m=dis, azimuth_deg=azimuth, elevation_deg=elevation,
            local_time_us=local_time_us, cnt=cnt,
        ).to_dict()
    if msg_id == MSG_DIS:
        # time_cnt(8) + dis(4) + PRR(1) = 13
        if len(payload) < 13:
            return None
        time_cnt = struct.unpack_from("<Q", payload, 0)[0]
        local_time_us = time_cnt & 0x00FFFFFFFFFFFFFF
        cnt = time_cnt >> 56
        (dis,) = struct.unpack_from("<f", payload, 8)
        prr = payload[12]
        return {
            "kind": "followme_dis",
            "distance_m": dis,
            "prr_percent": prr,
            "local_time_us": local_time_us,
            "cnt": cnt,
        }
    if msg_id == MSG_RESULT:
        if len(payload) < 38:
            return None
        time_cnt = struct.unpack_from("<Q", payload, 0)[0]
        local_time_us = time_cnt & 0x00FFFFFFFFFFFFFF
        cnt = time_cnt >> 56
        pos = struct.unpack_from("<fff", payload, 8)
        vel_raw = struct.unpack_from("<hhh", payload, 20)
        return {
            "kind": "followme_result",
            "pos_m": list(pos),
            "vel_mps": [v / 100.0 for v in vel_raw],
            "local_time_us": local_time_us,
            "cnt": cnt,
        }
    if msg_id == MSG_PREV_RESULT:
        if len(payload) < 13:
            return None
        cnt = payload[0]
        pos = struct.unpack_from("<fff", payload, 1)
        return {
            "kind": "followme_prev_result",
            "pos_m": list(pos),
            "cnt": cnt,
        }
    if msg_id == MSG_PREV_SPHERICAL_RESULT:
        if len(payload) < 13:
            return None
        cnt = payload[0]
        dis, azimuth, elevation = struct.unpack_from("<fff", payload, 1)
        return {
            "kind": "followme_prev_spherical",
            "distance_m": dis,
            "azimuth_deg": azimuth,
            "elevation_deg": elevation,
            "cnt": cnt,
        }
    return None


class FollowMeStreamParser:
    """增量字节流解析器 (喂任意切分/含噪声的 bytes, 吐完整帧)。

    与 NLinkStreamParser 同一使用模式: feed() 返回本批解出的帧列表;
    校验失败重同步扫描, 不影响后续好帧; 内部缓冲有上限保护。
    """

    def __init__(self, max_buffer_bytes: int = 4096) -> None:
        self._buffer = bytearray()
        self._max_buffer = max(int(max_buffer_bytes), 256)
        self.frames_parsed = 0
        self.crc_errors = 0

    def feed(self, data: bytes) -> List[FollowMeFrame]:
        self._buffer.extend(data)
        frames: List[FollowMeFrame] = []
        while True:
            frame = self._try_parse_one()
            if frame is None:
                break
            frames.append(frame)
        self._trim_buffer()
        return frames

    # ------------------------------------------------------------------
    def _try_parse_one(self) -> Optional[FollowMeFrame]:
        # 重同步循环: 假帧头/坏 CRC 只丢 1 字节继续扫, 不放弃本批后续好帧
        # (实测踩坑: 初版坏帧后直接 return None, feed 外层 break, 同批喂入的
        # 下一好帧被留在缓冲里等下一次 feed — 单次大块喂入场景永远丢帧)。
        while True:
            idx = self._buffer.find(bytes([FRAME_HEADER]))
            if idx < 0:
                # 无帧头: 保留末 1 字节 (可能是跨包的 AA)
                if len(self._buffer) > 1:
                    del self._buffer[:-1]
                return None
            if idx > 0:
                del self._buffer[:idx]
            if len(self._buffer) < _FRAME_PREFIX_LEN:
                return None  # 半帧, 等更多字节
            func_field = self._buffer[1]
            # 官方示例帧 (PDF §1.1.1): Anchor 帧 FuncField=0x06, 即
            # Role 在高 4 位 (Anchor=0), SystemType 0x06 在低 4 位。
            # (实测踩坑: 初版按表序猜 sys 高位/role 低位, 与示例帧 0x06 矛盾。)
            system_type = func_field & 0x0F
            role = func_field >> 4
            if system_type != SYSTEM_TYPE_FOLLOW_ME:
                # 不是 Follow-Me 帧 (可能是 NLink 0x55 流里的数据): 跳过这个 AA
                del self._buffer[:1]
                continue
            payload_size = struct.unpack_from("<H", self._buffer, 9)[0]
            if payload_size > _MAX_PAYLOAD:
                del self._buffer[:1]
                continue
            total = _FRAME_PREFIX_LEN + payload_size + 2  # +CRC16
            if len(self._buffer) < total:
                return None
            frame_bytes = bytes(self._buffer[:total])
            crc_wire = struct.unpack_from("<H", frame_bytes, total - 2)[0]
            crc_calc = crc16_modbus(frame_bytes[: total - 2])
            if crc_wire != crc_calc:
                # CRC 失败: 帧损坏 (或 AA 是数据里的假帧头), 从下一字节继续扫描。
                # 丢弃范围 = 这个 AA (而非整帧), 保证真帧头恰好嵌在坏帧尾部时仍可恢复。
                self.crc_errors += 1
                del self._buffer[:1]
                continue
            del self._buffer[:total]
            break  # 跳出重同步循环, 解析这条好帧


        frame_cnt = frame_bytes[2]
        uid = frame_bytes[3:9].hex()
        messages: List[dict] = []
        offset = _FRAME_PREFIX_LEN
        payload_end = _FRAME_PREFIX_LEN + payload_size
        while offset + 2 <= payload_end:
            msg_id = frame_bytes[offset]
            msg_size = frame_bytes[offset + 1]
            offset += 2
            if offset + msg_size > payload_end:
                break  # 消息声明越界: 放弃本帧剩余消息 (CRC 已过, 属固件异常)
            parsed = _parse_message(msg_id, frame_bytes[offset:offset + msg_size])
            if parsed is not None:
                parsed["msg_id"] = msg_id
                messages.append(parsed)
            offset += msg_size
        self.frames_parsed += 1
        return FollowMeFrame(
            role=role, role_name=_role_name(role), frame_cnt=frame_cnt,
            uid=uid, messages=messages, raw=frame_bytes,
        )

    def _trim_buffer(self) -> None:
        if len(self._buffer) > self._max_buffer:
            # 缓冲爆满仍无完整帧: 流不是 Follow-Me 协议, 保留尾部重扫
            del self._buffer[:-64]

    @property
    def pending_bytes(self) -> int:
        return len(self._buffer)


# ----------------------------------------------------------------------
# 编码器 (测试/回放辅助; 生产数据只来自串口)
# ----------------------------------------------------------------------

def build_spherical_frame(
    *,
    dis_m: float,
    azimuth_deg: float,
    elevation_deg: float,
    local_time_us: int = 123456789,
    cnt: int = 11,
    role: int = ROLE_ANCHOR,
    uid: bytes = bytes.fromhex("83e3a165449e"),
    frame_cnt: int = 4,
) -> bytes:
    """构造一条 MSG_SPHERICAL_RESULT Uplink 帧 (含正确 CRC)。"""
    payload = bytearray()
    payload.append(MSG_SPHERICAL_RESULT)
    time_cnt = ((cnt & 0xFF) << 56) | (local_time_us & 0x00FFFFFFFFFFFFFF)
    msg_body = struct.pack("<Q", time_cnt) + struct.pack(
        "<fff", dis_m, azimuth_deg, elevation_deg)
    payload.append(len(msg_body))
    payload.extend(msg_body)

    header = bytearray([FRAME_HEADER, ((role & 0x0F) << 4) | SYSTEM_TYPE_FOLLOW_ME,
                        frame_cnt & 0xFF])
    header.extend(uid if len(uid) == 6 else b"\x00" * 6)
    header.extend(struct.pack("<H", len(payload)))
    frame = bytes(header) + bytes(payload)
    crc = crc16_modbus(frame)
    return frame + struct.pack("<H", crc)
