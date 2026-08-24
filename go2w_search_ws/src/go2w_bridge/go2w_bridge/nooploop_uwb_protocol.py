"""Nooploop LinkTrack UWB 串口协议解析器(纯逻辑,无 ROS / pyserial 依赖)。

背景(B-1 串口桥):机器狗跟随钥匙扣使用 Nooploop LinkTrack UWB。
狗背基站、钥匙扣为标签;基站通过 UART 输出 NLink 协议帧。
本模块只做"字节流 -> 结构化测量"的解析,刻意不依赖 rclpy /
pyserial,以便在无 ROS2 运行时、无串口硬件的 Windows 开发机上
用 pytest 做字节流回归(工单完成标准)。

帧格式依据 Nooploop 官方 nlink_unpack(C 库, github nooploop-dev/
nlink_unpack)逐字节核对:
- LinkTrack P TagFrame0   0x55 0x01,定长 128B
- LinkTrack P NodeFrame0  0x55 0x02,帧长在第 2-3 字节(u16 LE)
- LinkTrack AOA NodeFrame0 0x55 0x07,帧长在第 2-3 字节(u16 LE)
校验和 = 除最后一字节外全部字节累加取低 8 位(NLINK_VerifyCheckSum)。

数值换算(官方 nlink_utils.h):
- int24 小端有符号(最高字节 bit7 符号扩展)
- 位置 /1000 -> m,速度 /10000 -> m/s,距离 /1000 -> m
- 角度 int16 /100 -> 度,电压 u16 /1000 -> V,RSSI u8 /-2 -> dBm
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import List, Optional, Sequence, Tuple, Union

LOGGER_NAME = "go2w.nooploop_uwb_protocol"

# --- 官方帧标识(nlink_linktrack_*.c 中的 frame_header/function_mark) ---
SYNC_BYTE = 0x55
TAG_FRAME_MARK = 0x01        # nlt_tagframe0        fixed_part_size=128
NODE_FRAME_MARK = 0x02       # nlt_nodeframe0       fixed_part_size=11
AOA_NODE_FRAME_MARK = 0x07   # nltaoa_nodeframe0    fixed_part_size=21

_KNOWN_MARKS = frozenset({TAG_FRAME_MARK, NODE_FRAME_MARK, AOA_NODE_FRAME_MARK})

# --- 定长/变长帧边界 ---
TAG_FRAME_SIZE = 128            # 0x55 0x01 定长(含校验和)
NODE_FRAME_FIXED = 11           # 0x55 0x02 固定头(不含节点与校验和)
AOA_FRAME_FIXED = 21            # 0x55 0x07 固定头(不含节点与校验和)
AOA_NODE_SIZE = 11              # AOA 帧中每个节点记录长度
NODE_TLV_HEADER = 4             # 0x55 0x02 节点 TLV 头(role,id,u16 len)
MIN_NODE_FRAME_SIZE = NODE_FRAME_FIXED + 1   # 0 节点 + 校验和
MIN_AOA_FRAME_SIZE = AOA_FRAME_FIXED + 1     # 0 节点 + 校验和
MAX_AOA_NODES = 16              # 官方 MAX_TAG_COUNT/MAX_ANCHOR_COUNT 上限
MAX_FRAME_BYTES_DEFAULT = 1024  # 防止损坏长度字段撑爆缓冲区

# --- 官方换算系数(nlink_utils.h MULTIPLY_*) ---
MULTIPLY_VOLTAGE = 1000.0
MULTIPLY_POS = 1000.0
MULTIPLY_DIS = 1000.0
MULTIPLY_VEL = 10000.0
MULTIPLY_ANGLE = 100.0
MULTIPLY_RSSI = -2.0
MULTIPLY_EOP = 100.0

# --- 角色(nlink_typedef.h linktrack_role_e) ---
ROLE_NAMES = {
    0: "node",
    1: "anchor",
    2: "tag",
    3: "console",
    4: "dt_master",
    5: "dt_slave",
    6: "monitor",
}
ROLE_TAG = 2
ROLE_CONSOLE = 3

_INT24_MIN = -(1 << 23)
_INT24_MAX = (1 << 23) - 1


class ProtocolError(ValueError):
    """NLink 帧违反协议结构(仅编码器入参使用;流解析永不抛出)。"""


def role_name(role: int) -> str:
    """角色码转官方名称;未知码返回 "role_<n>"。"""
    return ROLE_NAMES.get(int(role), f"role_{int(role)}")


def compute_checksum(payload: bytes) -> int:
    """NLink 校验和:全部字节累加取低 8 位(不含校验位自身)。"""
    return sum(payload) & 0xFF


def unpack_int24(data: bytes, offset: int = 0) -> int:
    """小端有符号 int24,等价 NLINK_ParseInt24(符号扩展)。"""
    value = data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16)
    if value & (1 << 23):
        value -= 1 << 24
    return value


def pack_int24(value: float) -> bytes:
    """浮点(已按倍率换算前的原始值四舍五入)压成小端 int24,越界截断。"""
    raw = int(round(value))
    raw = max(_INT24_MIN, min(_INT24_MAX, raw))
    return bytes((raw & 0xFF, (raw >> 8) & 0xFF, (raw >> 16) & 0xFF))


@dataclass(frozen=True)
class AoaNodeMeasurement:
    """AOA 基站帧中的单个标签节点测量(跟随功能的直接输入)。"""

    role: int
    id: int
    distance_m: float
    angle_deg: float
    fp_rssi_dbm: float
    rx_rssi_dbm: float

    @property
    def role_name_(self) -> str:
        return role_name(self.role)

    def to_dict(self) -> dict:
        return {
            "role": self.role,
            "role_name": role_name(self.role),
            "id": self.id,
            "distance_m": self.distance_m,
            "angle_deg": self.angle_deg,
            "fp_rssi_dbm": self.fp_rssi_dbm,
            "rx_rssi_dbm": self.rx_rssi_dbm,
        }


@dataclass(frozen=True)
class AoaNodeFrame:
    """0x55 0x07 LinkTrack AOA 基站节点帧(狗背基站 -> 各标签距离/角度)。"""

    device_role: int
    device_id: int
    local_time_ms: int
    system_time_ms: int
    voltage_v: float
    nodes: Tuple[AoaNodeMeasurement, ...]
    raw: bytes = b""

    kind = "aoa_node_frame"

    @property
    def device_role_name(self) -> str:
        return role_name(self.device_role)

    def find_node(self, node_id: int) -> Optional[AoaNodeMeasurement]:
        for node in self.nodes:
            if node.id == node_id:
                return node
        return None

    def to_dict(self) -> dict:
        return {
            "kind": self.kind,
            "device_role": self.device_role,
            "device_role_name": role_name(self.device_role),
            "device_id": self.device_id,
            "local_time_ms": self.local_time_ms,
            "system_time_ms": self.system_time_ms,
            "voltage_v": self.voltage_v,
            "nodes": [node.to_dict() for node in self.nodes],
        }


@dataclass(frozen=True)
class TagFrame:
    """0x55 0x01 LinkTrack P 标签数据帧(标签自身输出的全量状态)。

    距离数组为标签到 8 个基站的测距;0.0 表示该基站无有效测量
    (官方库不做有效性标记,由上层解释)。
    """

    id: int
    role: int
    position_m: Tuple[float, float, float]
    velocity_mps: Tuple[float, float, float]
    distances_m: Tuple[float, ...]
    imu_gyro: Tuple[float, float, float]
    imu_acc: Tuple[float, float, float]
    angle_deg: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]
    local_time_ms: int
    system_time_ms: int
    eop: Tuple[float, float, float]
    voltage_v: float
    raw: bytes = b""

    kind = "tag_frame"

    @property
    def role_name_(self) -> str:
        return role_name(self.role)

    def to_dict(self) -> dict:
        return {
            "kind": self.kind,
            "id": self.id,
            "role": self.role,
            "role_name": role_name(self.role),
            "position_m": list(self.position_m),
            "velocity_mps": list(self.velocity_mps),
            "distances_m": list(self.distances_m),
            "imu_gyro": list(self.imu_gyro),
            "imu_acc": list(self.imu_acc),
            "angle_deg": list(self.angle_deg),
            "quaternion": list(self.quaternion),
            "local_time_ms": self.local_time_ms,
            "system_time_ms": self.system_time_ms,
            "eop": list(self.eop),
            "voltage_v": self.voltage_v,
        }


@dataclass(frozen=True)
class NodeTlv:
    """0x55 0x02 帧中的透传节点(role,id,payload)。"""

    role: int
    id: int
    data: bytes

    @property
    def role_name_(self) -> str:
        return role_name(self.role)

    def to_dict(self) -> dict:
        return {
            "role": self.role,
            "role_name": role_name(self.role),
            "id": self.id,
            "data_hex": self.data.hex(),
        }


@dataclass(frozen=True)
class NodeFrame:
    """0x55 0x02 LinkTrack P 节点透传帧(通用容器,不做载荷解释)。"""

    device_role: int
    device_id: int
    nodes: Tuple[NodeTlv, ...]
    raw: bytes = b""

    kind = "node_frame"

    @property
    def device_role_name(self) -> str:
        return role_name(self.device_role)

    def to_dict(self) -> dict:
        return {
            "kind": self.kind,
            "device_role": self.device_role,
            "device_role_name": role_name(self.device_role),
            "device_id": self.device_id,
            "nodes": [node.to_dict() for node in self.nodes],
        }


AnyFrame = Union[AoaNodeFrame, TagFrame, NodeFrame]


@dataclass
class ParserStats:
    """流解析统计,用于桥的降级判断与验收报告。"""

    frames_parsed: int = 0
    tag_frames: int = 0
    aoa_frames: int = 0
    node_frames: int = 0
    checksum_errors: int = 0
    malformed_frames: int = 0
    dropped_noise_bytes: int = 0

    def to_dict(self) -> dict:
        return {
            "frames_parsed": self.frames_parsed,
            "tag_frames": self.tag_frames,
            "aoa_frames": self.aoa_frames,
            "node_frames": self.node_frames,
            "checksum_errors": self.checksum_errors,
            "malformed_frames": self.malformed_frames,
            "dropped_noise_bytes": self.dropped_noise_bytes,
        }


def _decode_tag_frame(frame: bytes) -> TagFrame:
    f = struct.Struct("<2sBB")
    _, tag_id, role = f.unpack_from(frame, 0)

    position = tuple(
        unpack_int24(frame, 4 + 3 * i) / MULTIPLY_POS for i in range(3)
    )
    velocity = tuple(
        unpack_int24(frame, 13 + 3 * i) / MULTIPLY_VEL for i in range(3)
    )
    distances = tuple(
        unpack_int24(frame, 22 + 3 * i) / MULTIPLY_DIS for i in range(8)
    )
    gyro = struct.unpack_from("<3f", frame, 46)
    acc = struct.unpack_from("<3f", frame, 58)
    angles = tuple(
        value / MULTIPLY_ANGLE for value in struct.unpack_from("<3h", frame, 82)
    )
    quat = struct.unpack_from("<4f", frame, 88)
    local_time, system_time = struct.unpack_from("<II", frame, 108)
    eop = tuple(
        value / MULTIPLY_EOP for value in frame[117:120]
    )
    (voltage_raw,) = struct.unpack_from("<H", frame, 120)
    return TagFrame(
        id=tag_id,
        role=role,
        position_m=position,
        velocity_mps=velocity,
        distances_m=distances,
        imu_gyro=gyro,
        imu_acc=acc,
        angle_deg=angles,
        quaternion=quat,
        local_time_ms=local_time,
        system_time_ms=system_time,
        eop=eop,
        voltage_v=voltage_raw / MULTIPLY_VOLTAGE,
        raw=bytes(frame),
    )


def _decode_aoa_frame(frame: bytes) -> Optional[AoaNodeFrame]:
    device_role, device_id = frame[4], frame[5]
    local_time, system_time = struct.unpack_from("<II", frame, 6)
    (voltage_raw,) = struct.unpack_from("<H", frame, 18)
    count = frame[20]
    # 结构校验:校验和已过,但节点数与帧长不一致视为固件异常帧
    if count > MAX_AOA_NODES or AOA_FRAME_FIXED + AOA_NODE_SIZE * count != len(frame) - 1:
        return None
    nodes = []
    for i in range(count):
        base = AOA_FRAME_FIXED + AOA_NODE_SIZE * i
        node_role, node_id = frame[base], frame[base + 1]
        distance = unpack_int24(frame, base + 2) / MULTIPLY_DIS
        (angle_raw,) = struct.unpack_from("<h", frame, base + 5)
        fp_rssi = frame[base + 7] / MULTIPLY_RSSI
        rx_rssi = frame[base + 8] / MULTIPLY_RSSI
        nodes.append(
            AoaNodeMeasurement(
                role=node_role,
                id=node_id,
                distance_m=distance,
                angle_deg=angle_raw / MULTIPLY_ANGLE,
                fp_rssi_dbm=fp_rssi,
                rx_rssi_dbm=rx_rssi,
            )
        )
    return AoaNodeFrame(
        device_role=device_role,
        device_id=device_id,
        local_time_ms=local_time,
        system_time_ms=system_time,
        voltage_v=voltage_raw / MULTIPLY_VOLTAGE,
        nodes=tuple(nodes),
        raw=bytes(frame),
    )


def _decode_node_frame(frame: bytes) -> Optional[NodeFrame]:
    device_role, device_id = frame[4], frame[5]
    count = frame[10]
    nodes: List[NodeTlv] = []
    address = NODE_FRAME_FIXED
    limit = len(frame) - 1  # 最后一字节为校验和
    for _ in range(count):
        if address + NODE_TLV_HEADER > limit:
            return None  # TLV 头越界:结构损坏
        node_role, node_id = frame[address], frame[address + 1]
        data_length = frame[address + 2] | (frame[address + 3] << 8)
        address += NODE_TLV_HEADER
        if address + data_length > limit:
            return None  # TLV 载荷越界:结构损坏
        nodes.append(NodeTlv(role=node_role, id=node_id, data=bytes(frame[address:address + data_length])))
        address += data_length
    return NodeFrame(
        device_role=device_role,
        device_id=device_id,
        nodes=tuple(nodes),
        raw=bytes(frame),
    )


class NLinkStreamParser:
    """增量式 NLink 字节流解析器:任意切分/噪声/损坏均不丢后续帧。

    用法:parser.feed(serial_bytes) -> [TagFrame|AoaNodeFrame|NodeFrame]
    线程模型:单线程调用 feed(桥的读线程);不内置锁。
    """

    def __init__(self, max_frame_bytes: int = MAX_FRAME_BYTES_DEFAULT) -> None:
        if max_frame_bytes < TAG_FRAME_SIZE:
            raise ValueError("max_frame_bytes must be >= TAG_FRAME_SIZE(128)")
        self._max_frame_bytes = max_frame_bytes
        self._buffer = bytearray()
        self.stats = ParserStats()

    @property
    def pending_bytes(self) -> int:
        """已缓冲但尚未构成完整帧的字节数(诊断用)。"""
        return len(self._buffer)

    def feed(self, data: bytes) -> List[AnyFrame]:
        """喂入任意长度字节(可为半个帧),返回本次解析出的完整帧列表。"""
        self._buffer.extend(data)
        frames: List[AnyFrame] = []
        consumed = 0
        buffer = self._buffer
        while True:
            # 1. 跳到下一个同步字节 0x55;之前的都算噪声
            sync_index = buffer.find(SYNC_BYTE, consumed)
            if sync_index < 0:
                # [consumed:] 内无同步字节:全部为噪声,清空整个缓冲
                # (只删前缀会把噪声尾巴留在缓冲里,导致重复计数与滞留)
                self.stats.dropped_noise_bytes += len(buffer) - consumed
                del buffer[:]
                return frames
            if sync_index > consumed:
                self.stats.dropped_noise_bytes += sync_index - consumed
                consumed = sync_index
            available = len(buffer) - consumed
            # 2. 帧类型字节尚未到达:保留等待
            if available < 2:
                del buffer[:consumed]
                return frames
            mark = buffer[consumed + 1]
            if mark not in _KNOWN_MARKS:
                # 0x55 后不是已知帧类型:视为孤立的 0x55 噪声,前进 1 字节重同步
                self.stats.dropped_noise_bytes += 1
                consumed += 1
                continue
            # 3. 确定帧长
            if mark == TAG_FRAME_MARK:
                frame_length = TAG_FRAME_SIZE
            else:
                if available < 4:
                    del buffer[:consumed]
                    return frames  # 长度字段未到,等待
                frame_length = buffer[consumed + 2] | (buffer[consumed + 3] << 8)
                minimum = (
                    MIN_AOA_FRAME_SIZE
                    if mark == AOA_NODE_FRAME_MARK
                    else MIN_NODE_FRAME_SIZE
                )
                if frame_length < minimum or frame_length > self._max_frame_bytes:
                    # 损坏的长度字段:丢弃这个假同步头,前进 1 字节重同步
                    self.stats.dropped_noise_bytes += 1
                    consumed += 1
                    continue
            # 4. 整帧未到:等待更多数据
            if available < frame_length:
                del buffer[:consumed]
                return frames
            candidate = bytes(buffer[consumed:consumed + frame_length])
            # 5. 校验和
            if compute_checksum(candidate[:-1]) != candidate[-1]:
                self.stats.checksum_errors += 1
                self.stats.dropped_noise_bytes += 1
                consumed += 1  # 只丢这个 0x55,允许帧内重叠重同步
                continue
            # 6. 结构解码
            if mark == TAG_FRAME_MARK:
                frame: Optional[AnyFrame] = _decode_tag_frame(candidate)
            elif mark == AOA_NODE_FRAME_MARK:
                frame = _decode_aoa_frame(candidate)
            else:
                frame = _decode_node_frame(candidate)
            if frame is None:
                # 校验和通过但结构矛盾:帧长可信,整帧跳过
                self.stats.malformed_frames += 1
                consumed += frame_length
                continue
            frames.append(frame)
            self.stats.frames_parsed += 1
            if isinstance(frame, TagFrame):
                self.stats.tag_frames += 1
            elif isinstance(frame, AoaNodeFrame):
                self.stats.aoa_frames += 1
            else:
                self.stats.node_frames += 1
            consumed += frame_length

    def reset(self) -> None:
        """清空缓冲与统计(串口重连时调用)。"""
        self._buffer.clear()
        self.stats = ParserStats()


# ---------------------------------------------------------------------------
# 编码器:构造与真机逐字节同构的帧,供字节流测试与 mock 注入路径使用。
# ---------------------------------------------------------------------------

def build_tag_frame(
    *,
    tag_id: int = 1,
    role: int = ROLE_TAG,
    position_m: Sequence[float] = (0.0, 0.0, 0.0),
    velocity_mps: Sequence[float] = (0.0, 0.0, 0.0),
    distances_m: Sequence[float] = (0.0,) * 8,
    imu_gyro: Sequence[float] = (0.0, 0.0, 0.0),
    imu_acc: Sequence[float] = (0.0, 0.0, 0.0),
    angle_deg: Sequence[float] = (0.0, 0.0, 0.0),
    quaternion: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    local_time_ms: int = 0,
    system_time_ms: int = 0,
    eop: Sequence[float] = (0.0, 0.0, 0.0),
    voltage_v: float = 3.7,
) -> bytes:
    """构造 0x55 0x01 定长标签帧(自动填 0 保留区并计算校验和)。"""
    if len(distances_m) != 8:
        raise ProtocolError("distances_m must contain exactly 8 entries")
    frame = bytearray(TAG_FRAME_SIZE)
    frame[0] = SYNC_BYTE
    frame[1] = TAG_FRAME_MARK
    frame[2] = tag_id & 0xFF
    frame[3] = role & 0xFF
    for i in range(3):
        frame[4 + 3 * i:7 + 3 * i] = pack_int24(position_m[i] * MULTIPLY_POS)
        frame[13 + 3 * i:16 + 3 * i] = pack_int24(velocity_mps[i] * MULTIPLY_VEL)
    for i in range(8):
        frame[22 + 3 * i:25 + 3 * i] = pack_int24(distances_m[i] * MULTIPLY_DIS)
    struct.pack_into("<3f", frame, 46, *imu_gyro)
    struct.pack_into("<3f", frame, 58, *imu_acc)
    struct.pack_into(
        "<3h", frame, 82,
        *(int(round(value * MULTIPLY_ANGLE)) for value in angle_deg),
    )
    struct.pack_into("<4f", frame, 88, *quaternion)
    struct.pack_into("<II", frame, 108, local_time_ms & 0xFFFFFFFF, system_time_ms & 0xFFFFFFFF)
    for i in range(3):
        frame[117 + i] = int(round(eop[i] * MULTIPLY_EOP)) & 0xFF
    struct.pack_into("<H", frame, 120, int(round(voltage_v * MULTIPLY_VOLTAGE)))
    frame[TAG_FRAME_SIZE - 1] = compute_checksum(bytes(frame[:-1]))
    return bytes(frame)


@dataclass(frozen=True)
class AoaNodeSpec:
    """build_aoa_node_frame 的节点入参。"""

    id: int
    role: int = ROLE_TAG
    distance_m: float = 3.0
    angle_deg: float = 0.0
    fp_rssi_dbm: float = -60.0
    rx_rssi_dbm: float = -70.0


def build_aoa_node_frame(
    *,
    nodes: Sequence[AoaNodeSpec] = (),
    device_role: int = ROLE_CONSOLE,
    device_id: int = 0,
    local_time_ms: int = 0,
    system_time_ms: int = 0,
    voltage_v: float = 3.7,
    node_list: Optional[Sequence[AoaNodeSpec]] = None,
) -> bytes:
    """构造 0x55 0x07 AOA 基站节点帧(跟随场景的主要数据源)。"""
    if node_list is not None:  # 兼容旧关键字
        nodes = node_list
    if len(nodes) > MAX_AOA_NODES:
        raise ProtocolError(f"at most {MAX_AOA_NODES} nodes per frame")
    data_length = AOA_FRAME_FIXED + AOA_NODE_SIZE * len(nodes) + 1
    frame = bytearray(data_length)
    frame[0] = SYNC_BYTE
    frame[1] = AOA_NODE_FRAME_MARK
    struct.pack_into("<H", frame, 2, data_length)
    frame[4] = device_role & 0xFF
    frame[5] = device_id & 0xFF
    struct.pack_into("<II", frame, 6, local_time_ms & 0xFFFFFFFF, system_time_ms & 0xFFFFFFFF)
    struct.pack_into("<H", frame, 18, int(round(voltage_v * MULTIPLY_VOLTAGE)))
    frame[20] = len(nodes)
    for i, node in enumerate(nodes):
        base = AOA_FRAME_FIXED + AOA_NODE_SIZE * i
        frame[base] = node.role & 0xFF
        frame[base + 1] = node.id & 0xFF
        frame[base + 2:base + 5] = pack_int24(node.distance_m * MULTIPLY_DIS)
        struct.pack_into(
            "<h", frame, base + 5,
            int(round(node.angle_deg * MULTIPLY_ANGLE)),
        )
        frame[base + 7] = int(round(node.fp_rssi_dbm * MULTIPLY_RSSI)) & 0xFF
        frame[base + 8] = int(round(node.rx_rssi_dbm * MULTIPLY_RSSI)) & 0xFF
        # base+9, base+10 保留
    frame[-1] = compute_checksum(bytes(frame[:-1]))
    return bytes(frame)


def build_node_frame(
    *,
    entries: Sequence[Tuple[int, int, bytes]] = (),
    device_role: int = ROLE_CONSOLE,
    device_id: int = 0,
) -> bytes:
    """构造 0x55 0x02 节点透传帧;entries = [(role, id, payload), ...]。"""
    payload_size = sum(NODE_TLV_HEADER + len(data) for _, _, data in entries)
    data_length = NODE_FRAME_FIXED + payload_size + 1
    frame = bytearray(data_length)
    frame[0] = SYNC_BYTE
    frame[1] = NODE_FRAME_MARK
    struct.pack_into("<H", frame, 2, data_length)
    frame[4] = device_role & 0xFF
    frame[5] = device_id & 0xFF
    frame[10] = len(entries)
    address = NODE_FRAME_FIXED
    for role, node_id, data in entries:
        frame[address] = role & 0xFF
        frame[address + 1] = node_id & 0xFF
        struct.pack_into("<H", frame, address + 2, len(data))
        frame[address + NODE_TLV_HEADER:address + NODE_TLV_HEADER + len(data)] = data
        address += NODE_TLV_HEADER + len(data)
    frame[-1] = compute_checksum(bytes(frame[:-1]))
    return bytes(frame)
