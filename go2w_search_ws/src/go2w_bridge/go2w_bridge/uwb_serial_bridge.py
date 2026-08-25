"""Nooploop UWB 串口桥:串口读线程 + mock 注入路径(无 ROS 依赖)。

工单 B-1 要求:串口不可用(无硬件 / 无 pyserial / 打开失败)时必须
走 mock 注入路径,且 mock 路径与真机路径共用同一个 NLinkStreamParser,
保证下游跟随逻辑(功能 B-2)拿到的是协议同构数据,而不是旁路假对象。

数据流:
  serial.Serial(懒加载 pyserial)──read──┐
  MockUwbSource(轨迹函数生成 AOA 帧)──┼─> NLinkStreamParser.feed()
  inject_bytes()/inject_tag()(测试注入)┘        |
                                                 v
                                   on_frame / on_tag 回调 + latest() 快照

本模块刻意不 import rclpy(无 ROS2 运行时可测);eC/eI 在其上
自行包 ROS 节点或直接作为库使用。
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import threading
import time
from typing import Callable, Dict, List, Optional

from go2w_bridge.nooploop_uwb_protocol import (
    AoaNodeFrame,
    AoaNodeMeasurement,
    AoaNodeSpec,
    AnyFrame,
    NLinkStreamParser,
    ROLE_CONSOLE,
    build_aoa_node_frame,
)

LOGGER = logging.getLogger("go2w.uwb_serial_bridge")

DEFAULT_BAUDRATE = 115200  # LinkTrack 出厂默认
DEFAULT_MOCK_HZ = 50.0     # 与真机 20-50Hz 输出量级一致

FrameCallback = Callable[[AnyFrame], None]
MeasurementCallback = Callable[[int, Dict], None]


@dataclass
class UwbBridgeConfig:
    """串口桥配置(环境变量映射由上层 nx 节点负责)。"""

    port: Optional[str] = None          # None/空 => 自动降级 mock
    baudrate: int = DEFAULT_BAUDRATE
    mode: str = "auto"                  # auto | serial | mock
    mock_hz: float = DEFAULT_MOCK_HZ
    followed_tag_id: Optional[int] = None
    max_frame_bytes: int = 1024
    read_chunk: int = 256

    def __post_init__(self) -> None:
        if self.mode not in ("auto", "serial", "mock"):
            raise ValueError(f"unsupported bridge mode: {self.mode!r}")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if self.mock_hz <= 0:
            raise ValueError("mock_hz must be positive")


class MockUwbSource:
    """mock 数据源:按轨迹函数生成 AOA 基站帧字节。

    scenario(t 秒) -> (distance_m, angle_deg);默认静止 3m 正前方。
    用编码器构造真实帧字节(而非直接构造对象),保证 mock 注入路径
    与真机串口路径经过完全相同的解析器与校验逻辑。
    """

    def __init__(
        self,
        tag_id: int = 1,
        base_id: int = 0,
        hz: float = DEFAULT_MOCK_HZ,
        scenario: Optional[Callable[[float], tuple]] = None,
    ) -> None:
        self.tag_id = tag_id
        self.base_id = base_id
        self.hz = hz
        self._scenario = scenario or (lambda t: (3.0, 0.0))
        self._start = time.monotonic()
        self._tick = 0

    def next_frame_bytes(self) -> bytes:
        distance_m, angle_deg = self._scenario(time.monotonic() - self._start)
        self._tick += 1
        return build_aoa_node_frame(
            nodes=[AoaNodeSpec(id=self.tag_id, distance_m=distance_m, angle_deg=angle_deg)],
            device_role=ROLE_CONSOLE,
            device_id=self.base_id,
            local_time_ms=int(self._tick * 1000.0 / self.hz),
            system_time_ms=int(time.time() * 1000) & 0xFFFFFFFF,
        )


class UwbSerialBridge:
    """UWB 串口桥。open() 决定数据源(串口或 mock),永不抛出。"""

    def __init__(
        self,
        config: Optional[UwbBridgeConfig] = None,
        on_frame: Optional[FrameCallback] = None,
        on_tag: Optional[MeasurementCallback] = None,
    ) -> None:
        self.config = config or UwbBridgeConfig()
        self._parser = NLinkStreamParser(max_frame_bytes=self.config.max_frame_bytes)
        self._on_frame = on_frame
        self._on_tag = on_tag
        self._serial = None
        self._mock_source: Optional[MockUwbSource] = None
        self._source: Optional[str] = None          # "serial" | "mock"
        self._fallback_reason: Optional[str] = None  # 降级原因(诊断/验收)
        self._latest: Dict[int, Dict] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ 源
    def open(self) -> str:
        """按 mode 选择数据源;失败自动降级 mock 并返回源名。"""
        if self._source is not None:
            return self._source
        if self.config.mode != "mock":
            serial_source = self._try_open_serial()
            if serial_source is not None:
                self._source = "serial"
                LOGGER.info("UWB bridge opened serial port %s @ %d", self.config.port, self.config.baudrate)
                return self._source
        self._source = "mock"
        self._mock_source = MockUwbSource(
            tag_id=self.config.followed_tag_id if self.config.followed_tag_id is not None else 1,
            hz=self.config.mock_hz,
        )
        self._fallback_reason = self._fallback_reason or "mode=mock"
        LOGGER.warning("UWB bridge fell back to mock source: %s", self._fallback_reason)
        return self._source

    def _try_open_serial(self):
        """懒加载 pyserial 并打开串口;任何失败记入 _fallback_reason。"""
        port = self.config.port
        if not port:
            self._fallback_reason = "no serial port configured"
            return None
        try:
            import serial  # 懒加载:开发机无 pyserial 时降级 mock
        except ImportError as exc:
            self._fallback_reason = f"pyserial unavailable: {exc}"
            return None
        try:
            self._serial = serial.Serial(port=port, baudrate=self.config.baudrate, timeout=0.05)
            return self._serial
        except Exception as exc:  # 串口打开失败(占用/不存在)一律降级
            self._fallback_reason = f"cannot open {port}: {exc}"
            self._serial = None
            return None

    # ------------------------------------------------------------- 注入路径
    def inject_bytes(self, data: bytes) -> List[AnyFrame]:
        """mock 注入路径入口:原始字节流经真解析器(线程安全)。"""
        with self._lock:
            frames = self._parser.feed(data)
        for frame in frames:
            self._dispatch(frame)
        return frames

    def inject_tag(
        self,
        tag_id: int,
        distance_m: float,
        angle_deg: float = 0.0,
        local_time_ms: Optional[int] = None,
    ) -> List[AnyFrame]:
        """注入一条"钥匙扣标签测量"(构造协议帧再走解析器)。"""
        frame = build_aoa_node_frame(
            nodes=[AoaNodeSpec(id=tag_id, distance_m=distance_m, angle_deg=angle_deg)],
            local_time_ms=0 if local_time_ms is None else local_time_ms,
        )
        return self.inject_bytes(frame)

    # --------------------------------------------------------------- 分发
    def _dispatch(self, frame: AnyFrame) -> None:
        self._record_latest(frame)
        if self._on_frame is not None:
            self._safe_call(self._on_frame, frame)
        if self._on_tag is None:
            return
        payload = frame.to_dict()
        if isinstance(frame, AoaNodeFrame):
            for node in frame.nodes:
                if self.config.followed_tag_id is None or node.id == self.config.followed_tag_id:
                    self._safe_call(self._on_tag, node.id, node.to_dict())
        elif payload.get("kind") == "tag_frame":
            tag_id = payload.get("id")
            if self.config.followed_tag_id is None or tag_id == self.config.followed_tag_id:
                self._safe_call(self._on_tag, tag_id, payload)

    def _record_latest(self, frame: AnyFrame) -> None:
        # 到达时刻 (monotonic): 消费者计算 age 的唯一可靠基准。协议里的
        # system_time_ms 是 uint32 墙钟毫秒, &0xFFFFFFFF 后每 ~49.7 天回绕,
        # 直接相减会得到天文数字年龄 (集成期实测踩坑), 不得用于新鲜度。
        received_monotonic = time.monotonic()
        if isinstance(frame, AoaNodeFrame):
            for node in frame.nodes:
                snapshot = node.to_dict()
                snapshot["local_time_ms"] = frame.local_time_ms
                snapshot["system_time_ms"] = frame.system_time_ms
                snapshot["received_monotonic"] = received_monotonic
                self._latest[node.id] = snapshot
        elif frame.to_dict().get("kind") == "tag_frame":
            payload = frame.to_dict()
            snapshot = dict(payload)
            snapshot["distance_m"] = next(
                (d for d in frame.distances_m if d > 0.0), None
            )
            snapshot["received_monotonic"] = received_monotonic
            self._latest[frame.id] = snapshot

    def _safe_call(self, callback, *args) -> None:
        try:
            callback(*args)
        except Exception:  # 回调异常不得杀死读线程(长驻进程要求)
            LOGGER.exception("UWB bridge callback raised")

    # ---------------------------------------------------------------- 快照
    def latest(self, tag_id: Optional[int] = None) -> Optional[Dict]:
        """最近一次测量快照;不传 tag_id 时取 followed_tag_id。"""
        key = tag_id if tag_id is not None else self.config.followed_tag_id
        if key is None:
            return None
        return self._latest.get(key)

    def status(self) -> Dict:
        """桥状态(供健康检查/验收报告)。"""
        return {
            "source": self._source,
            "fallback_reason": self._fallback_reason,
            "port": self.config.port,
            "baudrate": self.config.baudrate,
            "mode": self.config.mode,
            "followed_tag_id": self.config.followed_tag_id,
            "stats": self._parser.stats.to_dict(),
            "pending_bytes": self._parser.pending_bytes,
        }

    # ---------------------------------------------------------------- 运行
    def start(self) -> str:
        """后台线程运行读循环(串口或 mock)。幂等。"""
        self.open()
        if self._thread is not None and self._thread.is_alive():
            return self._source
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run_loop, name="uwb-serial-bridge", daemon=True
        )
        self._thread.start()
        return self._source

    def stop(self, timeout: float = 2.0) -> None:
        """停止读线程并关闭串口。幂等。"""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                LOGGER.debug("serial close failed", exc_info=True)
            self._serial = None

    def run_for(self, duration_s: float) -> int:
        """同步运行指定时长(自测/脚本用),返回解析帧数。"""
        self.open()
        deadline = time.monotonic() + duration_s
        while not self._stop.is_set() and time.monotonic() < deadline:
            self._pump_once()
        return self._parser.stats.frames_parsed

    def _run_loop(self) -> None:
        LOGGER.info("UWB bridge loop started (source=%s)", self._source)
        while not self._stop.is_set():
            try:
                self._pump_once()
            except Exception:  # 任何异常不得退出长驻线程
                LOGGER.exception("UWB bridge loop iteration failed")
        LOGGER.info("UWB bridge loop stopped")

    def _pump_once(self) -> None:
        if self._source == "serial" and self._serial is not None:
            data = self._serial.read(self.config.read_chunk)
            if data:
                self.inject_bytes(bytes(data))
            return
        # mock 源:按 hz 生成一帧并注入
        if self._mock_source is None:
            self.open()
            return
        self.inject_bytes(self._mock_source.next_frame_bytes())
        self._stop.wait(1.0 / self._mock_source.hz)
