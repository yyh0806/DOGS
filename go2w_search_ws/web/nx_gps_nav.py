"""室外 GPS 航线导航 —— 纯逻辑核心 (零 rclpy 依赖)。

分层理由 (对齐 nx_navigation_arbiter.py / map_odom_fuser.py 范本):
  - 本文件只做"可脱离 ROS 测试"的决策与数学: NMEA 解析、GPS 质量门禁、
    大地坐标 → 局部 ENU → 地图系 变换、逐航点状态机。
  - ROS 侧 (订阅 NavSatFix / 串口 NMEA) 在 nx_gps_nav_ros.py 薄壳里,
    开发机无 ROS2 也能完整跑单测。

fail-closed 铁律 (每条停车路径都在此声明):
  1. GPS 定位质量降级 (无定位/卫星数不足/HDOP 超限/数据过期) → 取消当前
     Nav2 目标并 park 底盘, 绝不凭旧坐标盲走;
  2. 北向未标定 (map 系与真北夹角未知) → 拒绝受理航线 (heading_not_calibrated),
     否则整条航线被旋转一个未知角度, 属于盲走;
  3. 单航点超时 / Nav2 失败 (aborted/rejected/timed_out...) → 整条航线中止并 park;
  4. 航点所有权被抢 (generation 被其它生产者推进, 如面板点选) → 中止本航线并 park,
     绝不与其它 owner 抢动作客户端;
  5. 航点距离超上限 (max_waypoint_range_m) → 受理时拒绝, 防 lat/lon 手误把狗派到公里外。
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Optional, Sequence


logger = logging.getLogger("go2w.gps_nav")


__all__ = [
    "GpsFix",
    "GpsHealth",
    "GpsFixGate",
    "GpsWaypoint",
    "GpsRouteController",
    "nav_sat_fix_to_gps_fix",
    "parse_nmea_sentence",
    "enu_from_latlon",
    "latlon_from_enu",
    "map_goal_for_waypoint",
    "derive_heading_from_track",
]


# ---------------------------------------------------------------------------
# 基础数据结构
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class GpsFix:
    """一次 GPS 观测 (鸭子类型: 谁产生都行, 核心只认这份纯数据)。

    stamp 用调用方注入的单调时钟 (生产侧 = time.monotonic() 收包时刻),
    而不是设备时间戳: GPS 模块时钟不可信 (冷启动跳变), 新鲜度必须以
    "何时到达本机" 为准, 否则过期数据可能被误判为新鲜。
    """

    lat: float
    lon: float
    stamp: float
    altitude: Optional[float] = None
    # 0 = 有定位 (NavSatStatus.STATUS_FIX 及以上), -1 = 无定位。
    # NMEA GGA quality>0 / RMC 状态 A 均映射为 0。
    fix_status: int = 0
    satellites: Optional[int] = None
    hdop: Optional[float] = None


@dataclass(frozen=True)
class GpsHealth:
    """门禁裁决: ok=False 时 reason 说明停车/拒绝原因 (fail-closed)。"""

    ok: bool
    reason: Optional[str] = None
    age_sec: Optional[float] = None


@dataclass(frozen=True)
class GpsWaypoint:
    """航线中的一个 WGS-84 航点。yaw 为到达后朝向 (弧度, 可选)。"""

    lat: float
    lon: float
    yaw: float = 0.0
    name: Optional[str] = None


# ---------------------------------------------------------------------------
# ROS 消息鸭子类型转换 (纯函数, 无 rclpy import)
# ---------------------------------------------------------------------------

def nav_sat_fix_to_gps_fix(
    msg: Any, stamp_sec: float
) -> Optional[GpsFix]:
    """sensor_msgs/NavSatFix (或测试替身) → GpsFix; 非法数据返回 None。

    NavSatFix 标准里没有卫星数/HDOP 字段, 因此这两项置 None,
    由 GpsFixGate 决定是否要求 (None = 该维度不设限, 见门禁注释)。
    """
    try:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        altitude = getattr(msg, "altitude", None)
        alt = None if altitude is None else float(altitude)
        if not math.isfinite(lat) or not math.isfinite(lon):
            return None
        status_obj = getattr(msg, "status", None)
        fix_status = int(getattr(status_obj, "status", 0) or 0)
        stamp = float(stamp_sec)
        if not math.isfinite(stamp):
            return None
    except (TypeError, ValueError, AttributeError):
        return None
    return GpsFix(
        lat=lat, lon=lon, stamp=stamp, altitude=alt, fix_status=fix_status
    )


# ---------------------------------------------------------------------------
# NMEA 解析 (串口 GPS 直连路径, 纯函数)
# ---------------------------------------------------------------------------

def _nmea_checksum(body: str) -> Optional[int]:
    """'$' 与 '*' 之间逐字符异或; 解析失败返回 None。"""
    if len(body) < 1:
        return None
    value = 0
    for ch in body:
        value ^= ord(ch)
    return value


def _nmea_coord(value: str, hemisphere: str, *, lat: bool) -> Optional[float]:
    """ddmm.mmmm / dddmm.mmmm + 半球字母 → 十进制度; 非法返回 None。

    注意: NMEA 里纬度/经度的"分"都固定占两位整数 (ddmm / dddmm),
    所以度数提取统一除以 100 —— 不能按度位数除 100/1000 (实现初版就
    踩过这个坑: 经度 011°31.000' 被解析成 1°131', 分钟>=60 恒 False)。
    """
    try:
        raw = float(value)
    except (TypeError, ValueError):
        return None
    degrees = math.trunc(raw / 100.0)
    minutes = raw - degrees * 100.0
    if minutes < 0.0 or minutes >= 60.0:
        return None
    result = degrees + minutes / 60.0
    hemisphere = (hemisphere or "").strip().upper()
    if hemisphere not in (("N", "S") if lat else ("E", "W")):
        return None
    if hemisphere in ("S", "W"):
        result = -result
    return result


def parse_nmea_sentence(line: str) -> Optional[GpsFix]:
    """解析一条 NMEA 语句 (GGA/RMC) → GpsFix; 其余语句/坏校验/畸形 → None。

    为什么严格拒绝而不是"尽量解析": 半句可信的坐标比没有坐标更危险,
    fail-closed 文化里宁可丢一条数据也不能给门禁喂"看似合法"的脏值。
    """
    if not isinstance(line, str):
        return None
    text = line.strip()
    if not text.startswith("$") or "*" not in text:
        return None
    body, _, checksum_text = text[1:].partition("*")
    try:
        expected = int(checksum_text.strip()[:2], 16)
    except ValueError:
        return None
    actual = _nmea_checksum(body)
    if actual is None or actual != expected:
        return None
    fields = body.split(",")
    if len(fields) < 2:
        return None
    sentence = fields[0]
    talker_type = sentence[-3:] if len(sentence) >= 3 else ""
    stamp = time.monotonic()

    if talker_type == "GGA":
        # $xxGGA,time,lat,NS,lon,EW,quality,sats,hdop,alt,...
        if len(fields) < 10:
            return None
        lat = _nmea_coord(fields[2], fields[3], lat=True)
        lon = _nmea_coord(fields[4], fields[5], lat=False)
        if lat is None or lon is None:
            return None
        try:
            quality = int(fields[6])
            sats = int(fields[7])
            hdop = float(fields[8]) if fields[8] else None
            alt = float(fields[9]) if fields[9] else None
        except ValueError:
            return None
        return GpsFix(
            lat=lat, lon=lon, stamp=stamp, altitude=alt,
            fix_status=0 if quality > 0 else -1,
            satellites=sats, hdop=hdop,
        )

    if talker_type == "RMC":
        # $xxRMC,time,status,lat,NS,lon,EW,speed,course,...
        if len(fields) < 7:
            return None
        status = (fields[2] or "").strip().upper()
        lat = _nmea_coord(fields[3], fields[4], lat=True)
        lon = _nmea_coord(fields[5], fields[6], lat=False)
        if lat is None or lon is None:
            return None
        # RMC 没有 HDOP/卫星数字段: 置 None 交由门禁策略决定是否放行。
        return GpsFix(
            lat=lat, lon=lon, stamp=stamp,
            fix_status=0 if status == "A" else -1,
        )

    return None


# ---------------------------------------------------------------------------
# 大地坐标 → 局部 ENU (WGS-84 局部切平面)
# ---------------------------------------------------------------------------

_WGS84_A = 6378137.0                     # 长半轴 (米)
_WGS84_F = 1.0 / 298.257223563           # 扁率
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)  # 第一偏心率平方


def _meridional_radius(lat_deg: float) -> float:
    s = math.sin(math.radians(lat_deg)) ** 2
    return _WGS84_A * (1.0 - _WGS84_E2) / ((1.0 - _WGS84_E2 * s) ** 1.5)


def _prime_vertical_radius(lat_deg: float) -> float:
    s = math.sin(math.radians(lat_deg)) ** 2
    return _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * s)


def enu_from_latlon(
    lat0: float, lon0: float, lat: float, lon: float
) -> tuple[float, float]:
    """以 (lat0, lon0) 为原点的 (东向, 北向) 偏移, 单位米。

    局部切平面近似: ~10km 范围内误差 < 1m, 对轮足机器狗户外航线足够;
    引入完整椭球变换 (如 GeographicLib) 只增加依赖, 不改变精度需求量级。
    """
    rn = _prime_vertical_radius(lat0)
    rm = _meridional_radius(lat0)
    east = math.radians(lon - lon0) * rn * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * rm
    return east, north


def latlon_from_enu(
    lat0: float, lon0: float, east: float, north: float
) -> tuple[float, float]:
    """enu_from_latlon 的逆变换 (标定/回算用)。"""
    rn = _prime_vertical_radius(lat0)
    rm = _meridional_radius(lat0)
    lat = lat0 + math.degrees(north / rm)
    lon = lon0 + math.degrees(east / (rn * math.cos(math.radians(lat0))))
    return lat, lon


def map_goal_for_waypoint(
    anchor_fix: GpsFix,
    anchor_xy: tuple[float, float],
    north_heading_deg: float,
    waypoint: GpsWaypoint,
) -> tuple[float, float]:
    """航点 lat/lon → 地图系 (x, y)。

    推导: 设真北在地图系中的方向角为 H (从 map+x 逆时针, 度), 则
    东向单位向量在 map 系为 (sin H, -cos H), 北向为 (cos H, sin H):
        x = e·sinH + n·cosH ; y = -e·cosH + n·sinH
    """
    east, north = enu_from_latlon(
        anchor_fix.lat, anchor_fix.lon, waypoint.lat, waypoint.lon)
    h = math.radians(north_heading_deg)
    sin_h, cos_h = math.sin(h), math.cos(h)
    x = anchor_xy[0] + east * sin_h + north * cos_h
    y = anchor_xy[1] - east * cos_h + north * sin_h
    return x, y


def derive_heading_from_track(
    anchor_fix: GpsFix,
    anchor_xy: Sequence[float],
    track_fix: GpsFix,
    track_xy: Sequence[float],
) -> Optional[float]:
    """北向标定辅助: 用两对 (GPS, 地图位姿) 样本推 map 系真北方向角 (度)。

    做法: 同一段位移在 ENU 系与 map 系各有一个方位角 (均从各自 +x 轴
    逆时针量)。map_goal_for_waypoint 的旋转阵 R(H) = Rot(H - 90°),
    故 H = θ_map - θ_enu + 90°。直接取两方位角之差会得到 H-90°,
    是本函数初版踩过的坑 (正北样本被标成 0° 而非 90°)。
    返回 None 表示样本退化 (位移太短算不准 / 数学非法), 由调用方拒绝标定。
    """
    east, north = enu_from_latlon(
        anchor_fix.lat, anchor_fix.lon, track_fix.lat, track_fix.lon)
    enu_distance = math.hypot(east, north)
    if enu_distance < 1.0:
        return None  # 位移 <1m 时 GPS 噪声淹没信号, 标定必然失真
    map_dx = float(track_xy[0]) - float(anchor_xy[0])
    map_dy = float(track_xy[1]) - float(anchor_xy[1])
    if math.hypot(map_dx, map_dy) < 0.5:
        return None
    bearing_map = math.atan2(map_dy, map_dx)
    bearing_enu = math.atan2(north, east)
    heading = math.degrees(bearing_map - bearing_enu) + 90.0
    return heading % 360.0


# ---------------------------------------------------------------------------
# GPS 质量门禁 (fail-closed)
# ---------------------------------------------------------------------------

class GpsFixGate:
    """判定一次 GPS 观测是否可信; 任何维度不过 → ok=False。

    min_satellites / max_hdop 默认 None = 不检查: NavSatFix 标准消息不含
    这两个字段, 强制要求会令标准 ROS 驱动路径永远拒绝 (变成不可用)。
    串口 NMEA 路径 (GGA 自带 sats/hdop) 建议显式配置以收紧门槛。
    """

    def __init__(
        self,
        *,
        max_age_sec: float = 2.0,
        future_tolerance_sec: float = 0.25,
        min_satellites: Optional[int] = None,
        max_hdop: Optional[float] = None,
    ) -> None:
        self._max_age_sec = float(max_age_sec)
        self._future_tolerance_sec = float(future_tolerance_sec)
        self._min_satellites = min_satellites
        self._max_hdop = max_hdop

    def evaluate(self, fix: Optional[GpsFix], now: float) -> GpsHealth:
        if fix is None:
            return GpsHealth(ok=False, reason="no_fix")
        if not (
            math.isfinite(fix.lat) and math.isfinite(fix.lon)
            and -90.0 <= fix.lat <= 90.0 and -180.0 <= fix.lon <= 180.0
        ):
            return GpsHealth(ok=False, reason="invalid_coordinates")
        age = float(now) - fix.stamp
        if not math.isfinite(age) or age > self._max_age_sec:
            return GpsHealth(ok=False, reason="stale_fix", age_sec=age)
        if age < -self._future_tolerance_sec:
            # 允许极小的"未来"戳 (时钟相位抖动), 超出即视为非法时钟。
            return GpsHealth(ok=False, reason="clock_skew", age_sec=age)
        if fix.fix_status < 0:
            return GpsHealth(ok=False, reason="no_fix", age_sec=age)
        if (
            self._min_satellites is not None
            and (
                fix.satellites is None
                or int(fix.satellites) < int(self._min_satellites)
            )
        ):
            return GpsHealth(ok=False, reason="insufficient_satellites", age_sec=age)
        if (
            self._max_hdop is not None
            and (
                fix.hdop is None
                or not math.isfinite(float(fix.hdop))
                or float(fix.hdop) > float(self._max_hdop)
            )
        ):
            return GpsHealth(ok=False, reason="hdop_exceeded", age_sec=age)
        return GpsHealth(ok=True, age_sec=age)


# ---------------------------------------------------------------------------
# 航线控制器 (纯逻辑状态机)
# ---------------------------------------------------------------------------

# point_port 上视为"失败终态"的状态 (对齐 nx_navigation_gateway.TERMINAL_STATUSES)。
_PORT_FAILURE_STATUSES = frozenset({
    "aborted", "rejected", "timed_out", "cancel_failed",
    "server_unavailable", "error", "planner_failed",
})
_PORT_SUCCESS_STATUS = "succeeded"
_PORT_CANCEL_STATUSES = frozenset({"canceled"})


@dataclass
class _RouteRuntime:
    """一次受理后的航线运行时 (内部用)。"""

    waypoints: list[GpsWaypoint]
    index: int
    anchor_fix: GpsFix
    anchor_xy: tuple[float, float]
    started_at: float
    goal_generation: Optional[int] = None
    submitted_at: Optional[float] = None


class GpsRouteController:
    """逐航点驱动机器人沿 GPS 航线走, 全程 fail-closed。

    point_port 为鸭子类型注入 (生产 = PointNavigationController 或包了
    arbiter 的适配器), 只需: get_state()/submit(x,y,yaw)/cancel(reason)。
    park_hook 是"停车钩子": 航线中止/完成时回调, 生产侧接到仲裁器的
    park_drive —— 纯核心不自持运动链路, 只声明停车意图。
    """

    def __init__(
        self,
        point_port: Any,
        *,
        fix_gate: Optional[GpsFixGate] = None,
        park_hook: Optional[Callable[[str], None]] = None,
        state_callback: Optional[Callable[[dict[str, Any]], None]] = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        poll_interval: float = 0.05,
        arrive_timeout: float = 300.0,
        max_waypoint_range_m: float = 2000.0,
        north_heading_deg: float = 0.0,
        heading_calibrated: bool = False,
    ) -> None:
        self._point_port = point_port
        self._fix_gate = fix_gate or GpsFixGate()
        self._park_hook = park_hook
        self._state_callback = state_callback
        self._monotonic = monotonic
        self._sleep = sleep
        self._poll_interval = max(0.001, float(poll_interval))
        self._arrive_timeout = float(arrive_timeout)
        self._max_waypoint_range_m = float(max_waypoint_range_m)
        self._north_heading_deg = float(north_heading_deg)
        self._heading_calibrated = bool(heading_calibrated)
        self._lock = threading.RLock()
        self._latest_fix: Optional[GpsFix] = None
        self._route: Optional[_RouteRuntime] = None
        self._state: dict[str, Any] = {
            "active": False,
            "status": "idle",
            "waypoint_index": 0,
            "waypoint_total": 0,
            "goal": {"x": None, "y": None, "yaw": None},
            "reason": None,
            "gps_health": None,
        }

    # -- 对外 API ----------------------------------------------------------

    def update_fix(self, fix: Optional[GpsFix]) -> None:
        """供壳层喂数据: 每次 NavSatFix/NMEA 到达时调用。"""
        with self._lock:
            self._latest_fix = fix

    def set_heading_calibration(self, degrees: float) -> dict[str, Any]:
        """写入北向标定 (map 系与真北夹角, 度)。未标定则拒绝受理航线。"""
        value = float(degrees)
        if not math.isfinite(value):
            return {"ok": False, "reason": "invalid_heading"}
        with self._lock:
            self._north_heading_deg = value % 360.0
            self._heading_calibrated = True
        return {"ok": True, "heading_deg": self._north_heading_deg}

    def submit_route(
        self,
        waypoints: Iterable[Any],
        *,
        map_pose: dict[str, Any],
    ) -> dict[str, Any]:
        """受理一条航线; 任何前置条件不满足 → 拒绝 (fail-closed, 不动车)。"""
        normalized: list[GpsWaypoint] = []
        for raw in waypoints:
            converted = self._coerce_waypoint(raw)
            if converted is None:
                return {"ok": False, "reason": "invalid_waypoint"}
            normalized.append(converted)
        if not normalized:
            return {"ok": False, "reason": "empty_route"}

        with self._lock:
            if self._route is not None:
                # 已有航线在跑时拒绝新航线: 静默替换会让旧航线的停车钩子
                # 与新航线竞争, 属于未定义行为; 必须先显式 cancel。
                return {"ok": False, "reason": "route_active"}
            if not self._heading_calibrated:
                return {"ok": False, "reason": "heading_not_calibrated"}

            now = self._monotonic()
            health = self._fix_gate.evaluate(self._latest_fix, now)
            if not health.ok:
                return {"ok": False, "reason": f"gps_{health.reason}"}

            anchor_xy = self._coerce_map_pose(map_pose)
            if anchor_xy is None:
                return {"ok": False, "reason": "invalid_map_pose"}

            preflight = self._port_preflight()
            if preflight is not None:
                return {"ok": False, "reason": preflight}

            anchor_fix = self._latest_fix
            assert anchor_fix is not None  # 门禁通过即非空
            for waypoint in normalized:
                east, north = enu_from_latlon(
                    anchor_fix.lat, anchor_fix.lon, waypoint.lat, waypoint.lon)
                if math.hypot(east, north) > self._max_waypoint_range_m:
                    return {"ok": False, "reason": "waypoint_out_of_range"}

            self._route = _RouteRuntime(
                waypoints=normalized,
                index=0,
                anchor_fix=anchor_fix,
                anchor_xy=anchor_xy,
                started_at=now,
            )
            self._transition_locked(
                "navigating", message="route accepted", reason=None)

        # 首个航点在锁外提交 (submit 可能阻塞等 executor);
        # 失败则回滚整条航线并停车。
        return self._submit_current_waypoint()

    def cancel(self, reason: str = "operator_cancel") -> dict[str, Any]:
        """操作员取消: 只撤 Nav2 目标, 不 park (操作员可能要接管手动)。"""
        normalized = str(reason or "operator_cancel")
        with self._lock:
            route = self._route
            if route is None:
                return {"ok": True, "status": "idle", "ignored": True}
            self._route = None
            self._transition_locked(
                "canceled", message="route canceled", reason=normalized)
        try:
            self._point_port.cancel(normalized)
        except Exception:
            logger.exception("point_port cancel failed during route cancel")
        return {"ok": True, "status": "canceled", "reason": normalized}

    def tick(self) -> dict[str, Any]:
        """非阻塞心跳: 由壳层定时调用, 驱动航点推进与停车判定。"""
        with self._lock:
            route = self._route
            if route is None:
                return dict(self._state)
            now = self._monotonic()

            health = self._fix_gate.evaluate(self._latest_fix, now)
            self._state["gps_health"] = {
                "ok": health.ok, "reason": health.reason,
            }
            if not health.ok:
                # 停车条件 1: GPS 降级。立刻撤目标 + park, 绝不盲走。
                self._abort_locked(f"gps_{health.reason}", cancel=True)
                return dict(self._state)

            try:
                port_state = dict(self._point_port.get_state())
            except Exception:
                self._abort_locked("point_port_unavailable", cancel=True)
                return dict(self._state)

            preflight = self._preflight_from_state(port_state)
            if preflight is not None:
                self._abort_locked(preflight, cancel=True)
                return dict(self._state)

            generation = port_state.get("generation")
            status = str(port_state.get("status") or "")
            route_gen = route.goal_generation

            if route_gen is not None and isinstance(generation, int):
                if generation > route_gen:
                    # 停车条件 4: 我们的航点被替换 (面板点选/其它生产者)。
                    # 不 cancel (目标已不归我们)、也不 park —— 运动所有权已
                    # 移交给新 owner, 此时停车会与其抢底盘; 新 owner 的停车
                    # 义务由 arbiter 在它自己的 drain 路径上履行。
                    self._abort_locked(
                        "waypoint_ownership_lost", cancel=False, park=False)
                    return dict(self._state)

            if route_gen is not None and generation == route_gen:
                if status == _PORT_SUCCESS_STATUS:
                    if self._advance_locked():
                        return dict(self._state)
                    # advance 返回 False = 还有下一个航点: 解锁后再提交,
                    # submit 可能阻塞等 executor, 不能在持锁期间调用。
                    submit_needed = True
                elif status in _PORT_FAILURE_STATUSES:
                    # 停车条件 3: Nav2 失败终态 → 整线中止 (后续航点也不走)。
                    self._abort_locked(f"waypoint_{status}", cancel=False)
                    return dict(self._state)
                else:
                    submit_needed = False
            else:
                submit_needed = False

            if (
                route.submitted_at is not None
                and not submit_needed
                and now - route.submitted_at > self._arrive_timeout
            ):
                # 停车条件 3: 单航点超时 → 撤目标 + park。
                self._abort_locked("waypoint_timeout", cancel=True)
                return dict(self._state)

        if submit_needed:
            self._submit_current_waypoint()
        with self._lock:
            return dict(self._state)

    def wait_until_settled(self, timeout: float) -> dict[str, Any]:
        """阻塞等待航线到达终态 (completed/aborted/canceled); 供 bringup 脚本用。"""
        deadline = self._monotonic() + max(0.0, float(timeout))
        while True:
            state = self.tick()
            if not state.get("active"):
                return state
            remaining = deadline - self._monotonic()
            if remaining <= 0.0:
                return state
            self._sleep(min(self._poll_interval, remaining))

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._state)

    # -- 内部 --------------------------------------------------------------

    @staticmethod
    def _coerce_waypoint(raw: Any) -> Optional[GpsWaypoint]:
        if isinstance(raw, GpsWaypoint):
            waypoint = raw
        elif isinstance(raw, dict):
            try:
                waypoint = GpsWaypoint(
                    lat=float(raw["lat"]),
                    lon=float(raw["lon"]),
                    yaw=float(raw.get("yaw") or 0.0),
                    name=raw.get("name"),
                )
            except (KeyError, TypeError, ValueError):
                return None
        else:
            return None
        if not (
            math.isfinite(waypoint.lat) and math.isfinite(waypoint.lon)
            and math.isfinite(waypoint.yaw)
            and -90.0 <= waypoint.lat <= 90.0
            and -180.0 <= waypoint.lon <= 180.0
        ):
            return None
        return waypoint

    @staticmethod
    def _coerce_map_pose(pose: dict[str, Any]) -> Optional[tuple[float, float]]:
        try:
            x = float(pose["x"])
            y = float(pose["y"])
        except (KeyError, TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return (x, y)

    def _port_preflight(self) -> Optional[str]:
        try:
            state = dict(self._point_port.get_state())
        except Exception:
            return "point_port_unavailable"
        return self._preflight_from_state(state)

    @staticmethod
    def _preflight_from_state(state: dict[str, Any]) -> Optional[str]:
        if state.get("stopped"):
            return "point_nav_stopped"
        if state.get("quarantined"):
            return "point_nav_quarantined"
        if state.get("healthy") is False:
            return "point_nav_unhealthy"
        return None

    def _submit_current_waypoint(self) -> dict[str, Any]:
        with self._lock:
            route = self._route
            if route is None:
                return {"ok": False, "reason": "route_inactive"}
            waypoint = route.waypoints[route.index]
            goal_x, goal_y = map_goal_for_waypoint(
                route.anchor_fix, route.anchor_xy,
                self._north_heading_deg, waypoint)
            self._state["goal"] = {
                "x": goal_x, "y": goal_y, "yaw": waypoint.yaw}

        try:
            response = dict(self._point_port.submit(
                goal_x, goal_y, waypoint.yaw) or {})
        except Exception as exc:
            with self._lock:
                if self._route is route:
                    self._abort_locked("waypoint_submit_error", cancel=True)
            logger.exception("waypoint submit failed: %s", exc)
            return {"ok": False, "reason": "waypoint_submit_error"}

        if not response.get("ok", False):
            # 停车原因透传端口拒绝理由 (planner_failed 等), 便于面板诊断。
            abort_reason = str(
                response.get("reason") or "waypoint_submit_rejected")
            with self._lock:
                if self._route is route:
                    self._abort_locked(abort_reason, cancel=True)
            return {"ok": False, "reason": abort_reason}

        generation = response.get("generation")
        with self._lock:
            if self._route is not route:
                # 提交期间被并发取消: 目标已提交, 需要撤掉以保持 fail-closed。
                try:
                    self._point_port.cancel("route_cancelled_during_submit")
                except Exception:
                    pass
                return {"ok": False, "reason": "route_cancelled"}
            route.goal_generation = (
                int(generation) if isinstance(generation, int) else None)
            route.submitted_at = self._monotonic()
            self._transition_locked(
                "navigating", message="waypoint submitted", reason=None)
        return {"ok": True, "index": route.index}

    def _advance_locked(self) -> bool:
        """推进到下一航点; 返回 True 表示航线已收尾, False 表示还有下一站。"""
        route = self._route
        if route is None:
            return True
        route.index += 1
        if route.index >= len(route.waypoints):
            # 全部到达: 航线正常收尾也要 park (Nav2 succeeded ≠ 底盘静止,
            # wheel_balance 下零速仍可能滑移, 对齐 arbiter.on_point_state)。
            self._finish_locked("gps_route_completed")
            return True
        route.goal_generation = None
        route.submitted_at = None
        self._transition_locked(
            "navigating", message="waypoint reached", reason=None)
        return False

    def _abort_locked(
        self, reason: str, *, cancel: bool, park: bool = True
    ) -> None:
        self._route = None
        self._transition_locked("aborted", message="route aborted", reason=reason)
        if cancel:
            try:
                self._point_port.cancel(reason)
            except Exception:
                logger.exception(
                    "point_port cancel failed while aborting route")
        # park 钩子是停车意图的声明: 即使异常也必须吞掉 (回调方故障不能
        # 阻塞 fail-closed 主路径), 生产侧由 arbiter 兜底 estop。
        if park and self._park_hook is not None:
            try:
                self._park_hook(reason)
            except Exception:
                logger.exception("park hook failed: %s", reason)

    def _finish_locked(self, reason: str) -> None:
        self._route = None
        self._transition_locked(
            "completed", message="route completed", reason=reason)
        if self._park_hook is not None:
            try:
                self._park_hook(reason)
            except Exception:
                logger.exception("park hook failed: %s", reason)

    def _transition_locked(
        self, status: str, *, message: str, reason: Optional[str]
    ) -> None:
        route = self._route
        self._state = {
            "active": route is not None,
            "status": status,
            "waypoint_index": 0 if route is None else route.index,
            "waypoint_total": 0 if route is None else len(route.waypoints),
            "goal": dict(self._state["goal"]),
            "reason": reason,
            "gps_health": self._state.get("gps_health"),
            "message": message,
            "updated_monotonic": self._monotonic(),
        }
        if self._state_callback is not None:
            try:
                self._state_callback(dict(self._state))
            except Exception:
                # 坏的 WS 消费者不允许破坏安全主路径 (对齐 point_nav 约定)。
                pass
