#!/usr/bin/env python3
"""UWB 钥匙扣跟随控制器 (功能 B-2) —— ROS-free 纯逻辑状态机。

设计参照 nx_navigation_arbiter.py: 本模块刻意不 import ROS。所有物理世界
交互(UWB 测距、雷达间隙探测、速度发布、运动所有权)通过构造注入的回调完成,
使其在无 ROS2 运行时的开发机/CI 上可完整单测 (worktree 环境无 ROS)。

== 依赖注入合同 ==

uwb_source (拉模式, 可选):
    提供 get_fix() -> dict|None。UWB 桥 (功能 B-1, nx_uwb_bridge) 或测试替身
    实现该接口。fix 字段合同 (供 B-1 对齐):
        {
          "ok": True,             # 缺省 True; False 表示本拍无有效测距
          "range_m": 3.42,        # 必填: 钥匙扣↔机器人距离 (米), 有限数
          "azimuth_rad": -0.21,   # 可选: 钥匙扣方位角, 机器人系, +左 (REP-103);
                                  #       一维测距模块可缺省/None (只跟不转)
          "age_sec": 0.12,        # 可选: 源侧测距年龄 (秒); 缺省按"本拍新鲜"信任
          "anchor_id": "keyfob-01",  # 可选: 锚点/钥匙扣标识
          "quality": 92,          # 可选: 0-100
        }
    推模式替代: 任何线程可调 controller.ingest_fix(fix) 喂入 (桥线程直推),
    控制器每 tick 取"最新鲜"的一帧 (源拉取 vs 推送取 age 较小者)。

clearance_probe (必填, 避障闸门):
    probe(direction_deg, half_fov_deg) -> float|None
    返回该方向的最近障碍距离 (米); None/异常 = 未知。NxRobotBridge.directional_
    clearance 签名一致, 直接注入。闸门是**第一优先级**: 每 tick 先评闸门再算
    跟随; 探测失败按"被挡"处理 (fail-closed), 宁停不撞。

motion_sink (必填):
    sink(vx, vy, wz) -> None   发布底盘速度 (web 集成用 robot.move(manual=True),
    直发 /cmd_vel 通道; 闸门已在控制器内先行)。

ownership_acquire / ownership_release (可选):
    acquire(reason) -> {"ok": bool, "reason": str}
    release(reason) -> {"ok": bool, "reason": str}
    web 集成分别接 NavigationArbiter.run_manual_action / release_manual:
    跟随是新的自主运动生产者, 经 manual 所有权通道串行化 (零速 handoff 语义
    由 arbiter 保证), B-3 集成若引入独立 owner 只需换这两个回调, 控制器不动。

== 状态机 ==
    idle ──start()──► searching ──有效测距+闸门开──► following
    searching ◄──测距失效── following / obstacle_hold
    following ⇄ obstacle_hold        (闸门挡/恢复, 迟滞防抖)
    searching ─超时──► idle (target_not_acquired / target_lost)
    任意 ──stop()──► idle
    任意活跃态 ─运动发布连续失败/所有权重试耗尽──► fault (锁存, 需 stop 复位)

== 安全要点 ==
1. 避障闸门第一优先: 闸门挡住 → 线速度强制 0 (原地转向 wz 保留, 不平移不会
   更接近障碍); 迟滞 (stop 距离挡, stop+margin 才放) 防止边界抖动。
2. 只前进不后退: 距离小于死区 → 站住 (vx=0), 绝不倒车逼近后方未知区。
3. 测距野值剔除: 单帧跳变超阈值先丢弃, 连续 outlier_snap_count 帧才接受
   (目标真的快速移动), 平滑距离进控制律。
4. 所有权懒获取: 有非零指令才 acquire; 连续零速超过 idle 秒数自动 release
   (狗回 joint_lock, 省电); acquire 失败重试, 超限 → fault。
5. 每个活跃→非活跃转换都尽力补发一次零速, 再释放所有权 (物理先停再放)。
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, Optional, Tuple


logger = logging.getLogger("go2w.uwb_follow")

# 面向 web/前端的状态名 (稳定字符串, 契约的一部分)
STATE_IDLE = "idle"
STATE_SEARCHING = "searching"
STATE_FOLLOWING = "following"
STATE_OBSTACLE_HOLD = "obstacle_hold"
STATE_FAULT = "fault"
_ACTIVE_STATES = frozenset({STATE_SEARCHING, STATE_FOLLOWING,
                            STATE_OBSTACLE_HOLD})

# start(payload) 允许的参数键 + 取值范围 (闭区间; None 表示无该侧界).
# 未知键直接 ValueError (拼错键名立刻暴露, 不静默吞)。
FOLLOW_PARAM_BOUNDS: Dict[str, Tuple[Optional[float], Optional[float]]] = {
    "follow_distance_m": (0.3, 5.0),
    "approach_band_m": (0.05, 1.0),
    "range_gain": (0.05, 3.0),
    "max_vx": (0.05, 0.8),
    "max_wz": (0.1, 1.0),
    "yaw_gain": (0.1, 5.0),
    "yaw_deadzone_rad": (0.0, 0.5),
    "gate_stop_distance_m": (0.1, 2.0),
    "gate_resume_margin_m": (0.02, 0.5),
    "gate_probe_fov_deg": (5.0, 90.0),
    "clearance_taper_gain": (0.1, 5.0),
    "fix_max_age_sec": (0.1, 5.0),
    "fix_min_range_m": (0.05, 1.0),
    "fix_max_range_m": (5.0, 100.0),
    "acquire_timeout_sec": (1.0, 60.0),
    "lost_timeout_sec": (1.0, 60.0),
    "jump_threshold_m": (0.2, 10.0),
    "ownership_idle_release_sec": (0.5, 60.0),
}


def _finite(value: Any) -> Optional[float]:
    """Coerce to a finite float, else None (Never raise on dirty input)."""
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


@dataclass(frozen=True)
class FollowParams:
    """跟随控制参数 (frozen: 运行期调参走 update_params 整体替换)。"""

    follow_distance_m: float = 1.2       # 期望人狗间距
    approach_band_m: float = 0.2         # 距离死区 (±), 区间内站住不跟
    range_gain: float = 0.6              # vx = gain * (距离误差 - 死区)
    max_vx: float = 0.4                  # 与面板手动上限一致
    max_wz: float = 0.5                  # 与面板手动上限一致
    yaw_gain: float = 1.2                # wz = gain * 方位角
    yaw_deadzone_rad: float = 0.08       # 方位死区, 防小幅摆头
    gate_stop_distance_m: float = 0.5    # 前向净空低于此 → 闸门挡
    gate_resume_margin_m: float = 0.15   # 迟滞: 净空需回到 stop+margin 才放行
    gate_probe_fov_deg: float = 35.0     # 前向探测半张角
    clearance_taper_gain: float = 0.8    # 近障碍降速: vx 上限 = gain*(净空-stop)
    fix_max_age_sec: float = 0.7         # 测距帧最大年龄
    fix_min_range_m: float = 0.1         # 短于此视为无效 (UWB 近场盲区)
    fix_max_range_m: float = 30.0        # 长于此视为无效 (超跟随范围)
    acquire_timeout_sec: float = 8.0     # start 后首帧测距等待上限
    lost_timeout_sec: float = 5.0        # 已跟住后测距丢失容忍上限
    jump_threshold_m: float = 1.5        # 单帧跳变阈值 (野值剔除)
    ownership_idle_release_sec: float = 3.0  # 连续零速多久后释放运动所有权

    def validate(self) -> "FollowParams":
        for name, (low, high) in FOLLOW_PARAM_BOUNDS.items():
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ValueError(f"{name} 必须是数字")
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} 必须是有限数")
            if low is not None and value < low:
                raise ValueError(f"{name} 不能小于 {low}")
            if high is not None and value > high:
                raise ValueError(f"{name} 不能大于 {high}")
        if self.gate_resume_margin_m > self.gate_stop_distance_m:
            raise ValueError("gate_resume_margin_m 不能大于 gate_stop_distance_m")
        if self.fix_min_range_m >= self.follow_distance_m:
            raise ValueError("fix_min_range_m 必须小于 follow_distance_m")
        if self.fix_min_range_m >= self.fix_max_range_m:
            raise ValueError("fix_min_range_m 必须小于 fix_max_range_m")
        return self

    @classmethod
    def from_dict(cls, payload: Optional[dict], base: Optional["FollowParams"] = None) -> "FollowParams":
        """Merge payload onto base (缺省当前默认/现值), 未知键报错。"""
        params = base or cls()
        if payload is None:
            return params
        if not isinstance(payload, dict):
            raise ValueError("参数必须是 JSON 对象")
        unknown = set(payload) - set(FOLLOW_PARAM_BOUNDS)
        if unknown:
            raise ValueError(f"未知参数: {sorted(unknown)}")
        updates = {}
        for key, raw in payload.items():
            if isinstance(raw, bool):
                raise ValueError(f"{key} 必须是数字")
            try:
                value = float(raw)
            except (TypeError, ValueError):
                raise ValueError(f"{key} 必须是数字") from None
            if not math.isfinite(value):
                raise ValueError(f"{key} 必须是有限数")
            updates[key] = value
        return replace(params, **updates).validate()


def evaluate_obstacle_gate(
    clearance_m: Optional[float],
    currently_blocked: bool,
    params: FollowParams,
) -> Tuple[bool, str]:
    """第一优先避障闸门 (纯函数)。

    返回 (blocked, reason)。fail-closed: 探测缺失/非有限 → 挡。
    迟滞: 挡住后需净空 ≥ stop+margin 才放行, 防止在阈值附近抖动启停。
    """
    if clearance_m is None or not math.isfinite(clearance_m):
        return True, "clearance_unavailable"
    if currently_blocked:
        resume_at = params.gate_stop_distance_m + params.gate_resume_margin_m
        if clearance_m < resume_at:
            return True, "clearance_below_resume"
        return False, "clearance_resumed"
    if clearance_m < params.gate_stop_distance_m:
        return True, "clearance_below_stop"
    return False, "clearance_open"


def compute_follow_velocity(
    range_m: float,
    azimuth_rad: Optional[float],
    clearance_m: Optional[float],
    params: FollowParams,
) -> Tuple[float, float, float]:
    """跟随控制律 (纯函数): 距离→前进速度, 方位→转向速度。

    - 只前进: 误差 ≤ 死区 → vx=0 (绝不倒车)。
    - 大角度减速: vx *= max(0, cos(方位角)), 斜向接近时先转正再走。
    - 近障碍降速: vx 上限再压到 taper*(净空-挡距), 平滑贴闸门。
    - 返回的 vx 尚未经过闸门硬挡 (调用方先评闸门, 见 tick)。
    """
    error = range_m - params.follow_distance_m
    if error <= params.approach_band_m:
        vx = 0.0
    else:
        vx = params.range_gain * (error - params.approach_band_m)
        vx = min(vx, params.max_vx)
        if clearance_m is not None and math.isfinite(clearance_m):
            taper_cap = params.clearance_taper_gain * max(
                0.0, clearance_m - params.gate_stop_distance_m)
            vx = min(vx, taper_cap)
    wz = 0.0
    if azimuth_rad is not None and math.isfinite(azimuth_rad):
        if abs(azimuth_rad) > params.yaw_deadzone_rad:
            wz = max(-params.max_wz,
                     min(params.max_wz, params.yaw_gain * azimuth_rad))
        vx *= max(0.0, math.cos(azimuth_rad))
    return vx, 0.0, wz


class UwbFollowController:
    """UWB 钥匙扣跟随状态机 (线程安全, tick 驱动)。

    tick() 由宿主 (web 侧 daemon 线程 / 测试) 周期调用 (~10Hz); 本类不自建
    线程, 时钟经 monotonic 注入, 全部行为确定性可测。
    """

    def __init__(
        self,
        *,
        clearance_probe: Callable[[float, float], Optional[float]],
        motion_sink: Callable[[float, float, float], None],
        uwb_source: Any = None,
        ownership_acquire: Optional[Callable[[str], dict]] = None,
        ownership_release: Optional[Callable[[str], dict]] = None,
        state_callback: Optional[Callable[[dict], None]] = None,
        monotonic: Callable[[], float] = time.monotonic,
        params: Optional[FollowParams] = None,
        outlier_snap_count: int = 2,
        fault_limit: int = 3,
        ownership_retry_limit: int = 5,
        state_heartbeat_sec: float = 1.0,
    ) -> None:
        if not callable(clearance_probe):
            raise ValueError("clearance_probe 必须可调用 (避障闸门是硬前置)")
        if not callable(motion_sink):
            raise ValueError("motion_sink 必须可调用")
        if outlier_snap_count < 1:
            raise ValueError("outlier_snap_count 必须 ≥ 1")
        if fault_limit < 1:
            raise ValueError("fault_limit 必须 ≥ 1")
        if ownership_retry_limit < 1:
            raise ValueError("ownership_retry_limit 必须 ≥ 1")
        self._probe = clearance_probe
        self._sink = motion_sink
        self._source = uwb_source
        self._acquire_ownership = ownership_acquire
        self._release_ownership = ownership_release
        self._state_callback = state_callback
        self._monotonic = monotonic
        self._params = (params or FollowParams()).validate()
        self._outlier_snap_count = int(outlier_snap_count)
        self._fault_limit = int(fault_limit)
        self._ownership_retry_limit = int(ownership_retry_limit)
        self._state_heartbeat_sec = float(state_heartbeat_sec)

        self._lock = threading.RLock()
        self._state = STATE_IDLE
        self._last_reason: Optional[str] = None
        self._started_at: Optional[float] = None
        self._last_valid_fix_at: Optional[float] = None
        self._ever_had_fix = False
        # 推模式最新帧 (ingest_fix)
        self._pushed_fix: Optional[dict] = None
        self._pushed_fix_at: Optional[float] = None
        # 距离滤波器
        self._filtered_range: Optional[float] = None
        self._outlier_streak = 0
        # 闸门
        self._gate_blocked = False
        self._gate_reason: Optional[str] = "clearance_unavailable"
        self._last_clearance: Optional[float] = None
        # 所有权/发布
        self._owns_motion = False
        self._ownership_failures = 0
        self._zero_since: Optional[float] = None
        self._sink_failures = 0
        # 快照缓存 (get_state 无副作用)
        self._last_command = (0.0, 0.0, 0.0)
        self._last_fix: Optional[dict] = None
        self._last_emit_at: Optional[float] = None
        self._last_emitted_state: Optional[str] = None
        self._ticks = 0

    # ------------------------------------------------------------------
    # 生命周期 API
    # ------------------------------------------------------------------

    def start(self, payload: Optional[dict] = None) -> dict:
        """启动跟随 (idle→searching); 已活跃时幂等返回。"""
        try:
            new_params = FollowParams.from_dict(payload, base=self._params)
        except ValueError as exc:
            return {"ok": False, "reason": "invalid_params", "message": str(exc)}
        with self._lock:
            if self._state == STATE_FAULT:
                return {"ok": False, "reason": "fault_latched",
                        "message": "故障锁存, 先 stop() 复位再启动"}
            if self._state in _ACTIVE_STATES:
                return {"ok": True, "already_active": True,
                        "state": self._state}
            if self._source is None and self._pushed_fix is None:
                # 闸门/发布器构造期已校验; 这里只拦 UWB 源缺失。
                # 允许"先 start 后接桥"吗? 不允许: 无源跟随必空转, 拒绝更诚实。
                return {"ok": False, "reason": "uwb_source_unavailable",
                        "message": "UWB 测距源未接入 (nx_uwb_bridge 未加载)"}
            self._params = new_params
            self._reset_tracking_locked()
            self._state = STATE_SEARCHING
            self._started_at = self._monotonic()
            self._last_reason = None
            self._last_valid_fix_at = None
            self._ever_had_fix = False
            self._last_emit_at = None
            self._last_emitted_state = None
            logger.info("uwb follow started: params=%s", self._params)
            self._emit_locked(True)
            return {"ok": True, "state": self._state}

    def stop(self, reason: str = "operator_stop") -> dict:
        """停止跟随 (任意态→idle): 先零速后放所有权。"""
        with self._lock:
            was_active = self._state in _ACTIVE_STATES or self._state == STATE_FAULT
            self._deactivate_locked(reason, publish_zero=self._owns_motion)
            self._state = STATE_IDLE
            self._last_reason = reason
            if was_active:
                logger.info("uwb follow stopped: reason=%s", reason)
            self._emit_locked(True)
            return {"ok": True, "state": self._state, "reason": reason}

    def update_params(self, payload: Optional[dict]) -> dict:
        """运行期调参 (跟随中也可调, 下一 tick 生效)。"""
        try:
            merged = FollowParams.from_dict(payload, base=self._params)
        except ValueError as exc:
            return {"ok": False, "reason": "invalid_params", "message": str(exc)}
        with self._lock:
            self._params = merged
            return {"ok": True, "params": self._params_summary(merged)}

    def ingest_fix(self, fix: Optional[dict]) -> None:
        """推模式喂入一帧 UWB 测距 (桥线程直调; 坏帧静默丢弃)。"""
        with self._lock:
            self._pushed_fix = dict(fix) if isinstance(fix, dict) else None
            self._pushed_fix_at = self._monotonic()

    def get_state(self) -> dict:
        """无副作用状态快照 (web /api/status 与 WS 广播共用)。"""
        with self._lock:
            return self._snapshot_locked()

    # ------------------------------------------------------------------
    # tick: 状态机主循环 (每拍顺序 = 闸门 → 测距 → 控制律 → 所有权 → 发布)
    # ------------------------------------------------------------------

    def tick(self) -> dict:
        now = self._monotonic()
        with self._lock:
            self._ticks += 1
            if self._state not in _ACTIVE_STATES:
                return self._snapshot_locked()

            # ---- 1) 避障闸门 (第一优先, 先于一切跟随逻辑; fail-closed) ----
            try:
                clearance = self._probe(0.0, self._params.gate_probe_fov_deg)
            except Exception:
                clearance = None
            self._last_clearance = _finite(clearance)
            blocked, gate_reason = evaluate_obstacle_gate(
                self._last_clearance, self._gate_blocked, self._params)
            gate_changed = blocked != self._gate_blocked
            self._gate_blocked = blocked
            self._gate_reason = gate_reason

            # ---- 2) UWB 测距 (取最新鲜有效帧) ----
            fix, fix_error = self._read_fix(now)
            if fix is not None:
                self._last_fix = fix
                self._last_valid_fix_at = now
                self._ever_had_fix = True
                self._apply_range_filter(fix["range_m"])

            # ---- 3) 超时自动停 (首帧等待 / 丢失容忍) ----
            if fix is None:
                deadline_kind = None
                if not self._ever_had_fix:
                    if (self._started_at is not None
                            and now - self._started_at >= self._params.acquire_timeout_sec):
                        deadline_kind = "target_not_acquired"
                else:
                    if (self._last_valid_fix_at is not None
                            and now - self._last_valid_fix_at >= self._params.lost_timeout_sec):
                        deadline_kind = "target_lost"
                if deadline_kind is not None:
                    self._deactivate_locked(deadline_kind,
                                            publish_zero=self._owns_motion)
                    self._state = STATE_IDLE
                    self._last_reason = deadline_kind
                    logger.warning("uwb follow auto-stop: reason=%s", deadline_kind)
                    self._emit_locked(True)
                    return self._snapshot_locked()

            # ---- 4) 状态转移 + 控制律 ----
            if fix is None:
                target_state = STATE_SEARCHING
                command = (0.0, 0.0, 0.0)
            else:
                range_m = self._filtered_range if self._filtered_range is not None else fix["range_m"]
                vx, vy, wz = compute_follow_velocity(
                    range_m, fix.get("azimuth_rad"),
                    self._last_clearance, self._params)
                if blocked:
                    # 闸门第一优先: 平移清零, 仅保留原地转向 (不平移不逼近障碍)
                    target_state = STATE_OBSTACLE_HOLD
                    command = (0.0, 0.0, wz)
                else:
                    target_state = STATE_FOLLOWING
                    command = (vx, vy, wz)

            state_changed = target_state != self._state
            self._state = target_state

            # ---- 5) 运动所有权 (懒获取 / 零速闲置释放) ----
            nonzero = any(abs(v) > 1e-9 for v in command)
            ownership_ok = self._manage_ownership_locked(now, nonzero)
            if not ownership_ok:
                command = (0.0, 0.0, 0.0)

            # ---- 6) 发布速度 (故障计数 → fault 锁存) ----
            self._publish_locked(command)

            if gate_changed or state_changed:
                logger.info(
                    "uwb follow: state=%s gate=%s(%s) range=%s cmd=%s",
                    self._state, blocked, gate_reason,
                    None if fix is None else round(fix["range_m"], 2),
                    tuple(round(v, 3) for v in command))
            self._emit_locked(gate_changed or state_changed)
            return self._snapshot_locked()

    # ------------------------------------------------------------------
    # 内部 (全部在锁内)
    # ------------------------------------------------------------------

    def _read_fix(self, now: float) -> Tuple[Optional[dict], Optional[str]]:
        """合并拉/推两路, 取最新鲜的有效帧; 返回 (fix|None, error)。"""
        candidates: list = []
        source_error = "no_source"
        if self._source is not None:
            try:
                raw = self._source.get_fix()
            except Exception:
                raw = None
            parsed, source_error = self._validate_fix(
                raw, now, source_side=True)
            if parsed is not None:
                candidates.append(parsed)
        if self._pushed_fix is not None and self._pushed_fix_at is not None:
            parsed, _err = self._validate_fix(self._pushed_fix, now,
                                              source_side=False)
            if parsed is not None:
                candidates.append(parsed)
        if not candidates:
            return None, source_error if self._source is not None else "no_fix"
        # age 最小者最新鲜
        candidates.sort(key=lambda f: f["age_sec"])
        return candidates[0], None
        if self._pushed_fix is not None and self._pushed_fix_at is not None:
            parsed, err = self._validate_fix(self._pushed_fix, now,
                                             source_side=False)
            if parsed is not None:
                candidates.append(parsed)
        if not candidates:
            return None, source_error if self._source is not None else "no_fix"
        # age 最小者最新鲜
        candidates.sort(key=lambda f: f["age_sec"])
        return candidates[0], None

    def _validate_fix(self, raw: Any, now: float, *, source_side: bool) -> Tuple[Optional[dict], Optional[str]]:
        if not isinstance(raw, dict):
            return None, "fix_not_dict"
        if raw.get("ok") is False:
            return None, str(raw.get("reason") or "fix_not_ok")
        range_m = _finite(raw.get("range_m"))
        if range_m is None:
            return None, "range_invalid"
        if range_m < self._params.fix_min_range_m:
            return None, "range_too_short"
        if range_m > self._params.fix_max_range_m:
            return None, "range_too_far"
        azimuth = _finite(raw.get("azimuth_rad"))
        if raw.get("azimuth_rad") is not None and azimuth is None:
            return None, "azimuth_invalid"
        if source_side:
            age = _finite(raw.get("age_sec"))
            if age is None:
                age = 0.0  # 源未带年龄: 按本拍新鲜信任 (丢失看门狗仍兜底)
        else:
            age = now - float(self._pushed_fix_at or now)
        if age > self._params.fix_max_age_sec:
            return None, "fix_stale"
        return {
            "range_m": range_m,
            "azimuth_rad": azimuth,
            "age_sec": age,
            "anchor_id": raw.get("anchor_id"),
            "quality": raw.get("quality"),
        }, None

    def _apply_range_filter(self, range_m: float) -> None:
        """野值剔除: 跳变先丢, 连续 snap 帧才接受 (目标真的动了)。"""
        if self._filtered_range is None:
            self._filtered_range = range_m
            self._outlier_streak = 0
            return
        if abs(range_m - self._filtered_range) <= self._params.jump_threshold_m:
            self._filtered_range = range_m
            self._outlier_streak = 0
        else:
            self._outlier_streak += 1
            if self._outlier_streak >= self._outlier_snap_count:
                self._filtered_range = range_m
                self._outlier_streak = 0

    def _manage_ownership_locked(self, now: float, nonzero: bool) -> bool:
        """懒获取/闲置释放运动所有权; 返回本拍是否允许发布非零指令。"""
        if nonzero:
            self._zero_since = None
            if not self._owns_motion:
                if self._acquire_ownership is None:
                    self._owns_motion = True  # 无宿主仲裁 (测试/独立部署)
                    self._ownership_failures = 0
                    return True
                try:
                    result = dict(self._acquire_ownership("uwb_follow") or {})
                except Exception as exc:
                    result = {"ok": False, "reason": f"acquire_error:{exc}"}
                if result.get("ok"):
                    self._owns_motion = True
                    self._ownership_failures = 0
                    return True
                self._ownership_failures += 1
                logger.warning(
                    "uwb follow ownership acquire failed (%d/%d): %s",
                    self._ownership_failures, self._ownership_retry_limit,
                    result.get("reason"))
                if self._ownership_failures >= self._ownership_retry_limit:
                    self._enter_fault_locked("ownership_exhausted")
                return False
            return True
        # 零速路径
        if self._owns_motion:
            if self._zero_since is None:
                self._zero_since = now
            elif now - self._zero_since >= self._params.ownership_idle_release_sec:
                self._release_ownership_best_effort("uwb_follow_idle")
                self._ownership_failures = 0
        return True  # 零速发布不需要所有权 (只求物理停下)

    def _release_ownership_best_effort(self, reason: str) -> None:
        if not self._owns_motion:
            return
        self._owns_motion = False
        self._zero_since = None
        if self._release_ownership is not None:
            try:
                self._release_ownership(reason)
            except Exception:
                logger.debug("ownership release callback failed", exc_info=True)

    def _publish_locked(self, command: Tuple[float, float, float]) -> None:
        # 未持有运动所有权时不发零速: 手动 /cmd_vel 通道的零会盖住 Nav2 等
        # 其他生产者; "要停"且从未拥有 → 本来就没在动, 无需发。
        if not self._owns_motion and not any(abs(v) > 1e-9 for v in command):
            return
        self._last_command = command
        try:
            self._sink(command[0], command[1], command[2])
            self._sink_failures = 0
        except Exception:
            self._sink_failures += 1
            logger.warning("motion sink failure (%d/%d)",
                           self._sink_failures, self._fault_limit)
            if self._sink_failures >= self._fault_limit:
                self._enter_fault_locked("motion_sink_failed")

    def _enter_fault_locked(self, reason: str) -> None:
        """故障锁存: 零速 + 放所有权, 停在 fault 等 stop() 复位。"""
        try:
            self._sink(0.0, 0.0, 0.0)
        except Exception:
            pass
        self._release_ownership_best_effort(f"uwb_follow_fault:{reason}")
        self._state = STATE_FAULT
        self._last_reason = reason
        logger.error("uwb follow FAULT: %s", reason)
        self._emit_locked(True)

    def _deactivate_locked(self, reason: str, *, publish_zero: bool) -> None:
        if publish_zero:
            try:
                self._sink(0.0, 0.0, 0.0)
            except Exception:
                logger.debug("zero publish during deactivate failed",
                             exc_info=True)
        self._last_command = (0.0, 0.0, 0.0)
        self._release_ownership_best_effort(f"uwb_follow_stop:{reason}")
        self._reset_tracking_locked()

    def _reset_tracking_locked(self) -> None:
        self._filtered_range = None
        self._outlier_streak = 0
        self._gate_blocked = False
        self._gate_reason = "clearance_unavailable"
        self._last_clearance = None
        self._sink_failures = 0
        self._ownership_failures = 0
        self._zero_since = None

    @staticmethod
    def _params_summary(params: FollowParams) -> dict:
        return {
            "follow_distance_m": params.follow_distance_m,
            "approach_band_m": params.approach_band_m,
            "range_gain": params.range_gain,
            "max_vx": params.max_vx,
            "max_wz": params.max_wz,
            "gate_stop_distance_m": params.gate_stop_distance_m,
            "gate_resume_margin_m": params.gate_resume_margin_m,
        }

    def _snapshot_locked(self) -> dict:
        fix = self._last_fix or {}
        return {
            "state": self._state,
            "reason": self._last_reason,
            "active": self._state in _ACTIVE_STATES,
            "started_at": self._started_at,
            "ever_had_fix": self._ever_had_fix,
            "last_fix": {
                "range_m": fix.get("range_m"),
                "azimuth_rad": fix.get("azimuth_rad"),
                "age_sec": fix.get("age_sec"),
                "anchor_id": fix.get("anchor_id"),
                "quality": fix.get("quality"),
            } if fix else None,
            "filtered_range_m": self._filtered_range,
            "gate": {
                "blocked": self._gate_blocked,
                "reason": self._gate_reason,
                "clearance_m": self._last_clearance,
            },
            "command": {
                "vx": self._last_command[0],
                "vy": self._last_command[1],
                "wz": self._last_command[2],
            },
            "owns_motion": self._owns_motion,
            "ticks": self._ticks,
            "params": self._params_summary(self._params),
        }

    def _emit_locked(self, force: bool) -> None:
        if self._state_callback is None:
            return
        now = self._monotonic()
        due = (
            force
            or self._last_emitted_state != self._state
            or self._last_emit_at is None
            or now - self._last_emit_at >= self._state_heartbeat_sec
        )
        if not due:
            return
        self._last_emitted_state = self._state
        self._last_emit_at = now
        snapshot = self._snapshot_locked()
        try:
            self._state_callback(snapshot)
        except Exception:
            logger.debug("state callback failed", exc_info=True)


__all__ = [
    "FollowParams",
    "UwbFollowController",
    "compute_follow_velocity",
    "evaluate_obstacle_gate",
    "FOLLOW_PARAM_BOUNDS",
]
