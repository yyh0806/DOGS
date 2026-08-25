"""UWB 钥匙扣跟随控制器状态机测试 (功能 B-2)。

确定性测试: 时钟/测距源/间隙探测/速度发布/所有权全部注入替身, 无 ROS、
无线程、无睡眠。覆盖: 状态机全转移、避障闸门第一优先级(含 fail-closed 与
迟滞)、控制律、野值剔除、超时自动停、故障锁存、所有权生命周期、并发安全。
"""
import math
import threading
import time
from fractions import Fraction

import pytest

from nx_uwb_follow import (
    STATE_FAULT,
    STATE_FOLLOWING,
    STATE_IDLE,
    STATE_OBSTACLE_HOLD,
    STATE_SEARCHING,
    FollowParams,
    UwbFollowController,
    compute_follow_velocity,
    evaluate_obstacle_gate,
)


class FakeClock:
    """手动推进的单调时钟。"""

    def __init__(self, start=1000.0):
        self.now = float(start)

    def __call__(self):
        return self.now

    def advance(self, dt):
        self.now += float(dt)


class FakeSource:
    """拉模式 UWB 测距替身。"""

    def __init__(self):
        self.fix = None
        self.calls = 0

    def get_fix(self):
        self.calls += 1
        return self.fix

    def set(self, range_m, azimuth_rad=None, age_sec=None, **extra):
        fix = {"ok": True, "range_m": range_m}
        if azimuth_rad is not None:
            fix["azimuth_rad"] = azimuth_rad
        if age_sec is not None:
            fix["age_sec"] = age_sec
        fix.update(extra)
        self.fix = fix


class Recorder:
    """记录速度发布与所有权事件的替身基座。"""

    def __init__(self):
        self.commands = []
        self.acquire_calls = []
        self.release_calls = []
        self.states = []

    def sink(self, vx, vy, wz):
        self.commands.append((vx, vy, wz))

    def acquire(self, reason):
        self.acquire_calls.append(reason)
        return {"ok": True}

    def release(self, reason):
        self.release_calls.append(reason)
        return {"ok": True}

    def on_state(self, snapshot):
        self.states.append(snapshot)


def make_controller(source=None, clearance=5.0, *, params=None,
                    acquire_fails=0, sink_raises=False, probe_raises=False,
                    with_ownership=True, state_callback=False, clock=None):
    clock = clock or FakeClock()
    rec = Recorder()
    failures = {"n": 0}
    acquire_results = {"fail": acquire_fails}

    def probe(direction_deg, half_fov_deg):
        if probe_raises:
            raise RuntimeError("probe boom")
        return clearance() if callable(clearance) else clearance

    def acquire(reason):
        if acquire_results["fail"] > 0:
            acquire_results["fail"] -= 1
            return {"ok": False, "reason": "busy"}
        return rec.acquire(reason)

    def sink(vx, vy, wz):
        if sink_raises:
            raise RuntimeError("sink boom")
        rec.sink(vx, vy, wz)

    controller = UwbFollowController(
        clearance_probe=probe,
        motion_sink=sink,
        uwb_source=source,
        ownership_acquire=acquire if with_ownership else None,
        ownership_release=rec.release if with_ownership else None,
        state_callback=rec.on_state if state_callback else None,
        monotonic=clock,
        params=params,
    )
    return controller, rec, clock


def last_command(rec):
    return rec.commands[-1] if rec.commands else None


# ============================================================================
# 构造与参数校验
# ============================================================================

def test_constructor_requires_gate_and_sink():
    with pytest.raises(ValueError):
        UwbFollowController(clearance_probe=None, motion_sink=lambda a, b, c: None)
    with pytest.raises(ValueError):
        UwbFollowController(clearance_probe=lambda a, b: None, motion_sink=None)


def test_start_without_uwb_source_refused():
    controller, rec, _ = make_controller(source=None)
    result = controller.start()
    assert result["ok"] is False
    assert result["reason"] == "uwb_source_unavailable"
    assert controller.get_state()["state"] == STATE_IDLE


def test_start_rejects_unknown_and_invalid_params():
    controller, _, _ = make_controller(source=FakeSource())
    assert controller.start({"nope": 1})["reason"] == "invalid_params"
    assert controller.start({"follow_distance_m": 99})["reason"] == "invalid_params"
    assert controller.start({"follow_distance_m": "abc"})["reason"] == "invalid_params"
    assert controller.start({"follow_distance_m": True})["reason"] == "invalid_params"


def test_params_bounds_validated():
    with pytest.raises(ValueError):
        FollowParams(follow_distance_m=99).validate()
    with pytest.raises(ValueError):
        FollowParams(gate_resume_margin_m=5.0, gate_stop_distance_m=0.5).validate()
    ok = FollowParams.from_dict({"follow_distance_m": 2.0, "max_vx": 0.3})
    assert ok.follow_distance_m == 2.0 and ok.max_vx == 0.3
    # merge onto base 保留未改键
    merged = FollowParams.from_dict({"max_wz": 0.4}, base=ok)
    assert merged.follow_distance_m == 2.0 and merged.max_wz == 0.4


def test_update_params_runtime():
    controller, _, _ = make_controller(source=FakeSource())
    assert controller.update_params({"follow_distance_m": 1.5})["ok"] is True
    assert controller.update_params({"bogus": 1})["ok"] is False


# ============================================================================
# 状态机主干: searching → following → (hold) → idle
# ============================================================================

def test_start_enters_searching_and_zero_motion_without_fix():
    source = FakeSource()
    controller, rec, _ = make_controller(source=source)
    result = controller.start()
    assert result["ok"] is True and result["state"] == STATE_SEARCHING
    snap = controller.tick()
    assert snap["state"] == STATE_SEARCHING
    # 无测距: 不抢所有权、不发任何速度 (本来就没动)
    assert rec.acquire_calls == []
    assert rec.commands == []


def test_valid_fix_gate_open_transitions_to_following():
    source = FakeSource()
    source.set(range_m=3.0, azimuth_rad=0.0)
    controller, rec, _ = make_controller(source=source)
    controller.start()
    snap = controller.tick()
    assert snap["state"] == STATE_FOLLOWING
    assert snap["owns_motion"] is True
    assert rec.acquire_calls == ["uwb_follow"]
    # vx = 0.6*(3.0-1.2-0.2) = 0.96 → clamp max_vx 0.4 (净空 5m 不再压)
    cmd = last_command(rec)
    assert cmd is not None
    assert cmd[0] == pytest.approx(0.4)
    assert cmd[1] == 0.0 and cmd[2] == pytest.approx(0.0)


def test_within_deadzone_stops_never_reverses():
    source = FakeSource()
    source.set(range_m=1.0)  # 比目标距离近 → 死区内
    controller, rec, _ = make_controller(source=source)
    controller.start()
    snap = controller.tick()
    assert snap["state"] == STATE_FOLLOWING
    cmd = last_command(rec)
    assert cmd in (None, (0.0, 0.0, 0.0))  # 无非零指令 → 不发布


def test_proportional_speed_and_clamp():
    source = FakeSource()
    params = FollowParams(range_gain=0.5, max_vx=0.4)
    controller, rec, _ = make_controller(source=source, clearance=10.0,
                                         params=params)
    controller.start()
    source.set(range_m=2.0)  # 误差 0.8, 死区 0.2 → 0.5*0.6=0.3
    controller.tick()
    assert last_command(rec)[0] == pytest.approx(0.3)
    source.set(range_m=8.0)  # 跳变首帧被野值剔除拒绝
    controller.tick()
    assert last_command(rec)[0] == pytest.approx(0.3)
    source.set(range_m=8.0)  # 连续第 2 帧接受 → clamp 0.4
    controller.tick()
    assert last_command(rec)[0] == pytest.approx(0.4)


def test_close_range_creep_uses_filtered_range():
    # 死区边缘: 1.2+0.2=1.4 以内不动; 1.5 → 0.6*0.1=0.06 缓慢贴近
    source = FakeSource()
    controller, rec, _ = make_controller(source=source, clearance=10.0)
    controller.start()
    source.set(range_m=1.3)
    controller.tick()
    assert last_command(rec) is None  # 死区内零指令不发布
    source.set(range_m=1.5)
    controller.tick()
    assert last_command(rec)[0] == pytest.approx(0.06, abs=1e-6)


def test_azimuth_turns_and_slows_approach():
    source = FakeSource()
    controller, rec, _ = make_controller(source=source, clearance=10.0)
    controller.start()
    # 目标在左 30°: wz=1.2*0.5236≈0.63 → clamp 0.5; vx=0.6*(4-1.4)=1.56→
    # clamp 0.4 再乘 cos(30°)≈0.866 → 0.346
    source.set(range_m=4.0, azimuth_rad=math.radians(30))
    controller.tick()
    vx, vy, wz = last_command(rec)
    assert wz == pytest.approx(0.5)
    assert vx == pytest.approx(0.4 * math.cos(math.radians(30)), rel=1e-3)


def test_azimuth_deadzone_no_turn():
    source = FakeSource()
    controller, rec, _ = make_controller(source=source, clearance=10.0)
    controller.start()
    source.set(range_m=4.0, azimuth_rad=0.05)  # < 0.08 死区
    controller.tick()
    assert last_command(rec)[2] == 0.0


def test_range_only_fix_drives_straight():
    # 一维测距 (无 azimuth): 只跟不转
    source = FakeSource()
    controller, rec, _ = make_controller(source=source, clearance=10.0)
    controller.start()
    source.set(range_m=4.0)
    controller.tick()
    vx, vy, wz = last_command(rec)
    assert vx > 0 and wz == 0.0


# ============================================================================
# 避障闸门 (第一优先级)
# ============================================================================

def test_gate_blocks_forward_motion_state_obstacle_hold():
    source = FakeSource()
    source.set(range_m=4.0)
    controller, rec, _ = make_controller(source=source, clearance=0.3)
    controller.start()
    snap = controller.tick()
    assert snap["state"] == STATE_OBSTACLE_HOLD
    assert snap["gate"]["blocked"] is True
    cmd = last_command(rec)
    # 全零指令且未持有所有权 → 不发布 (本来就没动); 持有则发布零速
    assert cmd in (None, (0.0, 0.0, 0.0))
    assert snap["command"]["vx"] == 0.0 and snap["command"]["wz"] == 0.0


def test_gate_blocked_still_allows_in_place_turn():
    source = FakeSource()
    source.set(range_m=4.0, azimuth_rad=0.6)
    controller, rec, _ = make_controller(source=source, clearance=0.3)
    controller.start()
    snap = controller.tick()
    assert snap["state"] == STATE_OBSTACLE_HOLD
    vx, vy, wz = last_command(rec)
    assert vx == 0.0 and vy == 0.0
    assert wz == pytest.approx(0.5)  # 原地转向保留 (不平移)


def test_gate_fail_closed_when_probe_raises_or_missing():
    source = FakeSource()
    source.set(range_m=4.0)
    controller, rec, _ = make_controller(source=source, probe_raises=True)
    controller.start()
    snap = controller.tick()
    assert snap["state"] == STATE_OBSTACLE_HOLD
    assert snap["gate"]["reason"] == "clearance_unavailable"
    assert last_command(rec) in (None, (0.0, 0.0, 0.0))
    # 探测返回 None 同样 fail-closed
    controller2, rec2, _ = make_controller(source=None, clearance=None)
    controller2.ingest_fix({"ok": True, "range_m": 4.0})
    controller2.start()
    snap2 = controller2.tick()
    assert snap2["state"] == STATE_OBSTACLE_HOLD
    assert snap2["gate"]["blocked"] is True


def test_gate_hysteresis_prevents_chatter():
    source = FakeSource()
    source.set(range_m=4.0)
    controller, rec, _ = make_controller(source=source, clearance=0.55)
    controller.start()
    # stop=0.5 margin=0.15: 0.55 开 → following
    assert controller.tick()["state"] == STATE_FOLLOWING
    # 0.49 → 挡
    controller._probe = lambda d, f: 0.49
    assert controller.tick()["state"] == STATE_OBSTACLE_HOLD
    # 0.58 (<0.5+0.15=0.65) 仍挡 (迟滞)
    controller._probe = lambda d, f: 0.58
    snap = controller.tick()
    assert snap["state"] == STATE_OBSTACLE_HOLD
    # 0.66 ≥ 0.65 放行
    controller._probe = lambda d, f: 0.66
    snap = controller.tick()
    assert snap["state"] == STATE_FOLLOWING


def test_gate_taper_slows_near_obstacle():
    source = FakeSource()
    controller, rec, _ = make_controller(source=source, clearance=1.0)
    controller.start()
    source.set(range_m=6.0)  # 本应 0.4 上限
    controller.tick()
    # taper: 0.8*(1.0-0.5)=0.4 → 不压; 再贴近: 0.8*(0.75-0.5)=0.2
    controller._probe = lambda d, f: 0.75
    controller.tick()
    assert last_command(rec)[0] == pytest.approx(0.2)


def test_gate_evaluated_before_follow_regression_priority():
    # 闸门纯函数的直接回归: 挡住优先于任何跟随速度
    params = FollowParams()
    blocked, reason = evaluate_obstacle_gate(0.2, False, params)
    assert blocked is True and reason == "clearance_below_stop"
    vx, vy, wz = compute_follow_velocity(8.0, None, 2.0, params)
    assert vx > 0  # 控制律本身会给速度 → 由 tick 的闸门强制清零 (上覆测试)
    # 净空低于挡距时控制律自身也被 taper 压到 0 (双保险)
    assert compute_follow_velocity(8.0, None, 0.2, params)[0] == 0.0


# ============================================================================
# 测距有效性 / 野值剔除 / 超时
# ============================================================================

def test_fix_staleness_and_bounds_rejected():
    source = FakeSource()
    controller, rec, _ = make_controller(source=source, clearance=10.0)
    controller.start()
    source.set(range_m=4.0, age_sec=5.0)  # 陈旧
    assert controller.tick()["state"] == STATE_SEARCHING
    source.set(range_m=0.01)              # 近场盲区
    assert controller.tick()["state"] == STATE_SEARCHING
    source.set(range_m=999.0)             # 超范围
    assert controller.tick()["state"] == STATE_SEARCHING
    source.set(range_m=4.0, azimuth_rad="bad")
    assert controller.tick()["state"] == STATE_SEARCHING
    source.fix = {"ok": False, "reason": "no_anchor"}
    assert controller.tick()["state"] == STATE_SEARCHING
    assert rec.commands == []


def test_acquire_timeout_auto_stops():
    source = FakeSource()  # 始终无测距
    controller, rec, clock = make_controller(source=source,
                                             params=FollowParams(acquire_timeout_sec=8.0))
    controller.start()
    for _ in range(7):
        clock.advance(1.0)
        snap = controller.tick()
        assert snap["state"] == STATE_SEARCHING
    clock.advance(1.0)
    snap = controller.tick()  # t=8s 到点
    assert snap["state"] == STATE_IDLE
    assert controller.get_state()["reason"] == "target_not_acquired"


def test_target_lost_grace_then_auto_stop_then_reacquire():
    source = FakeSource()
    controller, rec, clock = make_controller(source=source, clearance=10.0,
                                             params=FollowParams(lost_timeout_sec=5.0))
    controller.start()
    source.set(range_m=3.0)
    assert controller.tick()["state"] == STATE_FOLLOWING
    # 丢失 3s: searching + 补发一次零速 (持有所有权)
    source.fix = None
    clock.advance(3.0)
    snap = controller.tick()
    assert snap["state"] == STATE_SEARCHING
    assert last_command(rec) == (0.0, 0.0, 0.0)
    # 恢复测距 → 回 following (还在 grace 内)
    source.set(range_m=3.0)
    clock.advance(0.5)
    assert controller.tick()["state"] == STATE_FOLLOWING
    # 再丢 6s → 超时自动停
    source.fix = None
    clock.advance(6.0)
    snap = controller.tick()
    assert snap["state"] == STATE_IDLE
    assert controller.get_state()["reason"] == "target_lost"
    # 停止时补发零速 + 释放所有权
    assert last_command(rec) == (0.0, 0.0, 0.0)
    assert any("uwb_follow_stop" in r for r in rec.release_calls)


def test_outlier_rejection_then_snap():
    source = FakeSource()
    controller, rec, _ = make_controller(source=source, clearance=10.0)
    controller.start()
    source.set(range_m=3.0)
    controller.tick()  # 滤波基准 3.0
    # 单帧跳到 10m (野值): 距离滤波拒绝, 速度按 3.0 不飙升
    source.set(range_m=10.0)
    snap = controller.tick()
    assert snap["filtered_range_m"] == pytest.approx(3.0)
    # 连续第 2 帧 10m → 接受 (目标真的跑远了)
    snap = controller.tick()
    assert snap["filtered_range_m"] == pytest.approx(10.0)


def test_ingest_fix_push_mode_and_freshest_wins():
    controller, rec, clock = make_controller(source=None, clearance=10.0)
    controller.ingest_fix({"ok": True, "range_m": 3.0})
    assert controller.start()["ok"] is True  # 推模式源也算已接源
    snap = controller.tick()
    assert snap["state"] == STATE_FOLLOWING
    assert snap["last_fix"]["range_m"] == pytest.approx(3.0)
    # 陈旧推送 → searching
    clock.advance(2.0)
    snap = controller.tick()
    assert snap["state"] == STATE_SEARCHING


def test_push_and_pull_freshest_wins():
    source = FakeSource()
    source.set(range_m=8.0, age_sec=0.5)   # 源: 较旧
    controller, rec, _ = make_controller(source=source, clearance=10.0)
    controller.start()
    controller.ingest_fix({"ok": True, "range_m": 2.0})  # 推: 新鲜
    snap = controller.tick()
    assert snap["filtered_range_m"] == pytest.approx(2.0)


# ============================================================================
# 停止 / 故障 / 所有权
# ============================================================================

def test_stop_publishes_zero_and_releases_ownership():
    source = FakeSource()
    source.set(range_m=3.0)
    controller, rec, _ = make_controller(source=source)
    controller.start()
    controller.tick()
    assert controller.get_state()["owns_motion"] is True
    result = controller.stop("operator_stop")
    assert result["ok"] is True
    assert controller.get_state()["state"] == STATE_IDLE
    assert last_command(rec) == (0.0, 0.0, 0.0)
    assert any("uwb_follow_stop:operator_stop" in r for r in rec.release_calls)
    # 停止后再 tick 无副作用
    controller.tick()
    assert last_command(rec) == (0.0, 0.0, 0.0)


def test_start_idempotent_while_active():
    source = FakeSource()
    source.set(range_m=3.0)
    controller, _, _ = make_controller(source=source)
    controller.start()
    controller.tick()
    again = controller.start()
    assert again["ok"] is True and again["already_active"] is True


def test_motion_sink_failures_latch_fault():
    source = FakeSource()
    source.set(range_m=3.0)
    controller, rec, _ = make_controller(source=source, sink_raises=True)
    controller.start()
    for _ in range(3):
        snap = controller.tick()
    assert snap["state"] == STATE_FAULT
    assert controller.get_state()["reason"] == "motion_sink_failed"
    # fault 后 start 被拒, 需 stop 复位
    assert controller.start()["reason"] == "fault_latched"
    controller.stop("reset_after_fault")
    assert controller.get_state()["state"] == STATE_IDLE
    assert controller.start()["ok"] is True


def test_ownership_acquire_failure_retries_then_fault():
    source = FakeSource()
    source.set(range_m=3.0)
    controller, rec, _ = make_controller(source=source, acquire_fails=99)
    controller.start()
    for _ in range(5):
        snap = controller.tick()
        # 获取失败 → 不发布非零速度
        assert not rec.commands or all(v == (0.0, 0.0, 0.0) for v in rec.commands)
    assert snap["state"] == STATE_FAULT
    assert controller.get_state()["reason"] == "ownership_exhausted"


def test_ownership_idle_release_and_reacquire():
    source = FakeSource()
    source.set(range_m=1.0)  # 死区内 → 零速
    controller, rec, clock = make_controller(
        source=source, params=FollowParams(ownership_idle_release_sec=3.0))
    controller.start()
    controller.tick()  # following, 零指令 → 不抢所有权
    assert controller.get_state()["owns_motion"] is False
    # 目标走远 → 非零 → 抢所有权 (2.0 首帧被剔除, 3.0 连续两帧接受)
    source.set(range_m=3.0)
    clock.advance(1.0)
    controller.tick()
    clock.advance(0.1)
    controller.tick()
    assert controller.get_state()["owns_motion"] is True
    # 目标停回死区 → 零速 ≥3s → 自动释放 (狗回锁关节, 省电)
    # (回程跳变先被野值剔除 1 拍, 再建立 zero_since, 之后满 3s 释放)
    source.set(range_m=1.0)
    for _ in range(7):
        clock.advance(1.0)
        controller.tick()
    assert controller.get_state()["owns_motion"] is False
    assert any("idle" in r for r in rec.release_calls)


def test_no_ownership_callbacks_still_follows():
    # 独立部署 (无宿主仲裁): 直发速度
    source = FakeSource()
    source.set(range_m=3.0)
    controller, rec, _ = make_controller(source=source, with_ownership=False)
    controller.start()
    snap = controller.tick()
    assert snap["state"] == STATE_FOLLOWING
    assert snap["owns_motion"] is True
    assert last_command(rec)[0] > 0


# ============================================================================
# 快照 / 回调 / 并发
# ============================================================================

def test_state_snapshot_contract():
    source = FakeSource()
    source.set(range_m=3.0, azimuth_rad=0.1, anchor_id="keyfob-01", quality=90)
    controller, rec, _ = make_controller(source=source, state_callback=True)
    controller.start()
    snap = controller.tick()
    for key in ("state", "reason", "active", "ever_had_fix", "last_fix",
                "filtered_range_m", "gate", "command", "owns_motion",
                "ticks", "params"):
        assert key in snap
    assert snap["last_fix"]["anchor_id"] == "keyfob-01"
    assert snap["gate"]["clearance_m"] == pytest.approx(5.0)
    assert snap["params"]["follow_distance_m"] == pytest.approx(1.2)
    # state_callback: start 强推一次 + 状态变化推
    assert rec.states, "state callback must fire"
    assert rec.states[-1]["state"] == STATE_FOLLOWING


def test_tick_returns_snapshot_and_is_side_effect_free():
    source = FakeSource()
    source.set(range_m=3.0)
    controller, rec, _ = make_controller(source=source)
    controller.start()
    controller.tick()
    before = len(rec.commands)
    snap = controller.get_state()
    assert len(rec.commands) == before
    assert snap["state"] == STATE_FOLLOWING


def test_concurrent_tick_stop_is_thread_safe():
    source = FakeSource()
    source.set(range_m=3.0)
    controller, rec, _ = make_controller(source=source)
    controller.start()
    errors = []

    def hammer():
        try:
            for _ in range(200):
                controller.tick()
        except Exception as exc:  # pragma: no cover
            errors.append(exc)

    threads = [threading.Thread(target=hammer) for _ in range(4)]
    threads.append(threading.Thread(
        target=lambda: [controller.stop("t") for _ in range(50)]))
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=10)
    assert not errors
    assert controller.get_state()["state"] in (STATE_IDLE, STATE_FOLLOWING)


def test_default_monotonic_is_real_clock():
    # 默认 time.monotonic 注入 (集成冒烟: 真时钟下 tick 不抛)
    source = FakeSource()
    source.set(range_m=3.0)
    controller = UwbFollowController(
        clearance_probe=lambda d, f: 5.0,
        motion_sink=lambda vx, vy, wz: None,
        uwb_source=source,
    )
    assert controller.start()["ok"] is True
    snap = controller.tick()
    assert snap["state"] == STATE_FOLLOWING
