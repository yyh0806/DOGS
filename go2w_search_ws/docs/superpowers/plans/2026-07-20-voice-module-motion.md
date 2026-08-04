# 语音运动指令 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 PC 麦克风说的"前进两米/后退一米/左转45度/右转"等运动指令通过 `/api/command` 解析并在狗上执行——前进/后退走 nav2（绕行避障），左/右转走 cmd_vel+odom 闭环（精确原地转）。

**Architecture:** PC 端 `voice_console` 维持"只听+发原文+播报"。NLU 扩展在狗的 `parse_product_command` 加运动解析；新增任务类型 `move_relative`（**不能用 `move`——已被 `search_area` waypoint 执行占用**，`nx_web_server.py:1794`）；执行器 linear 走 `PointNavigationController.submit`，angular 走 cmd_vel+odom 闭环。语音/文本/快捷按钮三入口统一走 `/api/command`，`quickCmd('前进两米')` 死按钮自动复活。

**Tech Stack:** Python ≥ 3.10, nav2 `NavigateToPose`, ROS `Twist` `/cmd_vel`, pytest, Vosk STT（PC）。

**Spec:** `docs/superpowers/specs/2026-07-20-voice-module-design.md` Section 1 + Section 3。

## Global Constraints

- **任务类型用 `move_relative`，绝不用 `move`**（`move` 被 `search_area` waypoint 执行占用，`nx_web_server.py:1794`；混用会破坏 lawn-mower 搜索）。
- 距离上限 20m（截断时 `clamped=True`）；角度无上限。
- 默认值：前进/后退 1m，左/右转 90°。
- 转向速度 `vyaw=0.5 rad/s`，yaw 到位容差 3°，兜底硬超时 `(角度/0.5)×2 + 1s`。
- 前进/后退走 nav2（遇障绕行或 abort）；左/右转走 cmd_vel+odom（原地转，无避障需求）。
- 坐标系：map frame 绝对位姿（`PointNavigationController.submit` 已固定 map frame）；cmd_vel 角速度正=左转（`publish_cmd_vel` 透传，`nx_web_server.py:888`）。
- 改动文件需 scp 部署到 NX `~/go2w_ws/web/`（或 `~/go2w/current/payload/web/`）+ `systemctl restart go2w-web`。
- Python ≥ 3.10，纯函数式执行器（注入 read_yaw/send_cmd_vel callable，便于单测）。

---

## File Structure

- `web/nx_product_command.py`（修改）— 加运动解析 + 中文数字解析。与 search_room 解析并列，先于搜索匹配。
- `web/nx_mission_schema.py`（修改）— 加 `canonicalize_move_tasks`，仿 `canonicalize_search_tasks`。
- `web/nx_move_executor.py`（新建）— 纯函数执行器：目标点计算 + yaw 误差 + 闭环转向逻辑（无 ROS 依赖，易测试）。
- `web/nx_web_server.py`（修改）— TaskManager 加 `_execute_move_relative` + admit 按 type 分流 + dispatch 注册 + 注入 `point_nav`。
- `tools/voice_console.py`（修改）— `validate_voice_command` 放行运动指令；`_on_ws_message` 加 `move_result` TTS。
- `web/static/panel.html`（修改）— 运动快捷按钮加无参数版（"前进/后退/左转/右转"）。
- 测试：`web/test_product_command.py`、`web/test_mission_schema.py`、`web/test_move_executor.py`（新建）、`web/test_task_manager_move.py`（新建）、`tools/test_voice_console.py`。

---

### Task 1: NLU 运动指令解析（含中文数字）

**Files:**
- Modify: `web/nx_product_command.py`
- Test: `web/test_product_command.py`

**Interfaces:**
- Produces: `parse_product_command(text)` 现在能返回 `{"tasks":[{"type":"move_relative","priority":5,"params":{"mode","direction","distance_m"|"angle_deg","clamped"}}], "response":str}`；不匹配时返回 None（与现有 search_room 路径一致）。
- 后续 Task 2 的 `canonicalize_move_tasks` 消费这里产出的 task dict。

- [ ] **Step 1: 写失败测试**

追加到 `web/test_product_command.py`：

```python
from nx_product_command import parse_product_command


def _move_task(text):
    result = parse_product_command(text)
    assert result is not None, f"未解析出运动指令: {text!r}"
    tasks = result["tasks"]
    assert len(tasks) == 1 and tasks[0]["type"] == "move_relative"
    return tasks[0]["params"]


def test_move_forward_default():
    p = _move_task("前进")
    assert p == {"mode": "linear", "direction": "forward",
                 "distance_m": 1.0, "clamped": False}


def test_move_forward_two_meters_chinese():
    p = _move_task("前进两米")
    assert p["mode"] == "linear" and p["direction"] == "forward"
    assert p["distance_m"] == 2.0 and p["clamped"] is False


def test_move_forward_half_meter():
    p = _move_task("前进半米")
    assert p["distance_m"] == 0.5


def test_move_backward_arabic():
    p = _move_task("后退1.5米")
    assert p["mode"] == "linear" and p["direction"] == "backward"
    assert p["distance_m"] == 1.5


def test_move_left_default():
    p = _move_task("左转")
    assert p == {"mode": "angular", "direction": "left",
                 "angle_deg": 90.0, "clamped": False}


def test_move_left_45_chinese():
    p = _move_task("左转四十五度")
    assert p["angle_deg"] == 45.0


def test_move_right_half_turn():
    p = _move_task("右转半圈")
    assert p["angle_deg"] == 180.0


def test_move_distance_clamped():
    p = _move_task("前进一百米")
    assert p["distance_m"] == 20.0 and p["clamped"] is True


def test_move_does_not_trigger_on_search():
    """"搜索前面的房间" 不应误触发前进。"""
    assert parse_product_command("搜索这个房间里所有人") is not None
    result = parse_product_command("搜索会议室里所有人")
    assert result["tasks"][0]["type"] == "search_room"


def test_move_negation_rejected():
    assert parse_product_command("别前进") is None
```

- [ ] **Step 2: 运行测试验证失败**

Run: `cd go2w_search_ws && python -m pytest web/test_product_command.py -k move -v`
Expected: FAIL（运动解析尚不存在，`_move_task` 里 `parse_product_command` 对"前进"返回 None，AssertionError）。

- [ ] **Step 3: 实现运动解析**

在 `web/nx_product_command.py` 顶部常量区追加：

```python
_MOVE_FORWARD_TERMS = ("前进", "向前走", "往前走", "直走")
_MOVE_BACKWARD_TERMS = ("后退", "向后走", "往后走", "倒退")
_MOVE_LEFT_TERMS = ("左转", "向左转", "左转弯", "往左转")
_MOVE_RIGHT_TERMS = ("右转", "向右转", "右转弯", "往右转")

_MOVE_DEFAULT_DISTANCE_M = 1.0
_MOVE_DEFAULT_ANGLE_DEG = 90.0
_MOVE_MAX_DISTANCE_M = 20.0

_CN_DIGIT = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
             "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}
```

在文件末尾（`_finite_float` 之后）追加解析函数：

```python
import re as _re


def _parse_chinese_number(text: str) -> float | None:
    """解析中文/阿拉伯数字。支持: 1, 1.5, 一, 两, 半, 十, 十二, 二十, 一百。"""
    text = (text or "").strip()
    if not text:
        return None
    if _re.fullmatch(r"\d+(\.\d+)?", text):
        return float(text)
    if text == "半":
        return 0.5
    if text == "十":
        return 10.0
    if text.startswith("十") and len(text) == 2 and text[1] in _CN_DIGIT:
        return 10.0 + _CN_DIGIT[text[1]]
    if text.endswith("十") and len(text) == 2 and text[0] in _CN_DIGIT:
        return _CN_DIGIT[text[0]] * 10.0
    if text in ("一百", "壹百"):
        return 100.0
    if len(text) == 1 and text in _CN_DIGIT:
        return float(_CN_DIGIT[text])
    return None


def _extract_amount(text: str, unit_chars: tuple[str, ...]) -> float | None:
    """从文本抽 "数字+单位" 的数值。unit_chars=("米","公尺") 或 ("度","°","圈")。"""
    unit_alt = "|".join(_re.escape(u) for u in unit_chars)
    m = _re.search(rf"([\d.]+|[零一二两三四五六七八九十百半]+)\s*({unit_alt})", text)
    if not m:
        return None
    raw, unit = m.group(1), m.group(2)
    value = _parse_chinese_number(raw)
    if value is None:
        return None
    if unit == "圈":
        value *= 360.0
    return value


def _detect_move_direction(text: str) -> str | None:
    if _contains_any(text, _MOVE_FORWARD_TERMS):
        return "forward"
    if _contains_any(text, _MOVE_BACKWARD_TERMS):
        return "backward"
    if _contains_any(text, _MOVE_LEFT_TERMS):
        return "left"
    if _contains_any(text, _MOVE_RIGHT_TERMS):
        return "right"
    return None


def _parse_move_command(text: str) -> dict | None:
    direction = _detect_move_direction(text)
    if direction is None:
        return None
    if direction in ("forward", "backward"):
        amount = _extract_amount(text, ("米", "公尺")) or _MOVE_DEFAULT_DISTANCE_M
        clamped = False
        if amount > _MOVE_MAX_DISTANCE_M:
            amount = _MOVE_MAX_DISTANCE_M
            clamped = True
        return {"mode": "linear", "direction": direction,
                "distance_m": amount, "clamped": clamped}
    amount = _extract_amount(text, ("度", "°", "圈")) or _MOVE_DEFAULT_ANGLE_DEG
    return {"mode": "angular", "direction": direction,
            "angle_deg": amount, "clamped": False}


def _move_command_result(move: dict) -> dict:
    params = {"mode": move["mode"], "direction": move["direction"],
              "clamped": move["clamped"]}
    if move["mode"] == "linear":
        params["distance_m"] = move["distance_m"]
    else:
        params["angle_deg"] = move["angle_deg"]
    dir_cn = {"forward": "前进", "backward": "后退",
              "left": "左转", "right": "右转"}[move["direction"]]
    amount = move.get("distance_m", move.get("angle_deg"))
    unit = "米" if move["mode"] == "linear" else "度"
    return {
        "response": f"{dir_cn}{amount}{unit}",
        "tasks": [{"type": "move_relative", "priority": 5, "params": params}],
    }
```

修改 `parse_product_command`，在现有 `_is_current_room_person_search` 调用**之前**插入运动匹配（`_detect_move_direction` 只认"前进/向前走"等完整词，不认单字"前"，故不会误触发"搜索前面房间"）：

```python
def parse_product_command(text: str) -> dict | None:
    normalized = _normalize_text(text)
    if not normalized:
        return None
    if _contains_any(normalized, _NEGATION_TERMS):
        return None

    move = _parse_move_command(normalized)
    if move is not None:
        return _move_command_result(move)

    # ↓↓↓ 现有 search_room 解析逻辑保持不变 ↓↓↓
    if _is_current_room_person_search(normalized):
        return _command_result(_CURRENT_ROOM)
    # ... (其余不变)
```

- [ ] **Step 4: 运行测试验证通过**

Run: `cd go2w_search_ws && python -m pytest web/test_product_command.py -v`
Expected: PASS（新运动测试全绿，现有 search_room 测试不回归）。

- [ ] **Step 5: Commit**

```bash
git add web/nx_product_command.py web/test_product_command.py
git commit -m "feat(voice): parse move_relative commands with chinese numerals"
```

---

### Task 2: move_relative 任务 canonicalize

**Files:**
- Modify: `web/nx_mission_schema.py`
- Test: `web/test_mission_schema.py`

**Interfaces:**
- Produces: `canonicalize_move_tasks(tasks) -> list[dict]`，校验单个 `move_relative` task，规范化 params，非法时抛 `MissionValidationError`。签名与 `canonicalize_search_tasks` 对称。
- 后续 Task 4 的 `_admit_command_result` 按 task type 调用它或 `canonicalize_search_tasks`。

- [ ] **Step 1: 写失败测试**

追加到 `web/test_mission_schema.py`：

```python
import pytest
from nx_mission_schema import canonicalize_move_tasks, MissionValidationError


def test_canonicalize_linear_move():
    out = canonicalize_move_tasks([{
        "type": "move_relative", "priority": 5,
        "params": {"mode": "linear", "direction": "forward",
                   "distance_m": 2.0, "clamped": False},
    }])
    assert len(out) == 1
    assert out[0]["type"] == "move_relative"
    assert out[0]["params"]["distance_m"] == 2.0
    assert out[0]["priority"] == 5


def test_canonicalize_angular_move():
    out = canonicalize_move_tasks([{
        "type": "move_relative", "priority": 5,
        "params": {"mode": "angular", "direction": "left",
                   "angle_deg": 45.0, "clamped": False},
    }])
    assert out[0]["params"]["angle_deg"] == 45.0
    assert "distance_m" not in out[0]["params"]


def test_canonicalize_rejects_wrong_type():
    with pytest.raises(MissionValidationError):
        canonicalize_move_tasks([{"type": "search_room", "params": {}}])


def test_canonicalize_rejects_bad_mode():
    with pytest.raises(MissionValidationError):
        canonicalize_move_tasks([{
            "type": "move_relative",
            "params": {"mode": "sideways", "direction": "forward",
                       "distance_m": 1.0},
        }])


def test_canonicalize_rejects_nonpositive_distance():
    with pytest.raises(MissionValidationError):
        canonicalize_move_tasks([{
            "type": "move_relative",
            "params": {"mode": "linear", "direction": "forward",
                       "distance_m": 0.0},
        }])
```

- [ ] **Step 2: 运行验证失败**

Run: `cd go2w_search_ws && python -m pytest web/test_mission_schema.py -k move -v`
Expected: FAIL（`canonicalize_move_tasks` 不存在，ImportError）。

- [ ] **Step 3: 实现 canonicalize**

在 `web/nx_mission_schema.py` 的 `canonicalize_search_tasks` 之后追加：

```python
_MOVE_MODES = frozenset({"linear", "angular"})
_MOVE_DIRECTIONS = frozenset({"forward", "backward", "left", "right"})


def canonicalize_move_tasks(tasks: object) -> list[dict]:
    """Validate exactly one move_relative task and return its canonical shape."""
    if not isinstance(tasks, list) or len(tasks) != 1:
        raise MissionValidationError("exactly one move_relative task is required")
    task = tasks[0]
    if not isinstance(task, Mapping) or task.get("type") != "move_relative":
        raise MissionValidationError("task type must be move_relative")
    raw = task.get("params", {})
    if not isinstance(raw, Mapping):
        raise MissionValidationError("move params must be an object")
    mode = str(raw.get("mode", ""))
    direction = str(raw.get("direction", ""))
    if mode not in _MOVE_MODES:
        raise MissionValidationError("invalid move mode")
    if direction not in _MOVE_DIRECTIONS:
        raise MissionValidationError("invalid move direction")
    params: dict = {"mode": mode, "direction": direction,
                    "clamped": bool(raw.get("clamped", False))}
    if mode == "linear":
        distance = _finite_or_raise(raw.get("distance_m"), "distance_m")
        if distance <= 0.0:
            raise MissionValidationError("distance_m must be positive")
        params["distance_m"] = distance
    else:
        angle = _finite_or_raise(raw.get("angle_deg"), "angle_deg")
        if angle <= 0.0:
            raise MissionValidationError("angle_deg must be positive")
        params["angle_deg"] = angle
    try:
        priority = int(task.get("priority", 5))
    except (TypeError, ValueError, OverflowError) as exc:
        raise MissionValidationError("invalid task priority") from exc
    priority = max(0, min(10, priority))
    return [{"type": "move_relative", "priority": priority, "params": params}]


def _finite_or_raise(value, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise MissionValidationError(f"{name} must be a finite number") from exc
    if not math.isfinite(parsed):
        raise MissionValidationError(f"{name} must be a finite number")
    return parsed
```

把 `canonicalize_move_tasks` 加入 `__all__`：

```python
__all__ = [
    "MissionValidationError", "SCHEMA_VERSION", "SearchMissionRequest",
    "canonicalize_search_tasks", "canonicalize_move_tasks",
    "normalize_target_class",
]
```

- [ ] **Step 4: 运行验证通过**

Run: `cd go2w_search_ws && python -m pytest web/test_mission_schema.py -v`
Expected: PASS。

- [ ] **Step 5: Commit**

```bash
git add web/nx_mission_schema.py web/test_mission_schema.py
git commit -m "feat(voice): canonicalize move_relative tasks"
```

---

### Task 3: move 执行器纯函数（目标点 + yaw 闭环）

**Files:**
- Create: `web/nx_move_executor.py`
- Test: `web/test_move_executor.py`

**Interfaces:**
- Produces（全部纯函数，无 ROS 依赖）:
  - `compute_linear_target(x, y, yaw, direction, distance_m) -> tuple[float,float,float]`：返回 `(target_x, target_y, target_yaw)`。
  - `compute_angular_target_yaw(current_yaw, direction, angle_deg) -> float`。
  - `yaw_error(current_yaw, target_yaw) -> float`：归一到 `[-pi, pi]`。
  - `angular_turn_complete(current_yaw, target_yaw, tolerance_rad) -> bool`。
  - `run_angular_turn(read_yaw, send_cmd_vel, sleep, monotonic, target_yaw, direction, *, vyaw, tolerance_rad, max_duration) -> str`：返回 `"succeeded"` 或 `"timed_out"`。`read_yaw()`/`send_cmd_vel(vx,vy,vyaw)`/`sleep(s)`/`monotonic()` 全部注入，便于单测。
- 后续 Task 4 的 `_execute_move_relative` 消费这些函数。

- [ ] **Step 1: 写失败测试**

新建 `web/test_move_executor.py`：

```python
import math
import pytest
from nx_move_executor import (
    compute_linear_target, compute_angular_target_yaw, yaw_error,
    angular_turn_complete, run_angular_turn,
)


def test_linear_forward_adds_distance_along_yaw():
    tx, ty, tyaw = compute_linear_target(1.0, 0.0, 0.0, "forward", 2.0)
    assert (tx, ty, tyaw) == pytest.approx((3.0, 0.0, 0.0))


def test_linear_backward_subtracts_distance():
    tx, ty, tyaw = compute_linear_target(3.0, 4.0, 0.0, "backward", 1.0)
    assert (tx, ty, tyaw) == pytest.approx((2.0, 4.0, 0.0))


def test_linear_forward_45deg():
    yaw = math.radians(45)
    tx, ty, _ = compute_linear_target(0.0, 0.0, yaw, "forward", math.sqrt(2))
    assert (tx, ty) == pytest.approx((1.0, 1.0), abs=1e-6)


def test_angular_left_adds_angle():
    assert compute_angular_target_yaw(0.0, "left", 90.0) == pytest.approx(math.radians(90))


def test_angular_right_subtracts_angle():
    assert compute_angular_target_yaw(math.radians(10), "right", 45.0) == pytest.approx(math.radians(-35))


def test_yaw_error_wraps_to_neg_pi_pi():
    assert yaw_error(0.0, math.radians(350)) == pytest.approx(math.radians(-10), abs=1e-6)
    assert yaw_error(0.0, math.radians(10)) == pytest.approx(math.radians(10), abs=1e-6)


def test_angular_turn_complete_within_tolerance():
    assert angular_turn_complete(0.0, math.radians(2), math.radians(3)) is True
    assert angular_turn_complete(0.0, math.radians(10), math.radians(3)) is False


def test_run_angular_turn_succeeds_when_yaw_reaches_target():
    target = math.radians(90)
    yaw_readings = [0.0, math.radians(30), math.radians(60), math.radians(89)] + [target] * 20
    idx = {"i": 0}
    read_yaw = lambda: yaw_readings[min(idx["i"], len(yaw_readings) - 1)]
    sent = []
    send_cmd = lambda vx, vy, vyaw: sent.append((vx, vy, vyaw))

    def sleep(_s):
        idx["i"] += 1

    result = run_angular_turn(read_yaw, send_cmd, sleep, lambda: 0.0,
                              target, "left", vyaw=0.5,
                              tolerance_rad=math.radians(3), max_duration=10.0)
    assert result == "succeeded"
    assert sent[0][2] == 0.5           # 起步: 左转正 vyaw
    assert sent[-1] == (0.0, 0.0, 0.0)  # 结束: 停


def test_run_angular_turn_times_out_when_yaw_never_reaches():
    read_yaw = lambda: 0.0  # 永不动
    sent = []
    send_cmd = lambda vx, vy, vyaw: sent.append((vx, vy, vyaw))
    clock = iter(range(0, 100, 2))
    monotonic = lambda: next(clock)
    result = run_angular_turn(read_yaw, send_cmd, lambda _s: None, monotonic,
                              math.radians(90), "right", vyaw=0.5,
                              tolerance_rad=math.radians(3), max_duration=1.0)
    assert result == "timed_out"
    assert sent[-1] == (0.0, 0.0, 0.0)  # 超时也停
```

- [ ] **Step 2: 运行验证失败**

Run: `cd go2w_search_ws && python -m pytest web/test_move_executor.py -v`
Expected: FAIL（`nx_move_executor` 模块不存在，ImportError）。

- [ ] **Step 3: 实现执行器**

新建 `web/nx_move_executor.py`：

```python
"""Pure-function move execution primitives for move_relative tasks.

No ROS imports here — callers inject read_yaw/send_cmd_vel/sleep/monotonic so
the closed-loop turn logic is unit-testable without nav2 or hardware.
"""

from __future__ import annotations

import math
from typing import Callable


def compute_linear_target(
    x: float, y: float, yaw: float, direction: str, distance_m: float
) -> tuple[float, float, float]:
    """Forward/backward target pose in the same frame, keeping heading."""
    sign = 1.0 if direction == "forward" else -1.0
    tx = x + sign * distance_m * math.cos(yaw)
    ty = y + sign * distance_m * math.sin(yaw)
    return (tx, ty, yaw)


def compute_angular_target_yaw(
    current_yaw: float, direction: str, angle_deg: float
) -> float:
    """Target yaw after turning in place. Left = +, right = -."""
    delta = math.radians(angle_deg)
    return current_yaw + delta if direction == "left" else current_yaw - delta


def yaw_error(current_yaw: float, target_yaw: float) -> float:
    """Smallest signed difference, wrapped to [-pi, pi]."""
    err = (target_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
    return err


def angular_turn_complete(
    current_yaw: float, target_yaw: float, tolerance_rad: float
) -> bool:
    return abs(yaw_error(current_yaw, target_yaw)) <= tolerance_rad


def run_angular_turn(
    read_yaw: Callable[[], float],
    send_cmd_vel: Callable[[float, float, float], None],
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    target_yaw: float,
    direction: str,
    *,
    vyaw: float = 0.5,
    tolerance_rad: float = math.radians(3.0),
    max_duration: float | None = None,
) -> str:
    """Closed-loop in-place turn. Returns 'succeeded' or 'timed_out'.

    Always publishes a zero-velocity stop before returning so the robot never
    keeps spinning on timeout/exception.
    """
    sign = 1.0 if direction == "left" else -1.0
    if max_duration is None:
        remaining = abs(yaw_error(read_yaw(), target_yaw))
        max_duration = (remaining / max(vyaw, 1e-6)) * 2.0 + 1.0
    deadline = monotonic() + max_duration
    try:
        send_cmd_vel(0.0, 0.0, sign * vyaw)
        while monotonic() < deadline:
            sleep(0.05)
            if angular_turn_complete(read_yaw(), target_yaw, tolerance_rad):
                return "succeeded"
        return "timed_out"
    finally:
        send_cmd_vel(0.0, 0.0, 0.0)


__all__ = [
    "compute_linear_target", "compute_angular_target_yaw", "yaw_error",
    "angular_turn_complete", "run_angular_turn",
]
```

- [ ] **Step 4: 运行验证通过**

Run: `cd go2w_search_ws && python -m pytest web/test_move_executor.py -v`
Expected: PASS（全部 9 个测试绿）。

- [ ] **Step 5: Commit**

```bash
git add web/nx_move_executor.py web/test_move_executor.py
git commit -m "feat(voice): pure-function move executor (linear target + angular loop)"
```

---

### Task 4: TaskManager 接 move_relative（admit 分流 + 执行 + dispatch）

**Files:**
- Modify: `web/nx_web_server.py`（TaskManager）
- Test: `web/test_task_manager_move.py`（新建）

**Interfaces:**
- Consumes: Task 1 的 `parse_product_command` 产出 move_relative task；Task 2 的 `canonicalize_move_tasks`；Task 3 的执行器纯函数；`PointNavigationController.submit/get_state`；`NxWebNode.get_localization_health()`；`NxRobotBridge.move(vx,vy,vyaw,manual=True)`。
- Produces: `TaskManager._execute_move_relative(task)` + admit 按 type 分流 + `set_point_nav(port)` 注入器。WS 广播 `{"type":"move_result","data":{"phase","direction",...}}`。

- [ ] **Step 1: 写失败测试**

新建 `web/test_task_manager_move.py`：

```python
"""TaskManager move_relative dispatch + admit分流 集成测试 (mock point_nav/robot/node)."""
import math
import pytest


@pytest.fixture
def tm_with_mocks():
    from nx_web_server import TaskManager

    class FakePointNav:
        def __init__(self):
            self.submitted = []
            self._state = {"status": "idle"}

        def submit(self, x, y, yaw):
            self.submitted.append((x, y, yaw))
            self._state = {"status": "active"}
            return {"ok": True}

        def get_state(self):
            return dict(self._state)

        def set_terminal(self, status):
            self._state = {"status": status}

        def cancel(self, reason=""):
            self._state = {"status": "canceled"}

    class FakeNode:
        def __init__(self):
            self._pose = {"healthy": True, "x": 0.0, "y": 0.0, "yaw": 0.0}

        def get_localization_health(self):
            return dict(self._pose)

    class FakeRobot:
        def __init__(self):
            self._node = FakeNode()
            self.moves = []

        def move(self, vx, vy, vyaw, manual=False):
            self.moves.append((vx, vy, vyaw))

        def stop_move(self):
            self.moves.append(("stop",))

    broadcasts = []
    robot = FakeRobot()
    tm = TaskManager(robot)
    point_nav = FakePointNav()
    tm.set_point_nav(point_nav)
    import nx_web_server
    orig = nx_web_server.ws_broadcast
    nx_web_server.ws_broadcast = lambda payload: broadcasts.append(payload)
    yield tm, point_nav, robot, broadcasts
    nx_web_server.ws_broadcast = orig


def test_admit_routes_move_to_canonicalize(tm_with_mocks):
    """submit_command('前进两米') 应产出 move_relative task, 同步 admit (parser='product')."""
    tm, point_nav, robot, broadcasts = tm_with_mocks
    result = tm.submit_command("前进两米")
    assert result["ok"] is True
    assert result["tasks"][0]["type"] == "move_relative"
    assert result.get("parser") == "product"


def test_execute_linear_submits_nav_goal(tm_with_mocks):
    from nx_web_server import Task
    tm, point_nav, robot, broadcasts = tm_with_mocks
    task = Task("move_relative", {"mode": "linear", "direction": "forward",
                                  "distance_m": 2.0, "clamped": False}, 5)
    point_nav.set_terminal("succeeded")
    tm._execute_move_relative(task)
    assert len(point_nav.submitted) == 1
    tx, ty, tyaw = point_nav.submitted[0]
    assert (tx, ty) == pytest.approx((2.0, 0.0))
    assert task.status == "completed"
    assert any(b["type"] == "move_result" and b["data"]["phase"] == "succeeded"
               for b in broadcasts)


def test_execute_linear_aborted(tm_with_mocks):
    from nx_web_server import Task
    tm, point_nav, robot, broadcasts = tm_with_mocks
    task = Task("move_relative", {"mode": "linear", "direction": "forward",
                                  "distance_m": 1.0, "clamped": False}, 5)
    point_nav.set_terminal("aborted")
    tm._execute_move_relative(task)
    assert task.status == "failed"
    assert any(b["data"]["phase"] == "aborted" for b in broadcasts
               if b["type"] == "move_result")


def test_execute_angular_turns_and_stops(tm_with_mocks):
    from nx_web_server import Task
    tm, point_nav, robot, broadcasts = tm_with_mocks
    poses = [math.radians(a) for a in (0, 20, 40, 60, 80, 89)]
    idx = {"i": 0}
    orig_health = robot._node.get_localization_health

    def health():
        h = orig_health()
        i = min(idx["i"], len(poses) - 1)
        h["yaw"] = poses[i]
        idx["i"] += 1
        return h
    robot._node.get_localization_health = health

    task = Task("move_relative", {"mode": "angular", "direction": "left",
                                  "angle_deg": 90.0, "clamped": False}, 5)
    tm._execute_move_relative(task)
    assert task.status == "completed"
    assert any(len(m) == 3 and m[2] != 0 for m in robot.moves)
    assert robot.moves[-1] == ("stop",) or robot.moves[-1][2] == 0.0
```

- [ ] **Step 2: 运行验证失败**

Run: `cd go2w_search_ws && python -m pytest web/test_task_manager_move.py -v`
Expected: FAIL（`set_point_nav` / `_execute_move_relative` 不存在）。

- [ ] **Step 3: 实现 TaskManager 改动**

在 `web/nx_web_server.py` 顶部导入区，紧邻现有 `from nx_mission_schema import ...` 加：

```python
from nx_mission_schema import (
    canonicalize_search_tasks, canonicalize_move_tasks, MissionValidationError,
)
from nx_move_executor import (
    compute_linear_target, compute_angular_target_yaw, run_angular_turn,
)
```

在 `TaskManager.__init__` 末尾（`self._navigation_arbiter = None` 之后）加：

```python
        self._point_nav = None  # OwnerNavigationPort, set_point_nav 注入
```

在 `set_navigation_arbiter` 之后加注入器：

```python
    def set_point_nav(self, port):
        """Inject the PointNavigationController owner port for linear moves."""
        self._point_nav = port
```

修改 `_admit_command_result` 的 canonicalize 调用（约 1566-1573 行），把单一 `canonicalize_search_tasks` 改为按 type 分流：

```python
        invalid_reason = None
        if tasks:
            try:
                first_type = tasks[0].get("type") if isinstance(tasks[0], dict) else None
                if first_type == "move_relative":
                    tasks = canonicalize_move_tasks(tasks)
                else:
                    tasks = canonicalize_search_tasks(tasks)
            except MissionValidationError as exc:
                logger.warning("拒绝非规范任务: %s", exc)
                response = "任务格式无效"
                tasks = []
                invalid_reason = "invalid_task"
```

在 `_execute_search` 之后追加 `_execute_move_relative` 与 `_await_point_nav_terminal`：

```python
    def _execute_move_relative(self, task):
        p = task.params
        mode = p.get("mode")
        direction = p.get("direction")

        def broadcast(phase, **extra):
            ws_broadcast({"type": "move_result",
                          "data": {"phase": phase, "direction": direction, **extra}})

        node_obj = getattr(self.robot, "_node", None)
        health_getter = getattr(node_obj, "get_localization_health", None)
        health = health_getter() if callable(health_getter) else {}
        if not health.get("healthy"):
            task.status = "failed"
            task.result = "localization_unhealthy"
            broadcast("aborted", reason="localization_unhealthy")
            return

        if mode == "linear":
            if self._point_nav is None:
                task.status = "failed"
                task.result = "point_nav_unavailable"
                broadcast("aborted", reason="point_nav_unavailable")
                return
            x = float(health["x"]); y = float(health["y"]); yaw = float(health["yaw"])
            tx, ty, tyaw = compute_linear_target(x, y, yaw, direction, float(p["distance_m"]))
            self._point_nav.submit(tx, ty, tyaw)
            phase = self._await_point_nav_terminal(task)
            if phase == "succeeded":
                task.status = "completed"
                task.result = {"distance_m": p["distance_m"], "direction": direction}
            else:
                task.status = "failed"
                task.result = phase
            broadcast(phase, distance_m=p["distance_m"])
        else:  # angular
            yaw0 = float(health["yaw"])
            target_yaw = compute_angular_target_yaw(yaw0, direction, float(p["angle_deg"]))

            def read_yaw():
                h = health_getter() if callable(health_getter) else {}
                return float(h.get("yaw", yaw0))

            def send_cmd(vx, vy, vyaw):
                self.robot.move(vx, vy, vyaw, manual=True)

            phase = run_angular_turn(
                read_yaw, send_cmd, time.sleep, time.monotonic,
                target_yaw, direction, vyaw=0.5,
                tolerance_rad=math.radians(3.0),
            )
            self.robot.stop_move()
            if phase == "succeeded":
                task.status = "completed"
                task.result = {"angle_deg": p["angle_deg"], "direction": direction}
            else:
                task.status = "failed"
                task.result = phase
            broadcast(phase, angle_deg=p["angle_deg"])

    def _await_point_nav_terminal(self, task, timeout=60.0):
        """Poll point_nav state until terminal; cancel-aware. Returns phase str."""
        if self._point_nav is None:
            return "aborted"
        terminal = {"succeeded", "aborted", "timed_out", "canceled",
                    "error", "rejected"}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if task.status == "cancelled":
                try:
                    self._point_nav.cancel("task_cancelled")
                except Exception:
                    pass
                return "cancelled"
            status = self._point_nav.get_state().get("status")
            if status in terminal:
                return status
            time.sleep(0.1)
        try:
            self._point_nav.cancel("timeout")
        except Exception:
            pass
        return "timed_out"
```

在 worker dispatch（约 1816 行 `elif task.type == "search_room":` 之前）加 `move_relative` 分支：

```python
                elif task.type == "move_relative":
                    self._execute_move_relative(task)
```

在 `main()` 里 `task_mgr = TaskManager(...)` 之后注入 point_nav：

```python
    task_mgr = TaskManager(robot, vlm_engine=vlm_proxy, detector=detector_proxy)
    task_mgr.set_point_nav(point_nav)
```

- [ ] **Step 4: 运行验证通过**

Run: `cd go2w_search_ws && python -m pytest web/test_task_manager_move.py web/test_product_command.py -v`
Expected: PASS。

- [ ] **Step 5: Commit**

```bash
git add web/nx_web_server.py web/test_task_manager_move.py
git commit -m "feat(voice): TaskManager executes move_relative (nav2 linear + cmd_vel angular)"
```

---

### Task 5: voice_console 放行运动 + WS move_result TTS

**Files:**
- Modify: `tools/voice_console.py`
- Test: `tools/test_voice_console.py`

**Interfaces:**
- Consumes: Task 1 的 `parse_product_command`（现能解析运动）；Task 4 的 WS `move_result` 广播。
- Produces: `validate_voice_command(text)`（替代 `validate_search_command`，放行 move_relative + search_room）；`_on_ws_message` 处理 `move_result` → TTS。

- [ ] **Step 1: 写失败测试**

追加到 `tools/test_voice_console.py`：

```python
from voice_console import validate_voice_command


def test_validate_accepts_forward():
    v = validate_voice_command("前进两米")
    assert v["ok"] is True
    assert v["task"]["type"] == "move_relative"


def test_validate_accepts_turn():
    v = validate_voice_command("左转45度")
    assert v["ok"] is True
    assert v["task"]["params"]["mode"] == "angular"


def test_validate_rejects_unrelated():
    v = validate_voice_command("今天天气真好")
    assert v["ok"] is False
    assert v["reason"] == "unsupported_voice_command"


def test_validate_still_accepts_search():
    v = validate_voice_command("搜索这个房间里所有人")
    assert v["ok"] is True
    assert v["task"]["type"] == "search_room"


def test_dedupe_distinguishes_distances():
    """前进两米 vs 前进一米 是不同 fingerprint, 不互相压制."""
    from voice_console import SearchCommandDispatcher

    sent = []
    d = SearchCommandDispatcher(sender=lambda url, text: sent.append(text) or
                                {"ok": True, "accepted": True})
    assert d.dispatch("http://x", "前进两米").get("ok")
    assert d.dispatch("http://x", "前进一米").get("ok")
```

- [ ] **Step 2: 运行验证失败**

Run: `cd go2w_search_ws && python -m pytest tools/test_voice_console.py -v`
Expected: FAIL（`validate_voice_command` 不存在；旧 `validate_search_command` 拒绝"前进两米"）。

- [ ] **Step 3: 实现 validate_voice_command + move_result TTS**

在 `tools/voice_console.py` 里把 `validate_search_command` 重命名为 `validate_voice_command` 并放宽（去掉"必须是 search_room"硬约束，保留单任务 + fingerprint）：

```python
def validate_voice_command(text: str) -> dict:
    """Allow any deterministic product command (search_room OR move_relative)."""
    raw_text = text.strip() if isinstance(text, str) else ""
    result = parse_product_command(raw_text)
    tasks = result.get("tasks", []) if isinstance(result, dict) else []
    task = tasks[0] if len(tasks) == 1 and isinstance(tasks[0], dict) else None
    if task is None:
        return {"ok": False, "reason": "unsupported_voice_command", "text": raw_text}
    if task.get("type") == "search_room":
        try:
            SearchMissionRequest.from_dict(task["params"]["mission_request"])
        except (MissionValidationError, TypeError, KeyError):
            return {"ok": False, "reason": "unsupported_voice_command", "text": raw_text}
    fingerprint = json.dumps(task, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":"))
    return {
        "ok": True, "text": raw_text,
        "response": result.get("response", ""),
        "task": task,
        "fingerprint": fingerprint,
    }


# 向后兼容别名
validate_search_command = validate_voice_command
```

更新 `SearchCommandDispatcher.dispatch` 与 `main()` 里所有 `validate_search_command(...)` 调用为 `validate_voice_command(...)`（`tools/voice_console.py` 内全局替换调用点）。

在 `_on_ws_message` 的 `search_room` 分支之后追加 `move_result`：

```python
    elif mtype == "move_result":
        phase = payload.get("phase")
        direction = payload.get("direction")
        dir_cn = {"forward": "前进", "backward": "后退",
                  "left": "左转", "right": "右转"}.get(direction, direction)
        amount = payload.get("distance_m") or payload.get("angle_deg")
        unit = "米" if payload.get("distance_m") is not None else "度"
        if phase == "succeeded" and amount:
            speak(f"已{dir_cn}{amount}{unit}")
        elif phase == "aborted":
            reason = payload.get("reason") or ""
            speak(f"无法{dir_cn}：{reason}" if reason else f"无法{dir_cn}")
        elif phase == "timed_out":
            speak(f"{dir_cn}超时，已停止")
```

- [ ] **Step 4: 运行验证通过**

Run: `cd go2w_search_ws && python -m pytest tools/test_voice_console.py -v`
Expected: PASS。

- [ ] **Step 5: Commit**

```bash
git add tools/voice_console.py tools/test_voice_console.py
git commit -m "feat(voice): voice_console allows move_relative + speaks move_result"
```

---

### Task 6: 前端运动快捷按钮（无参数版）

**Files:**
- Modify: `web/static/panel.html:248-251`（现有 4 个 quick-btn 旁加 4 个无参数版）
- Test: `web/test_panel_navigation_contract.py`

**Interfaces:**
- Consumes: Task 1-4 后 `quickCmd('前进')` 走 `/api/command` → NLU 解析 → 默认 1m/90° 执行。复用现有 `quickCmd` JS 函数，无新接口。

- [ ] **Step 1: 写失败测试**

追加到 `web/test_panel_navigation_contract.py`：

```python
from pathlib import Path

PANEL = Path(__file__).resolve().parent / "static" / "panel.html"


def test_panel_has_bare_move_buttons():
    """无参数运动按钮 (前进/后退/左转/右转) 各至少出现一次."""
    html = PANEL.read_text(encoding="utf-8")
    for cmd in ("quickCmd('前进')", "quickCmd('后退')",
                "quickCmd('左转')", "quickCmd('右转')"):
        assert cmd in html, f"缺无参数按钮: {cmd}"


def test_panel_keeps_parameterized_move_buttons():
    """原有带参数按钮不丢失."""
    html = PANEL.read_text(encoding="utf-8")
    assert "quickCmd('前进两米')" in html
    assert "quickCmd('左转90度')" in html
```

- [ ] **Step 2: 运行验证失败**

Run: `cd go2w_search_ws && python -m pytest web/test_panel_navigation_contract.py -k move_buttons -v`
Expected: FAIL（无参数按钮尚不存在）。

- [ ] **Step 3: 加按钮**

在 `web/static/panel.html` 第 251 行（`右转90度` span 之后）插入：

```html
      <span class="quick-btn" onclick="quickCmd('前进')">前进</span>
      <span class="quick-btn" onclick="quickCmd('后退')">后退</span>
      <span class="quick-btn" onclick="quickCmd('左转')">左转</span>
      <span class="quick-btn" onclick="quickCmd('右转')">右转</span>
```

- [ ] **Step 4: 运行验证通过**

Run: `cd go2w_search_ws && python -m pytest web/test_panel_navigation_contract.py -k move_buttons -v`
Expected: PASS。

- [ ] **Step 5: Commit**

```bash
git add web/static/panel.html web/test_panel_navigation_contract.py
git commit -m "feat(voice): add bare move quick-buttons (default 1m/90deg)"
```

---

## 部署与验收（Plan A 完成后）

1. **部署到 NX**（记忆 [[nx-deploy-bare-python]]：scp + restart）：
   ```bash
   scp web/nx_product_command.py web/nx_mission_schema.py web/nx_move_executor.py web/nx_web_server.py nx@<NX_IP>:~/go2w_ws/web/
   scp web/static/panel.html nx@<NX_IP>:~/go2w_ws/web/static/
   ssh nx@<NX_IP> 'sudo systemctl restart go2w-web'
   ```
2. **PC 端**：`pip install -r requirements-voice.txt`（无新依赖）；`python tools/voice_console.py --nx <NX_IP> --token-file ...`
3. **端到端验收**：
   - 文本模式：`python tools/voice_console.py --text "前进两米" --no-auto-send` → 显示 move_relative task
   - 语音模式：说"前进两米" → 狗 nav2 前进 2m → TTS"已前进两米"
   - 说"左转" → 狗原地左转 90° → TTS"已左转 90 度"
   - 说"前进一百米" → 截断 20m → TTS 提示
   - 前方有墙时"前进两米" → nav2 绕行或 abort → TTS"无法前进"

## Self-Review

**Spec 覆盖**：
- Section 1.1 NLU 解析 → Task 1 ✅
- Section 1.2 schema → Task 2 ✅
- Section 1.3 执行（linear nav2 / angular cmd_vel+odom）→ Task 3+4 ✅
- Section 1.4 反馈（move_result + TTS）→ Task 4+5 ✅
- Section 3.1 voice_console 放行 → Task 5 ✅
- Section 3.2 运动快捷按钮 → Task 6 ✅

**Placeholder 扫描**：无 TBD/TODO；每步含完整代码或精确命令。

**Type 一致性**：`move_relative`（非 `move`）在所有 task 一致；`canonicalize_move_tasks` / `_execute_move_relative` / `validate_voice_command` 命名跨 task 对齐；执行器函数签名（Task 3 定义、Task 4 消费）匹配。

**关键约束**：Global Constraints 已注明"绝不用 `move` type"，避免与 `search_area` waypoint 执行冲突（`nx_web_server.py:1794`）。
