"""Process-wide ownership arbitration for all autonomous motion producers.

This module is deliberately ROS-free.  It coordinates the Panel point-goal
controller, TaskManager (including RoomSearchOrchestrator), and operator pose /
manual commands.  Each transition is bounded, but a timeout never authorizes a
new owner: the caller gets a failure and the robot is emergency-stopped.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Iterable, Optional


class NavigationArbiter:
    """Serialize PointNav, room/task navigation, and operator motion."""

    def __init__(
        self,
        point_nav: Any,
        task_manager: Any,
        robot: Any,
        *,
        transition_timeout: float = 3.0,
        drive_activation_timeout: float = 6.0,
        poll_interval: float = 0.05,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._point_nav = point_nav
        self._tasks = task_manager
        self._robot = robot
        self._transition_timeout = self._positive(
            transition_timeout, "transition_timeout"
        )
        self._drive_activation_timeout = self._positive(
            drive_activation_timeout, "drive_activation_timeout"
        )
        self._poll_interval = self._positive(poll_interval, "poll_interval")
        self._monotonic = monotonic
        self._sleep = sleep
        self._lock = threading.RLock()
        self._emergency_lock = threading.Lock()
        self._emergency_epoch = 0
        self._stopping = False
        self._motion_owner: Optional[str] = None

    def get_motion_owner(self) -> Optional[str]:
        """Return the serialized producer name for request-edge auditing."""
        with self._lock:
            return self._motion_owner

    def _read_emergency_epoch(self) -> int:
        with self._emergency_lock:
            return self._emergency_epoch

    @staticmethod
    def _positive(value: Any, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite positive number") from exc
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be a finite positive number")
        return result

    def _point_preflight(self, *, include_health: bool = True) -> Optional[str]:
        try:
            state = self._point_nav.get_state()
        except Exception:
            return "point_nav_unavailable"
        if state.get("quarantined"):
            return "point_nav_quarantined"
        if state.get("stopped"):
            return "point_nav_stopped"
        if include_health and state.get("healthy") is False:
            return "point_nav_unhealthy"
        return None

    def _park_drive(self, reason: str, *, wait: bool = False) -> bool:
        try:
            self._robot.park_drive_session(reason)
        except Exception:
            self._emergency_stop()
            return False
        if not wait:
            return True
        waiter = getattr(self._robot, "wait_drive_parked", None)
        if not callable(waiter):
            return False
        try:
            return bool(waiter(self._drive_activation_timeout))
        except Exception:
            return False

    def _activate_drive(self, owner: str, failure_reason: str) -> dict:
        # An already-balanced wheel session changes software ownership at
        # zero speed.  Parking and re-balancing here would physically alternate
        # StandUp/BalanceStand and can destabilize Go2W during rapid handoffs.
        handoff = False
        state_reader = getattr(self._robot, "get_drive_session_state", None)
        if callable(state_reader):
            try:
                state = dict(state_reader())
            except Exception as exc:
                return {
                    "ok": False,
                    "reason": "drive_session_feedback_error",
                    "message": str(exc),
                }
            if state.get("drive_session") in ("active", "nav_active"):
                # "nav_active" = nav 已激活 (motion_machine 用此值表示 nav owner 激活态),
                # 语义等同 "active": 已激活需零速 handoff, 不应走 wait_drive_parked。
                # 之前只认 "active" → nav_active 被当作未激活 → handoff=False →
                # wait_drive_parked 等 parked 永不满足 → drive_session_not_parked。
                handoff = True
                try:
                    self._robot.stop_move()
                except Exception as exc:
                    return {
                        "ok": False,
                        "reason": "drive_handoff_zero_error",
                        "message": str(exc),
                    }
                deadline = self._monotonic() + self._drive_activation_timeout
                while True:
                    try:
                        state = dict(state_reader())
                    except Exception as exc:
                        return {
                            "ok": False,
                            "reason": "drive_session_feedback_error",
                            "message": str(exc),
                        }
                    session = state.get("drive_session")
                    if session == "parked":
                        handoff = False
                        break
                    wheels = state.get("wheel_dq")
                    try:
                        stopped = (
                            len(wheels) == 4
                            and all(math.isfinite(float(value)) for value in wheels)
                            and sum(abs(float(value)) for value in wheels) / 4.0 < 0.15
                        )
                    except (TypeError, ValueError, OverflowError):
                        stopped = False
                    if (session in ("active", "nav_active")
                            and state.get("sport_mode") in (1, 3)
                            and stopped):
                        break
                    if state.get("drive_fault"):
                        return {"ok": False, "reason": state["drive_fault"]}
                    remaining = deadline - self._monotonic()
                    if remaining <= 0.0:
                        return {
                            "ok": False,
                            "reason": "drive_handoff_not_stopped",
                        }
                    self._sleep(min(self._poll_interval, remaining))

        if not handoff:
            # A pose/stop command may have published the parking request while
            # mode-6 acknowledgement is still in flight.  Serialize on
            # physical feedback before a fresh BalanceStand activation.
            waiter = getattr(self._robot, "wait_drive_parked", None)
            if not callable(waiter):
                return {"ok": False, "reason": "drive_park_feedback_unavailable"}
            try:
                parked = bool(waiter(self._drive_activation_timeout))
            except Exception as exc:
                return {
                    "ok": False,
                    "reason": "drive_session_park_wait_error",
                    "message": str(exc),
                }
            if not parked:
                return {"ok": False, "reason": "drive_session_not_parked"}
        try:
            started = dict(self._robot.start_drive_session(owner))
        except Exception as exc:
            return {"ok": False, "reason": "drive_session_start_error", "message": str(exc)}
        if not started.get("ok"):
            return started
        try:
            ready = dict(self._robot.wait_drive_ready(
                owner, self._drive_activation_timeout))
        except Exception as exc:
            ready = {
                "ok": False,
                "reason": "drive_session_wait_error",
                "message": str(exc),
            }
        if not ready.get("ok"):
            if not handoff:
                self._park_drive(failure_reason)
            return ready
        return ready

    def _wait_point_drained(self, timeout: float) -> bool:
        deadline = self._monotonic() + max(0.0, float(timeout))
        while True:
            try:
                state = self._point_nav.get_state()
            except Exception:
                return False
            if state.get("drained"):
                return True
            remaining = deadline - self._monotonic()
            if remaining <= 0.0:
                return False
            try:
                self._point_nav.tick()
            except Exception:
                return False
            self._sleep(min(self._poll_interval, remaining))

    def _wait_tasks_drained(self, timeout: float) -> bool:
        waiter = getattr(self._tasks, "wait_drained", None)
        if not callable(waiter):
            return False
        try:
            return bool(waiter(max(0.0, float(timeout))))
        except Exception:
            return False

    def _emergency_stop(self) -> None:
        # 2026-07-17 治本(可观测): estop 触发点十几个, 之前全靠逆推猜.
        # 用 inspect 抓调用者函数:行号 print 到 stdout→go2w-web journal, 下次
        # EMERGENCY 直接 journalctl 看 caller, 不再猜.
        try:
            import inspect
            frm = inspect.stack()[1]
            print(f"[ARBITER] _emergency_stop from {frm.function}:{frm.lineno} "
                  f"(→ e_stop → estop_latched)", flush=True)
        except Exception:
            pass
        try:
            self._robot.e_stop()
        except Exception:
            pass

    def start_point_goal(self, x: float, y: float, yaw: float) -> dict:
        """Replace task/room ownership with one Panel point goal."""
        with self._lock:
            admission_epoch = self._read_emergency_epoch()
            if self._stopping:
                return {"ok": False, "reason": "shutting_down"}
            preflight_error = self._point_preflight(include_health=False)
            if preflight_error is not None:
                # Important ordering: a quarantined/stopped controller must not
                # destroy the currently valid room mission before submit fails.
                return {"ok": False, "reason": preflight_error}

            self._tasks.cancel_all()
            if not self._wait_tasks_drained(self._transition_timeout):
                self._emergency_stop()
                return {"ok": False, "reason": "tasks_not_drained"}
            if self._read_emergency_epoch() != admission_epoch:
                return {"ok": False, "reason": "emergency_interrupted"}

            # P1 (2026-07-16): 先确认 Nav2 action server 就绪 + ComputePath 成功,
            # 再激活轮式。原顺序激活轮式后才 submit, 若 Nav2 没 ready / 目标在 lethal
            # 区, 狗已进 wheel_balance 滑移(轮式平衡滚动)却无路径跟踪。预检失败则拒绝,
            # 狗保持 joint_lock。ComputePath 是 read-only, 不驱动底盘。
            wait_for_server = getattr(self._point_nav, "wait_for_server", None)
            if callable(wait_for_server) and not wait_for_server(timeout=2.0):
                return {"ok": False, "reason": "server_unavailable",
                        "message": "Nav2 action server 未就绪, 请稍后重试"}
            compute_path = getattr(self._point_nav, "compute_path_to_pose", None)
            if callable(compute_path):
                try:
                    path_result = compute_path(
                        x, y, yaw, frame_id="map", timeout=3.0) or {}
                except TypeError:
                    path_result = compute_path(x, y, yaw) or {}
                if not path_result.get("ok"):
                    return {"ok": False, "reason": "planner_failed",
                            "message": path_result.get(
                                "message", "目标不可达或在障碍区, 无法规划路径")}
            activation = self._activate_drive("nav", "point_activation_failed")
            if not activation.get("ok"):
                return activation
            self._motion_owner = "point"
            try:
                self._point_nav.tick()
            except Exception:
                self._motion_owner = None
                self._park_drive("point_health_refresh_failed")
                return {"ok": False, "reason": "point_nav_unavailable"}

            # Health/quarantine may change while the previous owner drains or
            # while the physical wheel mode is activating.
            preflight_error = self._point_preflight()
            if preflight_error is not None:
                self._motion_owner = None
                self._park_drive("point_preflight_failed")
                return {"ok": False, "reason": preflight_error}
            try:
                result = dict(self._point_nav.submit(x, y, yaw))
                if not result.get("ok"):
                    self._motion_owner = None
                    self._park_drive("point_submit_rejected")
                return result
            except ValueError as exc:
                self._motion_owner = None
                self._park_drive("point_invalid_goal")
                return {"ok": False, "reason": "invalid_goal", "message": str(exc)}
            except RuntimeError as exc:
                self._motion_owner = None
                self._park_drive("point_submit_rejected")
                return {"ok": False, "reason": "point_nav_rejected", "message": str(exc)}
            except Exception as exc:
                self._motion_owner = None
                self._park_drive("point_submit_error")
                return {"ok": False, "reason": "point_nav_error", "message": str(exc)}

    def start_tasks(self, tasks: Iterable[Any], *, reason: str) -> dict:
        """Replace PointNav/old task ownership, then enqueue new tasks."""
        task_list = list(tasks)
        with self._lock:
            admission_epoch = self._read_emergency_epoch()
            if self._stopping:
                return {"ok": False, "reason": "shutting_down"}
            try:
                self._point_nav.cancel(reason)
            except Exception:
                self._emergency_stop()
                return {"ok": False, "reason": "point_nav_cancel_error"}
            if not self._wait_point_drained(self._transition_timeout):
                self._emergency_stop()
                return {"ok": False, "reason": "point_nav_not_drained"}
            if self._read_emergency_epoch() != admission_epoch:
                return {"ok": False, "reason": "emergency_interrupted"}

            self._tasks.cancel_all()
            if not self._wait_tasks_drained(self._transition_timeout):
                self._emergency_stop()
                return {"ok": False, "reason": "tasks_not_drained"}
            if self._read_emergency_epoch() != admission_epoch:
                return {"ok": False, "reason": "emergency_interrupted"}

            activation = self._activate_drive("nav", "task_activation_failed")
            if not activation.get("ok"):
                return activation
            self._motion_owner = "tasks"
            enqueue = getattr(self._tasks, "_add_list_unchecked", None)
            if not callable(enqueue):
                self._motion_owner = None
                self._park_drive("task_enqueue_unavailable")
                return {"ok": False, "reason": "task_enqueue_unavailable"}
            try:
                enqueue(task_list)
            except Exception as exc:
                self._motion_owner = None
                self._park_drive("task_enqueue_error")
                return {"ok": False, "reason": "task_enqueue_error", "message": str(exc)}
            return {"ok": True, "count": len(task_list)}

    def on_point_state(self, state: dict) -> None:
        terminal = {
            "succeeded", "aborted", "rejected", "timed_out",
            "canceled", "error", "cancel_failed",
            # P3 (2026-07-16): 原列表漏这俩 → Nav2 server 没起来/规划失败时
            # point_nav 报 server_unavailable/planner_failed 但 arbiter 不判终态,
            # 狗留在 nav_active/wheel_balance 滑移不锁关节。补全后失败即停车。
            "server_unavailable", "planner_failed",
        }
        status = str((state or {}).get("status", ""))
        if status not in terminal or not bool((state or {}).get("drained")):
            return
        with self._lock:
            if self._motion_owner != "point":
                return
            self._motion_owner = None
            # Zero velocity cannot hold a Go2-W still in wheel-balance mode:
            # the firmware continues moving the wheels to balance and the dog
            # can slide after Nav2 reports success.  A drained terminal action
            # therefore owns exactly one feedback-gated StandUp parking
            # transition.  The owner clear above makes duplicate callbacks
            # idempotent.
            if not self._park_drive(f"nav_point_{status}"):
                self._emergency_stop()

    def on_tasks_drained(self) -> None:
        with self._lock:
            if self._motion_owner != "tasks":
                return
            self._motion_owner = None
            if not self._park_drive("nav_tasks_drained"):
                self._emergency_stop()

    def cancel_all_and_drain(self, reason: str) -> dict:
        """Stop both autonomous owners before an operator motion/pose command."""
        with self._lock:
            try:
                self._point_nav.cancel(reason)
            except Exception:
                pass
            self._tasks.cancel_all()
            point_drained = self._wait_point_drained(self._transition_timeout)
            tasks_drained = self._wait_tasks_drained(self._transition_timeout)
            ok = point_drained and tasks_drained
            if not ok:
                # 2026-07-17 治本(用户a): drain失败改 park 不 estop 锁存. 原 _emergency_stop
                # → estop_latched 锁死(需 restart 清)致反复 EMERGENCY (cancel_all_and_drain
                # 导航停止 + shutdown web restart 两处). park 失败才由 _park_drive :86 兜底 estop.
                self._park_drive("drain_timeout")
            return {
                "ok": ok,
                "point_drained": point_drained,
                "tasks_drained": tasks_drained,
                "reason": None if ok else "autonomy_not_drained",
            }

    def run_operator_action(self, reason: str, action: Callable[[], Any]) -> dict:
        """Drain autonomy, then execute stand/sit/manual/stop atomically."""
        with self._lock:
            admission_epoch = self._read_emergency_epoch()
            drained = self.cancel_all_and_drain(reason)
            if not drained["ok"]:
                return drained
            if self._read_emergency_epoch() != admission_epoch:
                return {"ok": False, "reason": "emergency_interrupted"}
            try:
                action()
            except Exception as exc:
                self._emergency_stop()
                return {"ok": False, "reason": "operator_action_error", "message": str(exc)}
            return {"ok": True}

    def run_manual_action(self, reason: str, action: Callable[[], Any]) -> dict:
        """Activate one manual session; held-key refreshes reuse ownership."""
        with self._lock:
            admission_epoch = self._read_emergency_epoch()
            if self._stopping:
                return {"ok": False, "reason": "shutting_down"}
            if self._motion_owner != "manual":
                drained = self.cancel_all_and_drain(reason)
                if not drained["ok"]:
                    return drained
                if self._read_emergency_epoch() != admission_epoch:
                    return {"ok": False, "reason": "emergency_interrupted"}
                activation = self._activate_drive("manual", "manual_activation_failed")
                if not activation.get("ok"):
                    return activation
                self._motion_owner = "manual"
            try:
                action()
            except Exception as exc:
                self._motion_owner = None
                self._park_drive("manual_action_error")
                return {
                    "ok": False,
                    "reason": "operator_action_error",
                    "message": str(exc),
                }
            return {"ok": True, "phase": "active", "owner": "manual"}

    def stop_manual(self, reason: str = "operator_stop") -> dict:
        """Zero immediately, cancel other owners when needed, then park once."""
        with self._lock:
            if self._motion_owner != "manual":
                drained = self.cancel_all_and_drain(reason)
                if not drained["ok"]:
                    return drained
            try:
                self._robot.stop_move()
            except Exception:
                self._emergency_stop()
                return {"ok": False, "reason": "manual_stop_error"}
            self._motion_owner = None
            if not self._park_drive(reason):
                return {"ok": False, "reason": "drive_park_error"}
            return {"ok": True, "phase": "parking"}

    def stop_all(self, reason: str = "operator_stop") -> dict:
        """Globally cancel navigation/tasks, publish zero, and park once."""
        with self._lock:
            drained = self.cancel_all_and_drain(reason)
            if not drained["ok"]:
                return drained
            try:
                self._robot.stop_move()
            except Exception:
                self._emergency_stop()
                return {"ok": False, "reason": "global_zero_error"}
            self._motion_owner = None
            if not self._park_drive(reason):
                return {"ok": False, "reason": "drive_park_error"}
            return {"ok": True, "phase": "parking"}

    def release_manual(self, reason: str = "manual_release") -> dict:
        """Zero manual velocity without changing the physical drive mode.

        Browser blur, key-up, and pointer-up belong to the manual controller.
        They must never be interpreted as requests to cancel Nav2 or a room
        search which may currently own the drive session.  They also must not
        park an owned wheel session: rapid key press/release cycles would then
        alternate BalanceStand and StandUp before either transition can settle.
        Explicit operator stop/stand remains the only manual path that parks.
        """
        with self._lock:
            if self._motion_owner != "manual":
                return {
                    "ok": True,
                    "phase": "unchanged",
                    "owner": self._motion_owner,
                    "ignored": True,
                }
            try:
                self._robot.stop_move()
            except Exception:
                self._emergency_stop()
                return {"ok": False, "reason": "manual_stop_error"}
            return {
                "ok": True,
                "phase": "idle",
                "owner": "manual",
                "reason": reason,
            }

    def emergency_stop(self, reason: str = "emergency_stop") -> dict:
        """E-stop first; cancellation acknowledgement is not on the critical path."""
        # This lock is intentionally independent of the admission mutex.  A
        # concurrent hand-off may be spending seconds waiting for old action
        # ownership; physical e-stop must not queue behind that wait.
        with self._emergency_lock:
            self._emergency_epoch += 1
        self._emergency_stop()
        with self._lock:
            try:
                self._point_nav.cancel(reason)
            except Exception:
                pass
            try:
                self._tasks.cancel_all()
            except Exception:
                pass
            return {"ok": True}

    def shutdown(self) -> dict:
        """Seal admissions and drain PointNav plus room/task ownership."""
        with self._lock:
            self._stopping = True
            try:
                self._point_nav.stop()
            except Exception:
                pass
            try:
                self._tasks.cancel_all()
            except Exception:
                pass
            try:
                self._robot.stop_move()
            except Exception:
                pass
            point_drained = self._wait_point_drained(self._transition_timeout)
            tasks_drained = self._wait_tasks_drained(self._transition_timeout)
            ok = point_drained and tasks_drained
            if not ok:
                # 2026-07-17 治本(用户a): drain失败改 park 不 estop 锁存. 原 _emergency_stop
                # → estop_latched 锁死(需 restart 清)致反复 EMERGENCY (cancel_all_and_drain
                # 导航停止 + shutdown web restart 两处). park 失败才由 _park_drive :86 兜底 estop.
                self._park_drive("drain_timeout")
            return {
                "ok": ok,
                "point_drained": point_drained,
                "tasks_drained": tasks_drained,
                "reason": None if ok else "shutdown_not_drained",
            }


__all__ = ["NavigationArbiter"]
