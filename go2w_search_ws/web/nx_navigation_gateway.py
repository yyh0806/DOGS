"""Single process-wide owner for every Nav2 NavigateToPose goal."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import threading
import time
from typing import Any, Callable, Optional


TERMINAL_STATUSES = frozenset({
    "succeeded", "aborted", "rejected", "timed_out", "canceled",
    "cancel_failed", "server_unavailable", "error",
})


@dataclass(frozen=True)
class SubmitResult:
    accepted: bool
    reason: Optional[str]
    generation: Optional[int]
    owner: Optional[str]


@dataclass(frozen=True)
class CancelResult:
    accepted: bool
    reason: str


@dataclass(frozen=True)
class NavigationResult:
    ok: bool
    status: str
    reason: Optional[str] = None


@dataclass(frozen=True)
class PathResult:
    ok: bool
    reason: Optional[str] = None
    path_length: Optional[float] = None
    poses: int = 0
    endpoint_x: Optional[float] = None
    endpoint_y: Optional[float] = None
    goal_error_m: Optional[float] = None
    path: Optional[tuple] = None


class NavigationGateway:
    """Serialize logical owners over one proven asynchronous action port.

    The action port is normally ``PointNavigationController``.  It retains
    late-acceptance and cancel quarantine; this gateway adds cross-producer
    ownership so point navigation and missions cannot construct or race
    independent NavigateToPose clients.
    """

    def __init__(
        self,
        *,
        action_port: Any,
        path_port: Any = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        poll_interval: float = 0.02,
    ) -> None:
        self._action = action_port
        self._path = path_port
        self._monotonic = monotonic
        self._sleep = sleep
        self._poll_interval = max(0.001, float(poll_interval))
        self._lock = threading.RLock()
        self._owner: Optional[str] = None
        self._generation = 0
        self._port_generation: Optional[int] = None
        self._terminal: dict[int, NavigationResult] = {}
        self._stopped = False

    @staticmethod
    def _owner_name(owner: object) -> str:
        value = str(owner or "").strip().lower()
        if not value or len(value) > 64:
            raise ValueError("invalid navigation owner")
        return value

    @staticmethod
    def _pose(value: object) -> tuple[float, float, float]:
        try:
            pose = tuple(float(item) for item in value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("pose must contain finite x, y, yaw") from exc
        if len(pose) != 3 or not all(math.isfinite(item) for item in pose):
            raise ValueError("pose must contain finite x, y, yaw")
        return pose

    def submit(self, *, owner: object, pose: object, feedback_cb=None) -> SubmitResult:
        del feedback_cb  # State callbacks remain owned by the single action port.
        owner_name = self._owner_name(owner)
        x, y, yaw = self._pose(pose)
        with self._lock:
            if self._stopped:
                return SubmitResult(False, "navigation_gateway_stopped", None, None)
            if self._owner is not None and self._owner != owner_name:
                return SubmitResult(
                    False, "navigation_owner_busy", self._generation, self._owner)
            try:
                result = dict(self._action.submit(x, y, yaw))
            except Exception as exc:
                return SubmitResult(False, str(exc) or "navigation_submit_error", None, None)
            if not result.get("ok"):
                return SubmitResult(
                    False, str(result.get("reason") or "navigation_rejected"),
                    None, None)
            self._generation += 1
            self._owner = owner_name
            try:
                self._port_generation = int(result.get("generation"))
            except (TypeError, ValueError, OverflowError):
                self._port_generation = None
            return SubmitResult(True, None, self._generation, owner_name)

    def cancel(self, *, owner: object, reason: object) -> CancelResult:
        owner_name = self._owner_name(owner)
        with self._lock:
            if self._owner is None:
                return CancelResult(True, "already_drained")
            if self._owner != owner_name:
                return CancelResult(False, "navigation_owner_mismatch")
            try:
                accepted = bool(self._action.cancel(str(reason or "canceled")))
            except Exception:
                return CancelResult(False, "navigation_cancel_error")
            state = self._port_state()
            if state.get("drained"):
                self._finish_locked(str(state.get("status") or "canceled"))
            return CancelResult(accepted or bool(state.get("drained")), "cancel_requested")

    def tick(self) -> dict:
        try:
            state = self._action.tick()
        except Exception:
            state = self._port_state()
        if not isinstance(state, dict):
            state = self._port_state()
        with self._lock:
            status = str(state.get("status", "unknown"))
            port_generation = state.get("generation")
            generation_matches = (
                self._port_generation is None
                or port_generation is None
                or int(port_generation) == self._port_generation)
            if (self._owner is not None and generation_matches
                    and bool(state.get("drained"))
                    and status in TERMINAL_STATUSES):
                self._finish_locked(status, state.get("reason"))
            return self.snapshot()

    def observe_terminal(self, *, generation: int, status: str, reason=None) -> None:
        with self._lock:
            if int(generation) != self._generation:
                return
            if str(status) in TERMINAL_STATUSES:
                self._finish_locked(str(status), reason)

    def wait_terminal(
        self,
        *,
        owner: object,
        timeout: float,
        recovery_callback: Optional[Callable[[str], Any]] = None,
        recovery_interval: float = 0.5,
    ) -> NavigationResult:
        owner_name = self._owner_name(owner)
        deadline = self._monotonic() + max(0.0, float(timeout))
        recovery_interval = max(0.0, float(recovery_interval))
        last_recovery_at = float("-inf")
        with self._lock:
            generation = self._generation
            if self._owner != owner_name:
                return NavigationResult(False, "rejected", "navigation_owner_mismatch")
        while True:
            state = self.tick()
            with self._lock:
                terminal = self._terminal.get(generation)
                if terminal is not None:
                    return terminal
            now = self._monotonic()
            recoverable_motion_health = (
                (
                    str(state.get("status")) == "waiting_health"
                    and str(state.get("reason")) == "motion_unhealthy"
                )
                or (
                    bool(state.get("health_degraded"))
                    and str(state.get("health_reason")) == "motion_unhealthy"
                )
            )
            if (
                recovery_callback is not None
                and recoverable_motion_health
                and now - last_recovery_at >= recovery_interval
            ):
                last_recovery_at = now
                try:
                    recovered = dict(recovery_callback("motion_unhealthy") or {})
                except Exception as exc:
                    recovered = {
                        "ok": False,
                        "reason": "motion_recovery_error",
                        "message": str(exc),
                    }
                if not recovered.get("ok"):
                    self.cancel(owner=owner_name, reason="motion_recovery_failed")
                    return NavigationResult(
                        False,
                        "aborted",
                        str(recovered.get("reason") or "motion_recovery_failed"),
                    )
                continue
            remaining = deadline - self._monotonic()
            if remaining <= 0.0:
                self.cancel(owner=owner_name, reason="navigation_timeout")
                return NavigationResult(False, "timed_out", "navigation_timeout")
            self._sleep(min(self._poll_interval, remaining))

    def wait_drained(self, *, owner: object, timeout: float) -> bool:
        """Wait until ``owner`` no longer holds the shared action port.

        Ownership is scoped: a mission is already drained when a point goal
        owns the gateway.  Treating another owner as an in-flight mission
        would make TaskManager cancellation time out and spuriously trigger
        the emergency-stop path before every Panel point goal.
        """
        owner_name = self._owner_name(owner)
        deadline = self._monotonic() + max(0.0, float(timeout))
        while True:
            self.tick()
            with self._lock:
                if self._owner != owner_name:
                    return True
            remaining = deadline - self._monotonic()
            if remaining <= 0.0:
                return False
            self._sleep(min(self._poll_interval, remaining))

    def compute_path(self, pose: object, timeout: float) -> PathResult:
        normalized = self._pose(pose)
        if self._path is None:
            return PathResult(False, "planner_unavailable")
        try:
            result = dict(self._path.compute_path(normalized, float(timeout)))
        except Exception:
            return PathResult(False, "planner_error")
        return PathResult(
            ok=bool(result.get("ok")),
            reason=(None if result.get("ok") else str(
                result.get("reason") or "unreachable")),
            path_length=(float(result["path_length"])
                         if result.get("path_length") is not None else None),
            poses=int(result.get("poses", 0)),
            endpoint_x=(float(result["endpoint_x"])
                        if result.get("endpoint_x") is not None else None),
            endpoint_y=(float(result["endpoint_y"])
                        if result.get("endpoint_y") is not None else None),
            goal_error_m=(float(result["goal_error_m"])
                          if result.get("goal_error_m") is not None else None),
            path=(tuple(dict(point) for point in result.get("path", ()))
                  if result.get("path") else None),
        )

    def snapshot(self) -> dict:
        with self._lock:
            state = self._port_state()
            return {
                **state,
                "owner": self._owner,
                "generation": self._generation,
                "stopped": self._stopped,
                "drained": self._owner is None and bool(state.get("drained", True)),
            }

    def shutdown(self) -> None:
        with self._lock:
            self._stopped = True
            try:
                self._action.stop()
            except Exception:
                pass
            state = self._port_state()
            if self._owner is not None and bool(state.get("drained")):
                self._finish_locked(str(state.get("status") or "canceled"))

    def _finish_locked(self, status: str, reason=None) -> None:
        result = NavigationResult(
            ok=status == "succeeded",
            status=status,
            reason=(None if status == "succeeded" else str(reason or status)),
        )
        self._terminal[self._generation] = result
        self._owner = None
        self._port_generation = None

    def _port_state(self) -> dict:
        try:
            state = self._action.get_state()
        except Exception:
            return {"status": "error", "drained": False, "healthy": False}
        return dict(state or {})


class OwnerNavigationPort:
    """Compatibility facade exposing one gateway owner as a point controller."""

    def __init__(self, gateway: NavigationGateway, owner: str) -> None:
        self._gateway = gateway
        self._owner = str(owner)

    def submit(self, x, y, yaw=0.0) -> dict:
        result = self._gateway.submit(
            owner=self._owner, pose=(x, y, yaw))
        return {
            "ok": result.accepted,
            "reason": result.reason,
            "generation": result.generation,
            "goal": {"x": float(x), "y": float(y), "yaw": float(yaw),
                     "frame_id": "map"},
        }

    def cancel(self, reason="canceled") -> bool:
        return self._gateway.cancel(
            owner=self._owner, reason=reason).accepted

    def tick(self) -> dict:
        return self._gateway.tick()

    def get_state(self) -> dict:
        return self._gateway.snapshot()

    def stop(self) -> None:
        self._gateway.shutdown()


class MissionNavigationPort:
    """Blocking room/exploration facade over the same gateway owner."""

    def __init__(
        self,
        gateway: NavigationGateway,
        owner: str = "mission",
        *,
        recovery_callback: Optional[Callable[[str], Any]] = None,
        recovery_interval: float = 0.5,
    ) -> None:
        self._gateway = gateway
        self._owner = str(owner)
        self._feedback_callback = None
        self._recovery_callback = recovery_callback
        self._recovery_interval = max(0.0, float(recovery_interval))
        self._admission_lock = threading.RLock()
        self._accepting_goals = True

    def begin_mission(self) -> None:
        """Open a fresh admission epoch after the previous mission drained."""

        with self._admission_lock:
            self._accepting_goals = True

    def set_feedback_callback(self, callback) -> None:
        self._feedback_callback = callback

    def set_recovery_callback(self, callback) -> None:
        self._recovery_callback = callback

    def wait_for_server(self, timeout=2.0) -> bool:
        del timeout
        state = self._gateway.snapshot()
        return not state.get("stopped") and state.get("healthy", True) is not False

    def wait_for_planner(self, timeout=2.0) -> bool:
        del timeout
        return self._gateway._path is not None

    def send_goal_and_wait(self, x, y, yaw, frame_id="map") -> dict:
        if frame_id != "map":
            return {"ok": False, "reason": "invalid_frame"}
        # Serialize the mission-cancel fence with gateway submission. If the
        # submit wins, cancel_current() observes and cancels the new owner. If
        # cancellation wins, no stale worker may submit after it returns.
        with self._admission_lock:
            if not self._accepting_goals:
                return {"ok": False, "reason": "cancelled"}
            submitted = self._gateway.submit(
                owner=self._owner,
                pose=(x, y, yaw),
                feedback_cb=self._feedback_callback,
            )
        if not submitted.accepted:
            return {"ok": False, "reason": submitted.reason}
        # 36m+ frontier 在 0.5m/s 下仅直线移动就需要 72s，因此默认给 90s；
        # 现场可通过 env 缩短/延长，超时仍会 cancel 并跳过该 frontier。
        result = self._gateway.wait_terminal(
            owner=self._owner,
            timeout=float(os.environ.get("GO2W_FRONTIER_NAV_TIMEOUT", "90.0")),
            recovery_callback=self._recovery_callback,
            recovery_interval=self._recovery_interval,
        )
        if result.ok:
            return {"ok": True, "status": 4}
        reasons = {
            "canceled": "cancelled",
            "timed_out": "timeout",
            "aborted": "aborted",
            "rejected": "rejected",
        }
        return {
            "ok": False,
            "reason": reasons.get(result.status, result.reason or result.status),
        }

    def compute_path_to_pose(self, x, y, yaw, frame_id="map", timeout=None):
        if frame_id != "map":
            return {"ok": False, "reason": "invalid_frame"}
        result = self._gateway.compute_path(
            (x, y, yaw), timeout=3.0 if timeout is None else timeout)
        value = {"ok": result.ok}
        if result.reason is not None:
            value["reason"] = result.reason
        if result.path_length is not None:
            value["path_length"] = result.path_length
        if result.poses:
            value["poses"] = result.poses
        if result.endpoint_x is not None:
            value["endpoint_x"] = result.endpoint_x
        if result.endpoint_y is not None:
            value["endpoint_y"] = result.endpoint_y
        if result.goal_error_m is not None:
            value["goal_error_m"] = result.goal_error_m
        if result.path is not None:
            value["path"] = [dict(point) for point in result.path]
        return value

    def cancel_current(self, reason="mission_cancel") -> bool:
        with self._admission_lock:
            if str(reason or "") == "mission_cancel":
                self._accepting_goals = False
            return self._gateway.cancel(
                owner=self._owner, reason=reason).accepted

    def wait_drained(self, timeout: float) -> bool:
        return self._gateway.wait_drained(
            owner=self._owner, timeout=timeout)

    def get_state(self) -> dict:
        return self._gateway.snapshot()


class RosComputePathPort:
    """Read-only ComputePathToPose action adapter driven by the shared executor."""

    def __init__(self, node, action_name="/compute_path_to_pose") -> None:
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._node = node
        self._action_type = ComputePathToPose
        self._client = ActionClient(
            node, ComputePathToPose, action_name,
            callback_group=ReentrantCallbackGroup())

    @staticmethod
    def _wait(future, timeout):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        try:
            if future.done():
                return True
        except Exception:
            pass
        return event.wait(max(0.0, float(timeout)))

    def compute_path(self, pose, timeout):
        if not self._client.wait_for_server(timeout_sec=min(2.0, float(timeout))):
            return {"ok": False, "reason": "no_planner"}
        x, y, yaw = pose
        goal = self._action_type.Goal()
        goal.goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(yaw / 2.0)
        goal.use_start = False
        goal.planner_id = ""
        accepted_future = self._client.send_goal_async(goal)
        if not self._wait(accepted_future, timeout):
            return {"ok": False, "reason": "plan_timeout"}
        handle = accepted_future.result()
        if handle is None or not getattr(handle, "accepted", False):
            return {"ok": False, "reason": "plan_rejected"}
        result_future = handle.get_result_async()
        if not self._wait(result_future, timeout):
            handle.cancel_goal_async()
            return {"ok": False, "reason": "plan_timeout"}
        wrapped = result_future.result()
        poses = list(getattr(getattr(wrapped.result, "path", None), "poses", []) or [])
        if int(getattr(wrapped, "status", -1)) != 4 or not poses:
            return {"ok": False, "reason": "unreachable"}
        length = 0.0
        for previous, current in zip(poses, poses[1:]):
            length += math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y)
        path = [
            {"x": float(item.pose.position.x),
             "y": float(item.pose.position.y)}
            for item in poses
        ]
        endpoint_x = path[-1]["x"]
        endpoint_y = path[-1]["y"]
        return {
            "ok": True,
            "path_length": length,
            "poses": len(poses),
            "endpoint_x": endpoint_x,
            "endpoint_y": endpoint_y,
            "goal_error_m": math.hypot(endpoint_x - x, endpoint_y - y),
            "path": path,
        }


__all__ = [
    "CancelResult", "MissionNavigationPort", "NavigationGateway",
    "NavigationResult", "OwnerNavigationPort", "PathResult",
    "RosComputePathPort", "SubmitResult",
]
