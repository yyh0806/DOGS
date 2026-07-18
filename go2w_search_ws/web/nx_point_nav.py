"""Thread-safe, serialized Panel point goals for Nav2 ``NavigateToPose``.

The controller deliberately does not own an executor.  Its asynchronous action
futures are progressed by the executor that already spins the supplied ROS
node.  ``tick()`` is the non-blocking watchdog hook for server readiness,
localization health, and active-goal timeout checks.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional


_STATUS_SUCCEEDED = 4
_STATUS_CANCELED = 5
_STATUS_ABORTED = 6


@dataclass
class _Request:
    generation: int
    x: float
    y: float
    yaw: float
    server_deadline: Optional[float] = None

    def as_goal(self) -> dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "frame_id": "map",
        }


@dataclass
class _Sending:
    request: _Request
    future: Any = None
    cancel_reason: Optional[str] = None
    terminal_target: Optional[_Request] = None
    callback_registration_failed: bool = False
    quarantine_polled: bool = False
    response_deadline: Optional[float] = None


@dataclass
class _Active:
    request: _Request
    handle: Any
    accepted_at: float
    result_future: Any = None
    cancel_future: Any = None
    cancel_reason: Optional[str] = None
    terminal_target: Optional[_Request] = None
    cancel_started: bool = False
    result_monitor_failed: bool = False
    cancel_callback_registration_failed: bool = False
    cancel_attempts: int = 0
    cancel_response_received: bool = False
    cancel_rejected: bool = False
    cancel_response_deadline: Optional[float] = None
    cancel_terminal_deadline: Optional[float] = None


@dataclass(frozen=True)
class _HealthReading:
    healthy: bool
    immediate: bool = False
    reason: Optional[str] = None


class PointNavigationController:
    """Serialize click-to-go requests over one asynchronous Nav2 action client.

    ``state_callback``, when supplied, receives an immutable-by-convention dict
    snapshot.  Integrators can wrap it as ``{"type": "nav_goal", "data": state}``
    for WebSocket delivery.

    The injected ``action_client`` must expose ``server_is_ready()`` and
    ``send_goal_async()``.  Injection keeps the module importable on development
    machines that do not have ROS installed.
    """

    def __init__(
        self,
        node: Any,
        state_callback: Optional[Callable[[dict[str, Any]], None]] = None,
        *,
        action_client: Any = None,
        action_type: Any = None,
        goal_factory: Optional[Callable[[], Any]] = None,
        clock: Any = None,
        action_name: str = "navigate_to_pose",
        monotonic: Callable[[], float] = time.monotonic,
        server_timeout: float = 3.0,
        active_timeout: Optional[float] = 180.0,
        send_response_timeout: float = 5.0,
        cancel_response_timeout: float = 3.0,
        cancel_terminal_timeout: float = 10.0,
        cancel_max_attempts: int = 3,
        health_failure_grace: float = 5.0,  # 2026-07-17 候选③: 0.5→5.0 给激活(parked→active, wheel_balance 转换)时间, 避免激活中 ready 短暂 false 立即 motion_unhealthy cancel 打断激活死循环
        health_check: Optional[Callable[[], Any]] = None,
    ) -> None:
        self._server_timeout = self._positive_timeout(server_timeout, "server_timeout")
        self._send_response_timeout = self._positive_timeout(
            send_response_timeout, "send_response_timeout"
        )
        self._cancel_response_timeout = self._positive_timeout(
            cancel_response_timeout, "cancel_response_timeout"
        )
        self._cancel_terminal_timeout = self._positive_timeout(
            cancel_terminal_timeout, "cancel_terminal_timeout"
        )
        self._cancel_max_attempts = self._positive_integer(
            cancel_max_attempts, "cancel_max_attempts"
        )
        self._health_failure_grace = self._nonnegative_timeout(
            health_failure_grace, "health_failure_grace"
        )
        if active_timeout is None:
            self._active_timeout = None
        else:
            self._active_timeout = self._positive_timeout(active_timeout, "active_timeout")

        if action_client is None or goal_factory is None:
            if action_type is None:
                try:
                    from nav2_msgs.action import NavigateToPose  # type: ignore
                except ImportError as exc:  # pragma: no cover - exercised on the robot
                    raise RuntimeError(
                        "nav2_msgs is unavailable; inject action_client and goal_factory"
                    ) from exc
                action_type = NavigateToPose
            if goal_factory is None:
                goal_factory = action_type.Goal
            if action_client is None:
                try:
                    from rclpy.action import ActionClient  # type: ignore
                except ImportError as exc:  # pragma: no cover - exercised on the robot
                    raise RuntimeError("rclpy is unavailable; inject action_client") from exc
                action_client = ActionClient(node, action_type, action_name)

        if clock is None:
            if node is None or not hasattr(node, "get_clock"):
                raise ValueError("clock is required when node does not provide get_clock()")
            clock = node.get_clock()

        self._client = action_client
        self._goal_factory = goal_factory
        self._clock = clock
        self._monotonic = monotonic
        self._health_check = health_check
        self._state_callback = state_callback

        self._lock = threading.RLock()
        self._generation = 0
        self._queued: Optional[_Request] = None
        self._sending: Optional[_Sending] = None
        self._active: Optional[_Active] = None
        self._stopped = False
        self._quarantined = False
        self._quarantine_reason: Optional[str] = None
        self._health_sample_next = 0
        self._health_sample_applied = 0
        initial_health = self._read_health()
        self._healthy = initial_health.healthy
        self._health_reason = (
            None if initial_health.healthy else initial_health.reason
        )
        self._health_failure_started_at = (
            None if self._healthy else self._monotonic()
        )
        self._pending_notification: Optional[dict[str, Any]] = None
        self._notifying = False
        self._state: dict[str, Any] = {
            "generation": 0,
            "status": "idle",
            "x": None,
            "y": None,
            "yaw": None,
            "frame_id": "map",
            "message": "idle",
            "reason": None,
            "updated_monotonic": self._monotonic(),
            **self._runtime_fields_locked(),
        }

    @staticmethod
    def _positive_timeout(value: Any, name: str) -> float:
        try:
            normalized = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite positive number") from exc
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(f"{name} must be a finite positive number")
        return normalized

    @staticmethod
    def _positive_integer(value: Any, name: str) -> int:
        if isinstance(value, bool):
            raise ValueError(f"{name} must be a positive integer")
        try:
            normalized = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a positive integer") from exc
        if not math.isfinite(normalized) or normalized <= 0.0 or not normalized.is_integer():
            raise ValueError(f"{name} must be a positive integer")
        return int(normalized)

    @staticmethod
    def _nonnegative_timeout(value: Any, name: str) -> float:
        try:
            normalized = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite non-negative number") from exc
        if not math.isfinite(normalized) or normalized < 0.0:
            raise ValueError(f"{name} must be a finite non-negative number")
        return normalized

    @staticmethod
    def _normalize_value(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise ValueError(f"{name} must be a finite number")
        try:
            normalized = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite number") from exc
        if not math.isfinite(normalized):
            raise ValueError(f"{name} must be a finite number")
        return normalized

    @staticmethod
    def _normalize_health_reading(sample: Any) -> _HealthReading:
        if isinstance(sample, dict):
            healthy = bool(sample.get("healthy"))
            reason_value = sample.get("reason")
            reason = None if healthy else str(reason_value or "health_unhealthy")
            return _HealthReading(
                healthy=healthy,
                immediate=bool(sample.get("immediate", False)) if not healthy else False,
                reason=reason,
            )
        healthy = bool(sample)
        return _HealthReading(
            healthy=healthy,
            immediate=False,
            reason=None if healthy else "localization_unhealthy",
        )

    def _read_health(self) -> _HealthReading:
        if self._health_check is None:
            return _HealthReading(healthy=True)
        try:
            return self._normalize_health_reading(self._health_check())
        except Exception:
            return _HealthReading(
                healthy=False,
                immediate=True,
                reason="health_check_error",
            )

    def _sample_health(self) -> tuple[int, _HealthReading]:
        with self._lock:
            self._health_sample_next += 1
            sequence = self._health_sample_next
        return sequence, self._read_health()

    def _apply_health_sample_locked(
        self, sequence: int, reading: _HealthReading
    ) -> Optional[str]:
        """Apply only the newest sample and return a newly latched failure reason.

        Structured immediate failures are hard safety gates.  Only recoverable
        samples (localization in production) receive the configured grace.
        """

        if sequence <= self._health_sample_applied:
            return None
        self._health_sample_applied = sequence
        if reading.healthy:
            self._health_failure_started_at = None
            self._healthy = True
            self._health_reason = None
            return None

        now = self._monotonic()
        self._health_reason = reading.reason or "health_unhealthy"
        if self._health_failure_started_at is None:
            self._health_failure_started_at = now
        if reading.immediate:
            if not self._healthy:
                return None
            self._healthy = False
            return self._health_reason
        if not self._healthy:
            return None
        elapsed = now - self._health_failure_started_at
        if (not math.isfinite(elapsed) or elapsed < 0.0
                or elapsed >= self._health_failure_grace):
            self._healthy = False
            return self._health_reason
        return None

    def _runtime_fields_locked(self) -> dict[str, Any]:
        sending = self._sending is not None
        active = self._active is not None
        cancel_pending = bool(active and self._active.cancel_reason is not None)
        deadlines = []
        if self._sending is not None and self._sending.response_deadline is not None:
            deadlines.append(self._sending.response_deadline)
        if self._active is not None:
            if self._active.cancel_response_deadline is not None:
                deadlines.append(self._active.cancel_response_deadline)
            if self._active.cancel_terminal_deadline is not None:
                deadlines.append(self._active.cancel_terminal_deadline)
        health_degraded = (
            self._healthy and self._health_failure_started_at is not None
        )
        health_failure_elapsed = (
            None if self._health_failure_started_at is None
            else max(0.0, self._monotonic() - self._health_failure_started_at)
        )
        return {
            "sending": sending,
            "active": active,
            "in_flight": sending or active,
            "drained": self._queued is None and not sending and not active,
            "cancel_pending": cancel_pending,
            "cancel_acknowledged": bool(
                active and self._active.cancel_response_received
            ),
            "cancel_attempts": self._active.cancel_attempts if active else 0,
            "deadline_monotonic": min(deadlines) if deadlines else None,
            "healthy": self._healthy,
            "health_degraded": health_degraded,
            "health_reason": self._health_reason,
            "health_failure_elapsed_sec": health_failure_elapsed,
            "health_failure_grace_sec": self._health_failure_grace,
            "quarantined": self._quarantined,
            "stopped": self._stopped,
        }

    @staticmethod
    def _target_dict(request: Optional[_Request]) -> dict[str, Any]:
        if request is None:
            return {"x": None, "y": None, "yaw": None, "frame_id": "map"}
        return request.as_goal()

    def _transition_locked(
        self,
        status: str,
        request: Optional[_Request],
        *,
        generation: Optional[int] = None,
        message: str,
        reason: Optional[str] = None,
    ) -> None:
        next_state = {
            "generation": self._generation if generation is None else generation,
            "status": status,
            **self._target_dict(request),
            "message": str(message),
            "reason": None if reason is None else str(reason),
            **self._runtime_fields_locked(),
        }
        comparable = {key: value for key, value in self._state.items() if key != "updated_monotonic"}
        if next_state == comparable:
            return
        next_state["updated_monotonic"] = self._monotonic()
        self._state = next_state
        if self._state_callback is not None:
            self._pending_notification = dict(next_state)

    def _flush_notifications(self) -> None:
        if self._state_callback is None:
            return
        # Future implementations may invoke a newly registered callback
        # synchronously.  In that path an outer submit/tick RLock can still be
        # held even though the callback's own ``with`` block has exited.
        is_owned = getattr(self._lock, "_is_owned", None)
        if callable(is_owned) and is_owned():
            return
        while True:
            with self._lock:
                if self._notifying or self._pending_notification is None:
                    return
                self._notifying = True
                notification = self._pending_notification
                self._pending_notification = None
            try:
                try:
                    self._state_callback(dict(notification))
                except Exception:
                    # A broken WebSocket consumer must not break action safety.
                    pass
            finally:
                with self._lock:
                    self._notifying = False

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            state = dict(self._state)
            state.update(self._runtime_fields_locked())
            return state

    def submit(self, x: Any, y: Any, yaw: Any = 0.0) -> dict[str, Any]:
        """Queue the newest finite map-frame goal and return its generation."""

        target_x = self._normalize_value(x, "x")
        target_y = self._normalize_value(y, "y")
        target_yaw = self._normalize_value(yaw, "yaw")
        health_sequence, health = self._sample_health()

        with self._lock:
            if self._stopped:
                raise RuntimeError("point-navigation controller is stopped")
            if self._quarantined:
                raise RuntimeError(
                    "point-navigation controller is quarantined after an uncertain action state"
                )
            health_failure = self._apply_health_sample_locked(
                health_sequence, health)
            if health_failure is not None:
                self._handle_health_loss_locked(health_failure)
            self._generation += 1
            request = _Request(self._generation, target_x, target_y, target_yaw)
            self._queued = request

            if self._sending is not None:
                self._sending.cancel_reason = "replaced"
                self._sending.terminal_target = request
            if self._active is not None:
                self._active.cancel_reason = "replaced"
                self._active.terminal_target = request

            self._transition_locked(
                "pending",
                request,
                message="navigation goal queued",
            )
            if self._active is not None:
                self._request_cancel_locked(self._active)
            self._drive_locked()
            response = {
                "ok": True,
                "generation": request.generation,
                "goal": request.as_goal(),
            }
        self._flush_notifications()
        return response

    def cancel(self, reason: str = "canceled") -> bool:
        """Cancel all queued/in-flight work; an empty controller is unchanged."""

        normalized_reason = str(reason or "canceled")
        with self._lock:
            canceled = self._cancel_locked(normalized_reason)
        if not canceled:
            return False
        self._flush_notifications()
        return True

    def _cancel_locked(self, reason: str) -> bool:
        if self._queued is None and self._sending is None and self._active is None:
            return False

        terminal_target = self._queued
        if terminal_target is None and self._sending is not None:
            terminal_target = self._sending.request
        if terminal_target is None and self._active is not None:
            terminal_target = self._active.request

        self._generation += 1
        self._queued = None
        if self._sending is not None:
            self._sending.cancel_reason = reason
            self._sending.terminal_target = terminal_target
        if self._active is not None:
            self._active.cancel_reason = reason
            self._active.terminal_target = terminal_target

        if self._sending is None and self._active is None:
            self._transition_locked(
                "canceled",
                terminal_target,
                message="navigation canceled",
                reason=reason,
            )
        else:
            self._transition_locked(
                "canceling",
                terminal_target,
                message="waiting for Nav2 cancellation to reach a terminal state",
                reason=reason,
            )
            if self._active is not None:
                self._request_cancel_locked(self._active)
        return True

    def stop(self) -> None:
        """Prevent new submissions and request cancellation of existing work."""

        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            self._cancel_locked("shutdown")
        self._flush_notifications()

    def tick(self) -> dict[str, Any]:
        """Run non-blocking health, timeout, and server-readiness watchdogs."""

        health_sequence, health = self._sample_health()
        now = self._monotonic()
        with self._lock:
            # Cancellation is a safety obligation, not normal controller work.
            # It must keep retrying after stop/quarantine, while all goal-send
            # and health-restart paths below remain gated off.
            if (
                self._active is not None
                and self._active.cancel_reason is not None
                and not self._active.cancel_started
            ):
                self._request_cancel_locked(self._active)

            self._poll_quarantined_sending_locked()
            self._poll_cancel_response_locked()
            self._check_watchdog_deadlines_locked(now)
            if not self._stopped and not self._quarantined:
                health_failure = self._apply_health_sample_locked(
                    health_sequence, health)
                if health_failure is not None:
                    self._handle_health_loss_locked(health_failure)

                if (
                    self._active is not None
                    and self._active_timeout is not None
                    and not self._active.result_monitor_failed
                    and self._active.cancel_reason is None
                    and now - self._active.accepted_at >= self._active_timeout
                ):
                    self._generation += 1
                    self._active.cancel_reason = "timeout"
                    self._active.terminal_target = self._active.request
                    self._transition_locked(
                        "canceling",
                        self._active.request,
                        message="active navigation exceeded its timeout",
                        reason="timeout",
                    )
                    self._request_cancel_locked(self._active)

                self._drive_locked()
            state = dict(self._state)
            state.update(self._runtime_fields_locked())
        self._flush_notifications()
        return state

    def _check_watchdog_deadlines_locked(self, now: float) -> None:
        sending = self._sending
        if (
            sending is not None
            and not self._quarantined
            and sending.response_deadline is not None
            and now >= sending.response_deadline
        ):
            self._enter_quarantine_locked(
                sending.request,
                message="timed out waiting for Nav2 goal acceptance response",
                reason="send_response_timeout",
                attempt_cancel=False,
            )

        active = self._active
        if active is None:
            return
        if (
            active.cancel_started
            and not active.cancel_response_received
            and active.cancel_response_deadline is not None
            and now >= active.cancel_response_deadline
        ):
            active.cancel_response_deadline = None
            self._enter_quarantine_locked(
                active.terminal_target or active.request,
                message="timed out waiting for Nav2 cancel response",
                reason="cancel_response_timeout",
                attempt_cancel=False,
            )
            return
        if (
            active.cancel_response_received
            and active.cancel_terminal_deadline is not None
            and now >= active.cancel_terminal_deadline
        ):
            active.cancel_terminal_deadline = None
            self._enter_quarantine_locked(
                active.terminal_target or active.request,
                message="timed out waiting for canceled goal to become terminal",
                reason="cancel_terminal_timeout",
                attempt_cancel=False,
            )

    @staticmethod
    def _health_message(reason: str, *, canceling: bool) -> str:
        if reason == "motion_unhealthy":
            return (
                "canceling because motion safety is unhealthy"
                if canceling else "waiting for motion safety readiness"
            )
        if reason == "health_check_error":
            return (
                "canceling because the navigation health check failed"
                if canceling else "waiting for navigation health checks"
            )
        return (
            "canceling because localization is unhealthy"
            if canceling else "waiting for healthy localization"
        )

    def _handle_health_loss_locked(self, reason: str) -> None:
        in_flight = self._sending is not None or self._active is not None
        if in_flight and self._queued is None:
            self._generation += 1

        terminal_target = self._queued
        if terminal_target is None and self._sending is not None:
            terminal_target = self._sending.request
        if terminal_target is None and self._active is not None:
            terminal_target = self._active.request

        if self._sending is not None:
            self._sending.cancel_reason = reason
            self._sending.terminal_target = terminal_target
        if self._active is not None:
            self._active.cancel_reason = reason
            self._active.terminal_target = terminal_target
            self._request_cancel_locked(self._active)

        if self._queued is not None:
            self._transition_locked(
                "waiting_health",
                self._queued,
                message=self._health_message(reason, canceling=False),
                reason=reason,
            )
        elif in_flight:
            self._transition_locked(
                "canceling",
                terminal_target,
                message=self._health_message(reason, canceling=True),
                reason=reason,
            )

    def _drive_locked(self) -> None:
        if (
            self._stopped
            or self._quarantined
            or self._active is not None
            or self._sending is not None
        ):
            return
        if self._queued is None:
            return

        request = self._queued
        if not self._healthy or self._health_failure_started_at is not None:
            reason = self._health_reason or "localization_unhealthy"
            self._transition_locked(
                "waiting_health",
                request,
                message=self._health_message(reason, canceling=False),
                reason=reason,
            )
            return

        now = self._monotonic()
        try:
            server_ready = bool(self._client.server_is_ready())
        except Exception:
            server_ready = False
        if not server_ready:
            if request.server_deadline is None:
                request.server_deadline = now + self._server_timeout
            if now >= request.server_deadline:
                self._queued = None
                self._transition_locked(
                    "server_unavailable",
                    request,
                    message="Nav2 action server did not become ready before the deadline",
                    reason="server_unavailable",
                )
            else:
                self._transition_locked(
                    "waiting_server",
                    request,
                    message="waiting for Nav2 action server",
                    reason=None,
                )
            return

        self._queued = None
        sending = _Sending(
            request=request,
            response_deadline=now + self._send_response_timeout,
        )
        self._sending = sending
        self._transition_locked(
            "pending",
            request,
            message="waiting for Nav2 goal acceptance",
            reason=None,
        )
        try:
            goal_message = self._build_goal_message(request)
        except Exception as exc:
            self._sending = None
            self._transition_locked(
                "error",
                request,
                message=f"failed to build navigation goal: {exc}",
                reason="goal_build_exception",
            )
            return

        try:
            sending.future = self._client.send_goal_async(goal_message)
        except Exception as exc:
            self._enter_quarantine_locked(
                request,
                message=f"navigation send call failed with uncertain ownership: {exc}",
                reason="send_exception",
            )
            return

        try:
            sending.future.add_done_callback(
                lambda future, token=sending: self._on_goal_response(token, future)
            )
        except Exception as exc:
            sending.callback_registration_failed = True
            self._enter_quarantine_locked(
                request,
                message=f"could not register navigation send callback: {exc}",
                reason="send_callback_registration_exception",
            )

    def _enter_quarantine_locked(
        self,
        request: _Request,
        *,
        message: str,
        reason: str,
        status: str = "error",
        attempt_cancel: bool = True,
    ) -> None:
        """Retain uncertain action ownership and permanently block new goals."""

        self._quarantined = True
        self._quarantine_reason = reason
        self._queued = None
        if self._sending is not None and self._sending.request is request:
            self._sending.cancel_reason = "quarantined"
            self._sending.terminal_target = request
        if self._active is not None and self._active.request is request:
            self._active.cancel_reason = "quarantined"
            self._active.terminal_target = request
        self._transition_locked(
            status,
            request,
            message=message,
            reason=reason,
        )
        if (
            attempt_cancel
            and self._active is not None
            and self._active.request is request
        ):
            self._request_cancel_locked(self._active)

    def _poll_quarantined_sending_locked(self) -> None:
        """Cancel a quarantined goal if its otherwise-unobservable handle appears."""

        sending = self._sending
        if (
            not self._quarantined
            or sending is None
            or not sending.callback_registration_failed
            or sending.quarantine_polled
            or sending.future is None
        ):
            return
        try:
            if not bool(sending.future.done()):
                return
        except Exception:
            return

        sending.quarantine_polled = True
        try:
            handle = sending.future.result()
        except Exception as exc:
            self._transition_locked(
                "error",
                sending.request,
                message=f"quarantined navigation send future failed: {exc}",
                reason=self._quarantine_reason or "send_exception",
            )
            return
        if not bool(getattr(handle, "accepted", False)):
            self._sending = None
            return

        self._sending = None
        active = _Active(
            request=sending.request,
            handle=handle,
            accepted_at=self._monotonic(),
            cancel_reason="quarantined",
            terminal_target=sending.request,
        )
        self._active = active
        try:
            active.result_future = handle.get_result_async()
        except Exception:
            active.result_monitor_failed = True
        else:
            try:
                active.result_future.add_done_callback(
                    lambda result_future, token=active: self._on_result(token, result_future)
                )
            except Exception:
                active.result_monitor_failed = True
        self._request_cancel_locked(active)

    def _build_goal_message(self, request: _Request) -> Any:
        message = self._goal_factory()
        message.pose.header.frame_id = "map"
        message.pose.header.stamp = self._clock.now().to_msg()
        message.pose.pose.position.x = request.x
        message.pose.pose.position.y = request.y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(request.yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(request.yaw * 0.5)
        return message

    def _on_goal_response(self, sending: _Sending, future: Any) -> None:
        try:
            self._handle_goal_response(sending, future)
        finally:
            self._flush_notifications()

    def _handle_goal_response(self, sending: _Sending, future: Any) -> None:
        with self._lock:
            if self._sending is not sending:
                return
            try:
                handle = future.result()
            except Exception as exc:
                sending.quarantine_polled = True
                self._enter_quarantine_locked(
                    sending.request,
                    message=f"navigation send future failed with uncertain ownership: {exc}",
                    reason="send_exception",
                )
                return

            if not bool(getattr(handle, "accepted", False)):
                self._sending = None
                if self._queued is not None:
                    self._drive_locked()
                elif sending.cancel_reason is not None:
                    target = sending.terminal_target or sending.request
                    self._transition_locked(
                        "canceled",
                        target,
                        message="navigation canceled before goal acceptance",
                        reason=sending.cancel_reason,
                    )
                else:
                    self._transition_locked(
                        "rejected",
                        sending.request,
                        generation=sending.request.generation,
                        message="Nav2 rejected the navigation goal",
                        reason="goal_rejected",
                    )
                return

            self._sending = None
            active = _Active(
                request=sending.request,
                handle=handle,
                accepted_at=self._monotonic(),
                cancel_reason=sending.cancel_reason,
                terminal_target=sending.terminal_target,
            )
            self._active = active
            try:
                active.result_future = handle.get_result_async()
            except Exception as exc:
                active.result_monitor_failed = True
                active.cancel_reason = active.cancel_reason or "result_exception"
                active.terminal_target = active.terminal_target or active.request
                self._enter_quarantine_locked(
                    active.terminal_target,
                    message=f"could not obtain accepted navigation result future: {exc}",
                    reason="result_exception",
                )
                self._request_cancel_locked(active)
                return

            try:
                active.result_future.add_done_callback(
                    lambda result_future, token=active: self._on_result(token, result_future)
                )
            except Exception as exc:
                active.result_monitor_failed = True
                active.cancel_reason = active.cancel_reason or "result_exception"
                active.terminal_target = active.terminal_target or active.request
                self._enter_quarantine_locked(
                    active.terminal_target,
                    message=f"could not register accepted-goal result callback: {exc}",
                    reason="result_callback_registration_exception",
                )
                self._request_cancel_locked(active)
                return

            # A Future is allowed to invoke its callback immediately when it
            # was already complete.  In that case _on_result() has already
            # cleared this token and published the terminal state.
            if self._active is not active:
                return

            if active.cancel_reason is not None or self._queued is not None or not self._healthy:
                if active.cancel_reason is None:
                    active.cancel_reason = (
                        "localization_unhealthy" if not self._healthy else "replaced"
                    )
                if active.terminal_target is None:
                    active.terminal_target = self._queued or active.request
                self._request_cancel_locked(active)
            else:
                self._transition_locked(
                    "active",
                    active.request,
                    generation=active.request.generation,
                    message="navigation goal accepted",
                    reason=None,
                )

    def _request_cancel_locked(self, active: _Active) -> None:
        if active.cancel_started or active.cancel_response_received:
            return
        if active.cancel_attempts >= self._cancel_max_attempts:
            self._enter_quarantine_locked(
                active.terminal_target or active.request,
                message="Nav2 cancellation retry limit was exhausted",
                reason="cancel_retry_exhausted",
                status="cancel_failed",
                attempt_cancel=False,
            )
            return

        active.cancel_started = True
        active.cancel_attempts += 1
        active.cancel_callback_registration_failed = False
        try:
            active.cancel_future = active.handle.cancel_goal_async()
        except Exception as exc:
            active.cancel_started = False
            active.cancel_future = None
            active.cancel_response_deadline = None
            target = active.terminal_target or active.request
            self._transition_locked(
                "cancel_failed",
                target,
                message=f"Nav2 cancel request failed and will be retried: {exc}",
                reason="cancel_exception",
            )
            if active.cancel_attempts >= self._cancel_max_attempts:
                self._enter_quarantine_locked(
                    target,
                    message="Nav2 cancellation retry limit was exhausted",
                    reason="cancel_retry_exhausted",
                    status="cancel_failed",
                    attempt_cancel=False,
                )
            return

        active.cancel_response_deadline = self._monotonic() + self._cancel_response_timeout
        try:
            active.cancel_future.add_done_callback(
                lambda future, token=active: self._on_cancel_response(token, future)
            )
        except Exception as exc:
            active.cancel_callback_registration_failed = True
            target = active.terminal_target or active.request
            self._transition_locked(
                "canceling",
                target,
                message=f"Nav2 cancel was sent but its response callback could not register: {exc}",
                reason="cancel_callback_registration_exception",
            )

    def _on_cancel_response(self, active: _Active, future: Any) -> None:
        with self._lock:
            self._consume_cancel_response_locked(active, future)
        self._flush_notifications()

    def _consume_cancel_response_locked(self, active: _Active, future: Any) -> None:
        if (
            self._active is not active
            or active.cancel_future is not future
            or active.cancel_response_received
        ):
            return
        target = active.terminal_target or active.request
        try:
            response = future.result()
        except Exception as exc:
            active.cancel_started = False
            active.cancel_future = None
            active.cancel_response_deadline = None
            active.cancel_callback_registration_failed = False
            self._transition_locked(
                "cancel_failed",
                target,
                message=f"Nav2 cancel response failed and will be retried: {exc}",
                reason="cancel_exception",
            )
            if active.cancel_attempts >= self._cancel_max_attempts:
                self._enter_quarantine_locked(
                    target,
                    message="Nav2 cancellation retry limit was exhausted",
                    reason="cancel_retry_exhausted",
                    status="cancel_failed",
                    attempt_cancel=False,
                )
            return

        try:
            return_code = int(response.return_code)
            goals_canceling = list(response.goals_canceling)
        except Exception:
            return_code = -1
            goals_canceling = []
        if return_code != 0 or not goals_canceling:
            active.cancel_rejected = True
            active.cancel_started = False
            active.cancel_future = None
            active.cancel_response_deadline = None
            active.cancel_callback_registration_failed = False
            self._transition_locked(
                "cancel_failed",
                target,
                message=(
                    "Nav2 rejected cancellation"
                    if return_code != 0
                    else "Nav2 accepted no goal for cancellation"
                ),
                reason="cancel_rejected",
            )
            if active.cancel_attempts >= self._cancel_max_attempts:
                self._enter_quarantine_locked(
                    target,
                    message="Nav2 cancellation retry limit was exhausted after rejection",
                    reason="cancel_retry_exhausted",
                    status="cancel_failed",
                    attempt_cancel=False,
                )
            return

        active.cancel_response_received = True
        active.cancel_response_deadline = None
        active.cancel_callback_registration_failed = False
        active.cancel_terminal_deadline = self._monotonic() + self._cancel_terminal_timeout
        self._transition_locked(
            "canceling",
            target,
            message="Nav2 accepted cancellation; waiting for terminal result",
            reason=active.cancel_reason or "canceled",
        )

    def _poll_cancel_response_locked(self) -> None:
        active = self._active
        if (
            active is None
            or not active.cancel_callback_registration_failed
            or active.cancel_future is None
            or active.cancel_response_received
        ):
            return
        try:
            if not bool(active.cancel_future.done()):
                return
        except Exception:
            return
        self._consume_cancel_response_locked(active, active.cancel_future)

    def _on_result(self, active: _Active, future: Any) -> None:
        try:
            self._handle_result(active, future)
        finally:
            self._flush_notifications()

    def _handle_result(self, active: _Active, future: Any) -> None:
        with self._lock:
            if self._active is not active:
                return
            try:
                response = future.result()
                status = int(response.status)
            except Exception as exc:
                active.result_monitor_failed = True
                active.cancel_reason = active.cancel_reason or "result_exception"
                active.terminal_target = active.terminal_target or active.request
                self._enter_quarantine_locked(
                    active.terminal_target,
                    message=f"navigation result future failed with uncertain ownership: {exc}",
                    reason="result_exception",
                )
                self._request_cancel_locked(active)
                return

            self._active = None
            if self._quarantined:
                if active.cancel_rejected and status == _STATUS_SUCCEEDED:
                    self._transition_locked(
                        "cancel_failed",
                        active.terminal_target or active.request,
                        message="goal succeeded after Nav2 rejected cancellation",
                        reason="cancel_rejected_goal_succeeded",
                    )
                else:
                    self._transition_locked(
                        "error",
                        active.terminal_target or active.request,
                        message="controller remains quarantined after the goal reached a terminal state",
                        reason=self._quarantine_reason or "quarantined",
                    )
            elif self._queued is not None:
                # The old result only releases serialization; it never owns the
                # state of the newer generation.
                self._drive_locked()
            elif status == _STATUS_SUCCEEDED:
                self._transition_locked(
                    "succeeded",
                    active.terminal_target or active.request,
                    message=(
                        "navigation succeeded before cancellation took effect"
                        if active.cancel_reason is not None
                        else "navigation succeeded"
                    ),
                    reason=(
                        "cancel_not_effective"
                        if active.cancel_reason is not None
                        else None
                    ),
                )
            elif status == _STATUS_CANCELED:
                target = active.terminal_target or active.request
                if active.cancel_reason == "timeout":
                    self._transition_locked(
                        "timed_out",
                        target,
                        message="navigation timed out and reached a terminal state",
                        reason="timeout",
                    )
                else:
                    self._transition_locked(
                        "canceled",
                        target,
                        message="navigation canceled",
                        reason=active.cancel_reason or "nav2_canceled",
                    )
            elif status == _STATUS_ABORTED:
                self._transition_locked(
                    "aborted",
                    active.request,
                    generation=active.request.generation,
                    message="Nav2 aborted the navigation goal",
                    reason="nav2_aborted",
                )
            else:
                self._transition_locked(
                    "error",
                    active.request,
                    generation=active.request.generation,
                    message=f"Nav2 returned unknown goal status {status}",
                    reason=f"nav2_status_{status}",
                )
            self._drive_locked()


__all__ = ["PointNavigationController"]
