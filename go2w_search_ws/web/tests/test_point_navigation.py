"""Behavior tests for the serialized Panel -> Nav2 point-goal controller."""

from __future__ import annotations

import ast
import math
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

import nx_point_nav as point_nav_module  # noqa: E402
from nx_point_nav import PointNavigationController  # noqa: E402


SUCCEEDED = 4
CANCELED = 5
ABORTED = 6


class FakeCancelResponse:
    ERROR_NONE = 0
    ERROR_REJECTED = 1

    def __init__(self, return_code=ERROR_NONE, goals_canceling=None):
        self.return_code = return_code
        self.goals_canceling = [SimpleNamespace()] if goals_canceling is None else goals_canceling


class Deferred:
    """Small Future double whose callbacks are controlled by the test."""

    def __init__(self):
        self._callbacks = []
        self._value = None
        self._exception = None
        self._done = False

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._done:
            callback(self)

    def result(self):
        if not self._done:
            raise AssertionError("future is not complete")
        if self._exception is not None:
            raise self._exception
        return self._value

    def done(self):
        return self._done

    def resolve(self, value):
        self._value = value
        self._exception = None
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def fail(self, exception):
        self._exception = exception
        self._done = True
        for callback in list(self._callbacks):
            callback(self)


class CallbackRegistrationFailureFuture(Deferred):
    def add_done_callback(self, callback):
        raise RuntimeError("callback registration failed")


class RegisterThenRaiseFuture(Deferred):
    def add_done_callback(self, callback):
        super().add_done_callback(callback)
        raise RuntimeError("callback registration failed after registration")


class FakeGoal:
    def __init__(self):
        self.pose = SimpleNamespace(
            header=SimpleNamespace(frame_id=None, stamp=None),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )


class FakeClock:
    class _Now:
        def __init__(self, stamp):
            self._stamp = stamp

        def to_msg(self):
            return self._stamp

    def __init__(self, stamp="clock-stamp"):
        self.stamp = stamp

    def now(self):
        return self._Now(self.stamp)


class FakeMonotonic:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class FakeGoalHandle:
    def __init__(self, *, accepted=True):
        self.accepted = accepted
        self.result_future = Deferred()
        self.cancel_future = None
        self.cancel_futures = []
        self.cancel_future_factory = Deferred
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        self.cancel_future = self.cancel_future_factory()
        self.cancel_futures.append(self.cancel_future)
        return self.cancel_future


class TransientCancelFailureGoalHandle(FakeGoalHandle):
    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.cancel_calls == 1:
            raise RuntimeError("transient cancel transport failure")
        self.cancel_future = self.cancel_future_factory()
        self.cancel_futures.append(self.cancel_future)
        return self.cancel_future


class FakeActionClient:
    def __init__(self, *, ready=True):
        self.ready = ready
        self.goals = []
        self.send_futures = []
        self.future_factory = Deferred

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        future = self.future_factory()
        self.goals.append(goal)
        self.send_futures.append(future)
        return future


def make_controller(
    *,
    ready=True,
    server_timeout=2.0,
    active_timeout=30.0,
    send_response_timeout=2.0,
    cancel_response_timeout=2.0,
    cancel_terminal_timeout=4.0,
    cancel_max_attempts=3,
    health_failure_grace=0.5,
    health_check=None,
    state_callback=None,
):
    action_client = FakeActionClient(ready=ready)
    monotonic = FakeMonotonic()
    controller = PointNavigationController(
        None,
        state_callback=state_callback,
        action_client=action_client,
        goal_factory=FakeGoal,
        clock=FakeClock(),
        monotonic=monotonic,
        server_timeout=server_timeout,
        active_timeout=active_timeout,
        send_response_timeout=send_response_timeout,
        cancel_response_timeout=cancel_response_timeout,
        cancel_terminal_timeout=cancel_terminal_timeout,
        cancel_max_attempts=cancel_max_attempts,
        health_failure_grace=health_failure_grace,
        health_check=health_check,
    )
    return controller, action_client, monotonic


def accept(client, index=0, *, accepted=True):
    handle = FakeGoalHandle(accepted=accepted)
    client.send_futures[index].resolve(handle)
    return handle


def finish(handle, status):
    handle.result_future.resolve(SimpleNamespace(status=status, result=SimpleNamespace()))


def accept_cancel(handle, index=-1):
    handle.cancel_futures[index].resolve(FakeCancelResponse())


def reject_cancel(handle, *, return_code=FakeCancelResponse.ERROR_REJECTED, empty=False, index=-1):
    goals = [] if empty else [SimpleNamespace()]
    handle.cancel_futures[index].resolve(FakeCancelResponse(return_code, goals))


def target(state):
    return state["x"], state["y"], state["yaw"]


def test_success_builds_exact_map_pose_and_emits_state_snapshots():
    events = []
    ctl, client, monotonic = make_controller(state_callback=events.append)

    response = ctl.submit(2, -1, math.pi / 2)

    assert response == {
        "ok": True,
        "generation": 1,
        "goal": {"x": 2.0, "y": -1.0, "yaw": math.pi / 2, "frame_id": "map"},
    }
    assert len(client.goals) == 1
    message = client.goals[0]
    assert message.pose.header.frame_id == "map"
    assert message.pose.header.stamp == "clock-stamp"
    assert message.pose.pose.position.x == 2.0
    assert message.pose.pose.position.y == -1.0
    assert message.pose.pose.position.z == 0.0
    assert message.pose.pose.orientation.x == 0.0
    assert message.pose.pose.orientation.y == 0.0
    assert message.pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert message.pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))
    assert events[-1]["status"] == "pending"
    assert events[-1]["updated_monotonic"] == monotonic.value

    handle = accept(client)
    assert ctl.get_state()["status"] == "active"
    finish(handle, SUCCEEDED)

    state = ctl.get_state()
    assert state["generation"] == 1
    assert state["status"] == "succeeded"
    assert target(state) == (2.0, -1.0, math.pi / 2)
    assert state["message"] == "navigation succeeded"
    assert state["reason"] is None


def test_rejected_goal_maps_to_rejected():
    ctl, client, _ = make_controller()
    ctl.submit(1, 2, 0)

    accept(client, accepted=False)

    state = ctl.get_state()
    assert state["status"] == "rejected"
    assert state["reason"] == "goal_rejected"


def test_aborted_and_canceled_results_are_distinct():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    first = accept(client)
    finish(first, ABORTED)
    assert ctl.get_state()["status"] == "aborted"
    assert ctl.get_state()["reason"] == "nav2_aborted"

    ctl.submit(2, 0, 0)
    second = accept(client, 1)
    finish(second, CANCELED)
    assert ctl.get_state()["status"] == "canceled"
    assert ctl.get_state()["reason"] == "nav2_canceled"


@pytest.mark.parametrize("phase", ["send", "result"])
def test_future_exception_maps_to_error(phase):
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)

    if phase == "send":
        client.send_futures[0].fail(RuntimeError("send exploded"))
    else:
        handle = accept(client)
        handle.result_future.fail(RuntimeError("result exploded"))

    state = ctl.get_state()
    assert state["status"] == "error"
    assert state["reason"] == f"{phase}_exception"
    assert "exploded" in state["message"]


def test_already_completed_result_future_cannot_overwrite_success_with_active():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    handle = FakeGoalHandle()
    finish(handle, SUCCEEDED)

    client.send_futures[0].resolve(handle)

    assert ctl.get_state()["status"] == "succeeded"


def test_cancel_future_exception_is_retried_by_watchdog():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    handle = accept(client)
    ctl.cancel("operator_stop")
    assert handle.cancel_calls == 1
    handle.cancel_future.fail(RuntimeError("cancel transport failed"))

    ctl.tick()

    assert handle.cancel_calls == 2


def test_stop_marks_stopped_before_cancel_state_callback_can_release_lock_boundary():
    cancel_callback_entered = threading.Event()
    release_cancel_callback = threading.Event()

    def blocking_callback(state):
        if state["status"] == "canceling" and state["reason"] == "shutdown":
            cancel_callback_entered.set()
            assert release_cancel_callback.wait(1.0)

    ctl, client, _ = make_controller(state_callback=blocking_callback)
    ctl.submit(1, 0, 0)
    accept(client)
    stop_thread = threading.Thread(target=ctl.stop)
    stop_thread.start()
    try:
        assert cancel_callback_entered.wait(1.0)
        with pytest.raises(RuntimeError, match="stopped"):
            ctl.submit(2, 0, 0)
    finally:
        release_cancel_callback.set()
        stop_thread.join(1.0)
    assert not stop_thread.is_alive()
    assert len(client.goals) == 1


def test_stopped_controller_retries_transient_cancel_failure_on_tick():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    handle = TransientCancelFailureGoalHandle()
    client.send_futures[0].resolve(handle)

    ctl.stop()
    assert handle.cancel_calls == 1

    ctl.tick()
    assert handle.cancel_calls == 2
    assert len(client.goals) == 1


def test_send_callback_registration_failure_quarantines_and_never_sends_again():
    ctl, client, _ = make_controller()
    client.future_factory = CallbackRegistrationFailureFuture

    ctl.submit(1, 0, 0)

    state = ctl.get_state()
    assert state["status"] == "error"
    assert state["reason"] == "send_callback_registration_exception"
    with pytest.raises(RuntimeError, match="quarantined"):
        ctl.submit(2, 0, 0)
    assert len(client.goals) == 1

    # tick() may safely observe the handle and cancel it, but quarantine is
    # permanent because the original completion callback could not be owned.
    handle = FakeGoalHandle()
    client.send_futures[0].resolve(handle)
    ctl.tick()
    assert handle.cancel_calls == 1
    with pytest.raises(RuntimeError, match="quarantined"):
        ctl.submit(3, 0, 0)
    assert len(client.goals) == 1


def test_quarantined_controller_retries_transient_cancel_failure_on_tick():
    ctl, client, _ = make_controller()
    client.future_factory = CallbackRegistrationFailureFuture
    ctl.submit(1, 0, 0)
    handle = TransientCancelFailureGoalHandle()
    client.send_futures[0].resolve(handle)

    ctl.tick()
    assert handle.cancel_calls == 1

    ctl.tick()
    assert handle.cancel_calls == 2
    assert len(client.goals) == 1
    with pytest.raises(RuntimeError, match="quarantined"):
        ctl.submit(2, 0, 0)


def test_partially_registered_send_callback_still_cancels_under_quarantine():
    ctl, client, _ = make_controller()
    client.future_factory = RegisterThenRaiseFuture
    ctl.submit(1, 0, 0)
    handle = FakeGoalHandle()

    client.send_futures[0].resolve(handle)

    assert handle.cancel_calls == 1
    assert ctl.get_state()["status"] == "error"
    assert ctl.get_state()["reason"] == "send_callback_registration_exception"
    with pytest.raises(RuntimeError, match="quarantined"):
        ctl.submit(2, 0, 0)
    assert len(client.goals) == 1


def test_result_callback_registration_failure_retains_active_ownership():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    handle = FakeGoalHandle()
    handle.result_future = CallbackRegistrationFailureFuture()

    client.send_futures[0].resolve(handle)

    state = ctl.get_state()
    assert state["status"] == "error"
    assert state["reason"] == "result_callback_registration_exception"
    assert handle.cancel_calls == 1
    with pytest.raises(RuntimeError, match="quarantined"):
        ctl.submit(2, 0, 0)
    assert len(client.goals) == 1


def test_cancel_callback_registration_failure_waits_for_result_before_replacement():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    handle = FakeGoalHandle()
    handle.cancel_future_factory = CallbackRegistrationFailureFuture
    client.send_futures[0].resolve(handle)

    ctl.submit(2, 0, 0)

    assert handle.cancel_calls == 1
    assert ctl.get_state()["reason"] == "cancel_callback_registration_exception"
    ctl.tick()
    assert handle.cancel_calls == 1
    assert len(client.goals) == 1
    finish(handle, CANCELED)
    assert len(client.goals) == 2


def test_active_replacement_waits_for_old_terminal_result():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    first = accept(client)

    replacement = ctl.submit(3, 1, 0.5)

    assert replacement["generation"] == 2
    assert first.cancel_calls == 1
    assert len(client.goals) == 1
    accept_cancel(first)
    assert len(client.goals) == 1, "cancel acknowledgement is not a terminal result"

    finish(first, CANCELED)
    assert len(client.goals) == 2
    assert client.goals[1].pose.pose.position.x == 3.0
    assert ctl.get_state()["generation"] == 2
    assert ctl.get_state()["status"] == "pending"


def test_replacement_while_send_response_pending_cancels_after_acceptance():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    ctl.submit(4, 2, -0.25)

    assert len(client.goals) == 1
    first = accept(client)
    assert first.cancel_calls == 1
    assert len(client.goals) == 1

    finish(first, CANCELED)
    assert len(client.goals) == 2
    assert client.goals[1].pose.pose.position.x == 4.0
    assert client.goals[1].pose.pose.position.y == 2.0


def test_cancel_before_handle_acceptance_cancels_accepted_stale_goal():
    ctl, client, _ = make_controller()
    ctl.submit(1, 2, 0.1)

    assert ctl.cancel("emergency_stop") is True
    state_while_pending = ctl.get_state()
    assert state_while_pending["generation"] == 2
    assert state_while_pending["status"] == "canceling"
    assert target(state_while_pending) == (1.0, 2.0, 0.1)

    handle = accept(client)
    assert handle.cancel_calls == 1
    finish(handle, CANCELED)

    state = ctl.get_state()
    assert state["generation"] == 2
    assert state["status"] == "canceled"
    assert state["reason"] == "emergency_stop"
    assert target(state) == (1.0, 2.0, 0.1)


def test_active_cancel_waits_for_terminal_and_preserves_target():
    ctl, client, _ = make_controller()
    ctl.submit(-1, 3, -1.0)
    handle = accept(client)

    assert ctl.cancel("operator_stop") is True
    assert handle.cancel_calls == 1
    assert ctl.get_state()["status"] == "canceling"
    accept_cancel(handle)
    assert ctl.get_state()["status"] == "canceling"

    finish(handle, CANCELED)
    state = ctl.get_state()
    assert state["status"] == "canceled"
    assert state["reason"] == "operator_stop"
    assert target(state) == (-1.0, 3.0, -1.0)


def test_empty_cancel_is_exact_noop_and_never_synthesizes_zero_target():
    events = []
    ctl, client, _ = make_controller(state_callback=events.append)
    before = ctl.get_state()

    assert ctl.cancel("operator_stop") is False

    assert ctl.get_state() == before
    assert events == []
    assert client.goals == []


def test_concurrent_submits_keep_only_highest_generation_and_ignore_stale_result():
    ctl, client, _ = make_controller()
    ctl.submit(0, 0, 0)
    first = accept(client)
    barrier = threading.Barrier(5)
    responses = []
    responses_lock = threading.Lock()

    def submit_target(index):
        barrier.wait()
        response = ctl.submit(index, -index, index / 10.0)
        with responses_lock:
            responses.append(response)

    threads = [threading.Thread(target=submit_target, args=(i,)) for i in range(1, 5)]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join()

    newest = max(responses, key=lambda item: item["generation"])
    state = ctl.get_state()
    assert state["generation"] == newest["generation"] == 5
    assert target(state) == (
        newest["goal"]["x"],
        newest["goal"]["y"],
        newest["goal"]["yaw"],
    )
    assert first.cancel_calls == 1

    finish(first, CANCELED)
    assert len(client.goals) == 2
    second = accept(client, 1)
    assert ctl.get_state()["status"] == "active"
    assert target(ctl.get_state()) == target(state)

    # A duplicate late callback for the old future must not overwrite goal 5.
    finish(first, ABORTED)
    assert ctl.get_state()["status"] == "active"
    assert target(ctl.get_state()) == target(state)
    finish(second, SUCCEEDED)


def test_server_unavailable_expires_without_blocking_or_late_send():
    ctl, client, monotonic = make_controller(ready=False, server_timeout=2.0)
    ctl.submit(1, 1, 0)

    assert ctl.get_state()["status"] == "waiting_server"
    assert client.goals == []
    monotonic.advance(1.9)
    ctl.tick()
    assert ctl.get_state()["status"] == "waiting_server"

    monotonic.advance(0.2)
    ctl.tick()
    state = ctl.get_state()
    assert state["status"] == "server_unavailable"
    assert state["reason"] == "server_unavailable"
    client.ready = True
    ctl.tick()
    assert client.goals == []


@pytest.mark.parametrize(
    "values",
    [
        (math.nan, 0, 0),
        (0, math.inf, 0),
        (0, 0, -math.inf),
        ("not-a-number", 0, 0),
    ],
)
def test_submit_rejects_non_finite_or_non_numeric_input_without_mutation(values):
    ctl, client, _ = make_controller()
    before = ctl.get_state()

    with pytest.raises(ValueError):
        ctl.submit(*values)

    assert ctl.get_state() == before
    assert client.goals == []


def test_active_timeout_cancels_and_maps_terminal_to_timed_out():
    ctl, client, monotonic = make_controller(active_timeout=5.0)
    ctl.submit(2, 0, 0)
    handle = accept(client)

    monotonic.advance(5.1)
    ctl.tick()

    assert handle.cancel_calls == 1
    assert ctl.get_state()["status"] == "canceling"
    assert ctl.get_state()["reason"] == "timeout"
    finish(handle, CANCELED)
    assert ctl.get_state()["status"] == "timed_out"
    assert ctl.get_state()["reason"] == "timeout"


def test_health_loss_cancels_active_and_holds_replacement_until_recovery():
    health = {"ok": True}
    ctl, client, monotonic = make_controller(health_check=lambda: health["ok"])
    ctl.submit(1, 0, 0)
    first = accept(client)

    health["ok"] = False
    ctl.tick()
    assert first.cancel_calls == 0
    assert ctl.get_state()["health_degraded"] is True

    monotonic.advance(0.51)
    ctl.tick()
    assert first.cancel_calls == 1
    assert ctl.get_state()["reason"] == "localization_unhealthy"

    ctl.submit(5, 2, 0)
    finish(first, CANCELED)
    assert len(client.goals) == 1
    state = ctl.get_state()
    assert state["status"] == "waiting_health"
    assert target(state) == (5.0, 2.0, 0.0)

    health["ok"] = True
    ctl.tick()
    assert len(client.goals) == 2
    assert ctl.get_state()["status"] == "pending"


def test_transient_health_loss_inside_grace_does_not_cancel_active_goal():
    health = {"ok": True}
    ctl, client, monotonic = make_controller(
        health_check=lambda: health["ok"], health_failure_grace=0.5)
    ctl.submit(1, 0, 0)
    handle = accept(client)

    health["ok"] = False
    ctl.tick()
    monotonic.advance(0.3)
    health["ok"] = True
    ctl.tick()

    state = ctl.get_state()
    assert handle.cancel_calls == 0
    assert state["status"] == "active"
    assert state["healthy"] is True
    assert state["health_degraded"] is False


def test_initial_unhealthy_state_does_not_send_until_watchdog_recovers():
    health = {"ok": False}
    ctl, client, _ = make_controller(health_check=lambda: health["ok"])

    ctl.submit(3, 4, 0)
    assert client.goals == []
    assert ctl.get_state()["status"] == "waiting_health"

    health["ok"] = True
    ctl.tick()
    assert len(client.goals) == 1


def test_stop_cancels_active_and_disallows_new_submissions():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    handle = accept(client)

    ctl.stop()

    assert handle.cancel_calls == 1
    with pytest.raises(RuntimeError, match="stopped"):
        ctl.submit(2, 0, 0)


def test_controller_never_spins_or_imports_blocking_room_nav_client():
    tree = ast.parse(Path(point_nav_module.__file__).read_text(encoding="utf-8"))
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    imported_modules = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    } | {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module is not None
    }

    assert not any(name.startswith("spin") for name in called_attributes)
    assert "nx_room_orchestrator" not in imported_modules


def test_rejected_cancel_is_retried_then_replacement_waits_for_terminal():
    ctl, client, _ = make_controller(cancel_max_attempts=2)
    ctl.submit(1, 0, 0)
    first = accept(client)
    ctl.submit(2, 0, 0)

    reject_cancel(first)
    assert first.cancel_calls == 1
    assert len(client.goals) == 1
    assert ctl.get_state()["status"] == "cancel_failed"
    assert ctl.get_state()["reason"] == "cancel_rejected"

    ctl.tick()
    assert first.cancel_calls == 2
    accept_cancel(first)
    assert ctl.get_state()["cancel_acknowledged"] is True
    assert len(client.goals) == 1

    finish(first, CANCELED)
    assert len(client.goals) == 2


@pytest.mark.parametrize(
    "response",
    [
        FakeCancelResponse(FakeCancelResponse.ERROR_REJECTED, [SimpleNamespace()]),
        FakeCancelResponse(FakeCancelResponse.ERROR_NONE, []),
    ],
)
def test_cancel_rejection_exhaustion_quarantines_and_success_is_not_canceled(response):
    ctl, client, _ = make_controller(cancel_max_attempts=1)
    ctl.submit(1, 0, 0)
    handle = accept(client)
    ctl.cancel("operator_stop")

    handle.cancel_future.resolve(response)

    state = ctl.get_state()
    assert state["status"] == "cancel_failed"
    assert state["quarantined"] is True
    assert state["drained"] is False
    finish(handle, SUCCEEDED)
    state = ctl.get_state()
    assert state["status"] == "cancel_failed"
    assert state["reason"] == "cancel_rejected_goal_succeeded"
    assert state["drained"] is True


def test_pending_send_acceptance_deadline_quarantines_and_retains_ownership():
    ctl, client, monotonic = make_controller(send_response_timeout=1.0)
    ctl.submit(1, 0, 0)
    monotonic.advance(1.1)

    ctl.tick()

    state = ctl.get_state()
    assert state["status"] == "error"
    assert state["reason"] == "send_response_timeout"
    assert state["quarantined"] is True
    assert state["sending"] is True
    assert state["drained"] is False
    with pytest.raises(RuntimeError, match="quarantined"):
        ctl.submit(2, 0, 0)
    assert len(client.goals) == 1


def test_pending_cancel_response_deadline_quarantines_active_goal():
    ctl, client, monotonic = make_controller(cancel_response_timeout=1.0)
    ctl.submit(1, 0, 0)
    handle = accept(client)
    ctl.cancel("operator_stop")
    monotonic.advance(1.1)

    ctl.tick()

    state = ctl.get_state()
    assert state["status"] == "error"
    assert state["reason"] == "cancel_response_timeout"
    assert state["quarantined"] is True
    assert state["active"] is True
    assert handle.cancel_calls == 1


def test_cancel_to_terminal_deadline_quarantines_forever_pending_result():
    ctl, client, monotonic = make_controller(cancel_terminal_timeout=1.0)
    ctl.submit(1, 0, 0)
    handle = accept(client)
    ctl.cancel("operator_stop")
    accept_cancel(handle)
    monotonic.advance(1.1)

    ctl.tick()

    state = ctl.get_state()
    assert state["status"] == "error"
    assert state["reason"] == "cancel_terminal_timeout"
    assert state["quarantined"] is True
    assert state["active"] is True


def test_cancel_callback_registration_failure_is_polled_without_duplicate_cancel():
    ctl, client, _ = make_controller()
    ctl.submit(1, 0, 0)
    handle = FakeGoalHandle()
    handle.cancel_future_factory = CallbackRegistrationFailureFuture
    client.send_futures[0].resolve(handle)
    ctl.submit(2, 0, 0)
    accept_cancel(handle)

    ctl.tick()

    state = ctl.get_state()
    assert state["cancel_acknowledged"] is True
    assert handle.cancel_calls == 1
    assert len(client.goals) == 1
    finish(handle, CANCELED)
    assert len(client.goals) == 2


@pytest.mark.parametrize(
    "name,value",
    [
        ("send_response_timeout", 0),
        ("cancel_response_timeout", math.inf),
        ("cancel_terminal_timeout", -1),
        ("cancel_max_attempts", 0),
        ("cancel_max_attempts", 1.5),
        ("health_failure_grace", -0.1),
        ("health_failure_grace", math.inf),
    ],
)
def test_watchdog_configuration_is_finite_and_bounded(name, value):
    kwargs = {name: value}
    with pytest.raises(ValueError):
        make_controller(**kwargs)


class ReorderedHealth:
    def __init__(self):
        self.calls = 0
        self.old_sample_started = threading.Event()
        self.release_old_sample = threading.Event()

    def __call__(self):
        self.calls += 1
        if self.calls == 1:
            return True
        if self.calls == 2:
            self.old_sample_started.set()
            assert self.release_old_sample.wait(1.0)
            return True
        return False


def test_stale_healthy_sample_cannot_overwrite_newer_unhealthy_sample():
    health = ReorderedHealth()
    ctl, client, _ = make_controller(
        health_check=health, health_failure_grace=0.0)
    submit_result = []
    submit_thread = threading.Thread(target=lambda: submit_result.append(ctl.submit(1, 0, 0)))
    submit_thread.start()
    assert health.old_sample_started.wait(1.0)

    ctl.tick()
    health.release_old_sample.set()
    submit_thread.join(1.0)

    assert not submit_thread.is_alive()
    assert submit_result[0]["ok"] is True
    assert client.goals == []
    assert ctl.get_state()["status"] == "waiting_health"
    assert ctl.get_state()["reason"] == "localization_unhealthy"


def test_rejected_state_callback_does_not_hold_controller_lock_against_stop():
    callback_entered = threading.Event()
    release_callback = threading.Event()

    def callback(state):
        if state["status"] == "rejected":
            callback_entered.set()
            assert release_callback.wait(1.0)

    ctl, client, _ = make_controller(state_callback=callback)
    ctl.submit(1, 0, 0)
    resolver = threading.Thread(target=lambda: accept(client, accepted=False))
    resolver.start()
    assert callback_entered.wait(1.0)

    stopper = threading.Thread(target=ctl.stop)
    stopper.start()
    stopper.join(0.2)
    try:
        assert not stopper.is_alive(), "state_callback was invoked while controller lock was held"
    finally:
        release_callback.set()
        resolver.join(1.0)
        stopper.join(1.0)


def test_notifications_coalesce_to_latest_state_while_consumer_is_blocked():
    callback_entered = threading.Event()
    release_callback = threading.Event()
    statuses = []

    def callback(state):
        statuses.append(state["status"])
        if len(statuses) == 1:
            callback_entered.set()
            assert release_callback.wait(1.0)

    ctl, client, _ = make_controller(state_callback=callback)
    submitter = threading.Thread(target=lambda: ctl.submit(1, 0, 0))
    submitter.start()
    assert callback_entered.wait(1.0)
    handle = accept(client)
    finish(handle, SUCCEEDED)
    release_callback.set()
    submitter.join(1.0)

    assert statuses == ["pending", "succeeded"]
