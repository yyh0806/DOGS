"""Fault-injection tests for shared Nav2 ownership and bounded arbitration."""

from __future__ import annotations

import sys
import threading
from pathlib import Path

import pytest


WEB = Path(__file__).resolve().parent
if str(WEB) not in sys.path:
    sys.path.insert(0, str(WEB))

class FakePointNav:
    def __init__(self, events, *, drained=True, quarantined=False, stopped=False):
        self.events = events
        self.state = {
            "drained": drained,
            "quarantined": quarantined,
            "stopped": stopped,
            "healthy": True,
        }
        self.submissions = []

    def get_state(self):
        return dict(self.state)

    def cancel(self, reason):
        self.events.append(("point_cancel", reason))
        return not self.state["drained"]

    def tick(self):
        self.events.append(("point_tick",))
        return dict(self.state)

    def submit(self, x, y, yaw):
        self.events.append(("point_submit", x, y, yaw))
        self.submissions.append((x, y, yaw))
        return {"ok": True, "generation": 1}

    def stop(self):
        self.events.append(("point_stop",))


class FakeTaskManager:
    def __init__(self, events, *, drains=True):
        self.events = events
        self.drains = drains
        self.enqueued = []

    def cancel_all(self):
        self.events.append(("task_cancel",))

    def wait_drained(self, timeout):
        self.events.append(("task_wait", timeout))
        return self.drains

    def _add_list_unchecked(self, tasks):
        self.events.append(("task_enqueue", tuple(tasks)))
        self.enqueued.extend(tasks)


class FakeRobot:
    def __init__(self, events, *, activation_ok=True, ready_ok=True):
        self.events = events
        self.activation_ok = activation_ok
        self.ready_ok = ready_ok
        self.session = "parked"
        self.owner = None
        self.sport_mode = 6
        self.wheel_dq = [0.0, 0.0, 0.0, 0.0]

    def stop_move(self):
        self.events.append(("robot_stop",))

    def e_stop(self):
        self.events.append(("robot_e_stop",))

    def start_drive_session(self, owner):
        self.events.append(("drive_start", owner))
        if self.activation_ok:
            self.session = "active"
            self.owner = owner
            self.sport_mode = 1
        return {"ok": self.activation_ok, "reason": None if self.activation_ok else "rejected"}

    def wait_drive_ready(self, owner, timeout):
        self.events.append(("drive_wait", owner, timeout))
        return {"ok": self.ready_ok, "reason": None if self.ready_ok else "activation_timeout"}

    def park_drive_session(self, reason):
        self.events.append(("drive_park", reason))
        self.session = "parked"
        self.owner = None
        self.sport_mode = 6
        return {"ok": True}

    def wait_drive_parked(self, timeout):
        self.events.append(("park_wait", timeout))
        return True

    def get_drive_session_state(self):
        return {
            "drive_session": self.session,
            "drive_session_owner": self.owner,
            "sport_mode": self.sport_mode,
            "wheel_dq": list(self.wheel_dq),
        }


class FakeTimer:
    def __init__(self, delay, callback):
        self.delay = delay
        self.callback = callback
        self.daemon = False
        self.started = False
        self.cancelled = False

    def start(self):
        self.started = True

    def cancel(self):
        self.cancelled = True

    def fire(self):
        if not self.cancelled:
            self.callback()


class FakeTimerFactory:
    def __init__(self):
        self.timers = []

    def __call__(self, delay, callback):
        timer = FakeTimer(delay, callback)
        self.timers.append(timer)
        return timer


def load_arbiter():
    from nx_navigation_arbiter import NavigationArbiter

    return NavigationArbiter


def test_quarantined_point_submit_fails_preflight_without_canceling_room_task():
    events = []
    point = FakePointNav(events, quarantined=True)
    tasks = FakeTaskManager(events)
    arbiter = load_arbiter()(point, tasks, FakeRobot(events), transition_timeout=0.05)

    result = arbiter.start_point_goal(1.0, 2.0, 0.0)

    assert result["ok"] is False
    assert result["reason"] == "point_nav_quarantined"
    assert events == []


def test_point_goal_waits_for_room_task_drain_before_submit():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events, drains=True)
    arbiter = load_arbiter()(point, tasks, FakeRobot(events), transition_timeout=0.05)

    result = arbiter.start_point_goal(1.0, 2.0, 0.3)

    assert result["ok"] is True
    assert [event[0] for event in events] == [
        "task_cancel", "task_wait", "park_wait", "drive_start", "drive_wait",
        "point_tick", "point_submit",
    ]


def test_point_goal_never_submits_when_feedback_confirmed_activation_fails():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events, drains=True)
    robot = FakeRobot(events, ready_ok=False)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)

    result = arbiter.start_point_goal(1.0, 2.0, 0.3)

    assert result["ok"] is False
    assert result["reason"] == "activation_timeout"
    assert not any(event[0] == "point_submit" for event in events)
    assert ("drive_park", "point_activation_failed") in events


@pytest.mark.parametrize(
    "status",
    ["succeeded", "aborted", "rejected", "timed_out", "canceled", "error"],
)
def test_point_terminal_state_parks_once(status):
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)
    arbiter.start_point_goal(1.0, 2.0, 0.0)
    events.clear()

    arbiter.on_point_state({"status": status, "drained": True})
    arbiter.on_point_state({"status": status, "drained": True})

    assert events == [("drive_park", f"nav_point_{status}")]
    assert robot.session == "parked"
    assert robot.owner is None
    assert robot.sport_mode == 6


def test_room_task_terminal_parks_once():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)
    result = arbiter.start_tasks(["room"], reason="search_room")
    assert result["ok"] is True
    events.clear()

    arbiter.on_tasks_drained()
    arbiter.on_tasks_drained()

    assert events == [("drive_park", "nav_tasks_drained")]
    assert robot.session == "parked"
    assert robot.owner is None
    assert robot.sport_mode == 6


def test_repeated_manual_commands_share_one_feedback_confirmed_session():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)

    first = arbiter.run_manual_action(
        "manual_move", lambda: events.append(("manual_command", 0.2)))
    second = arbiter.run_manual_action(
        "manual_move", lambda: events.append(("manual_command", 0.3)))

    assert first["ok"] is True and second["ok"] is True
    assert [event[0] for event in events].count("drive_start") == 1
    assert [event[0] for event in events].count("drive_wait") == 1
    assert [event[0] for event in events].count("manual_command") == 2


def test_global_stop_always_cancels_both_autonomy_owners_zeros_and_parks():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)

    result = arbiter.stop_all("operator_stop")

    assert result["ok"] is True
    assert [event[0] for event in events] == [
        "point_cancel", "task_cancel", "task_wait",
        "robot_stop", "drive_park",
    ]


def test_manual_activation_waits_for_feedback_confirmed_park_before_start():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)

    result = arbiter.run_manual_action(
        "manual_move", lambda: events.append(("manual_command",))
    )

    assert result["ok"] is True
    names = [event[0] for event in events]
    assert names.index("park_wait") < names.index("drive_start")


def test_operator_stop_zeros_then_parks_manual_session():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)
    arbiter.run_manual_action("manual_move", lambda: events.append(("manual_command",)))
    events.clear()

    result = arbiter.stop_manual("operator_stop")

    assert result["ok"] is True
    assert events == [("robot_stop",), ("drive_park", "operator_stop")]


def test_manual_release_is_a_noop_while_autonomy_owns_motion():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)
    arbiter._motion_owner = "tasks"

    result = arbiter.release_manual("browser_blur")

    assert result == {
        "ok": True,
        "phase": "unchanged",
        "owner": "tasks",
        "ignored": True,
    }
    assert events == []


def test_manual_release_zeros_without_switching_out_of_wheel_mode():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)
    arbiter.run_manual_action("manual_move", lambda: None)
    events.clear()

    result = arbiter.release_manual("manual_release")

    assert result["ok"] is True
    assert result["phase"] == "idle"
    assert result["owner"] == "manual"
    assert events == [("robot_stop",)]

    resumed = arbiter.run_manual_action(
        "manual_move", lambda: events.append(("manual_command", 0.2))
    )

    assert resumed["ok"] is True
    assert not any(event[0] in {"drive_park", "drive_start"} for event in events)


def test_repeated_key_press_release_never_creates_pose_transition_storm():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)

    for _ in range(25):
        assert arbiter.run_manual_action("manual_move", lambda: None)["ok"] is True
        assert arbiter.release_manual("key_up")["ok"] is True

    assert [event[0] for event in events].count("drive_start") == 1
    assert [event[0] for event in events].count("drive_park") == 0
    assert [event[0] for event in events].count("robot_stop") == 25


def test_manual_release_parks_and_releases_owner_after_idle_lease():
    events = []
    timers = FakeTimerFactory()
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(
        point,
        tasks,
        robot,
        transition_timeout=0.05,
        manual_idle_timeout=0.8,
        timer_factory=timers,
    )
    arbiter.run_manual_action("manual_move", lambda: None)
    events.clear()

    result = arbiter.release_manual("manual_release")

    assert result["ok"] is True
    assert events == [("robot_stop",)]
    assert arbiter.get_motion_owner() == "manual"
    assert len(timers.timers) == 1
    assert timers.timers[0].delay == pytest.approx(0.8)
    assert timers.timers[0].daemon is True
    assert timers.timers[0].started is True

    timers.timers[0].fire()

    assert events == [
        ("robot_stop",),
        ("drive_park", "manual_idle_timeout"),
    ]
    assert arbiter.get_motion_owner() is None
    assert robot.session == "parked"


def test_new_manual_command_cancels_pending_idle_lease():
    events = []
    timers = FakeTimerFactory()
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(
        point,
        tasks,
        robot,
        transition_timeout=0.05,
        manual_idle_timeout=0.8,
        timer_factory=timers,
    )
    arbiter.run_manual_action("manual_move", lambda: None)
    arbiter.release_manual("manual_release")
    pending = timers.timers[0]
    events.clear()

    result = arbiter.run_manual_action(
        "manual_move", lambda: events.append(("manual_command",)))
    pending.fire()

    assert result["ok"] is True
    assert pending.cancelled is True
    assert events == [("manual_command",)]
    assert arbiter.get_motion_owner() == "manual"
    assert robot.session == "active"


def test_point_goal_handoffs_manual_wheel_session_to_nav_without_pose_switch():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)
    arbiter.run_manual_action("manual_move", lambda: None)
    events.clear()

    result = arbiter.start_point_goal(1.0, 0.0, 0.0)

    assert result["ok"] is True
    assert not any(event[0] in {"drive_park", "park_wait"} for event in events)
    assert [event for event in events if event[0] == "drive_start"] == [
        ("drive_start", "nav")
    ]


def test_room_task_reuses_an_already_active_nav_session_idempotently():
    events = []
    point = FakePointNav(events)
    tasks = FakeTaskManager(events)
    robot = FakeRobot(events)
    robot.session = "nav_active"
    robot.owner = "nav"
    robot.sport_mode = 1
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.05)

    result = arbiter.start_tasks(["room"], reason="search_room")

    assert result["ok"] is True
    assert ("robot_stop",) in events
    assert ("drive_wait", "nav", 6.0) in events
    assert not any(event[0] == "drive_start" for event in events)
    assert tasks.enqueued == ["room"]


def test_room_task_refreshes_navigation_health_after_drive_activation():
    events = []

    class RefreshingPointNav(FakePointNav):
        def __init__(self, shared_events):
            super().__init__(shared_events)
            self.state["healthy"] = False

        def tick(self):
            state = super().tick()
            self.state["healthy"] = True
            return state

    point = RefreshingPointNav(events)
    tasks = FakeTaskManager(events)
    arbiter = load_arbiter()(
        point, tasks, FakeRobot(events), transition_timeout=0.05)

    result = arbiter.start_tasks(["room"], reason="search_room")

    assert result["ok"] is True
    names = [event[0] for event in events]
    assert names.index("point_tick") < names.index("task_enqueue")
    assert tasks.enqueued == ["room"]


def test_recover_task_motion_reactivates_parked_drive_without_canceling_task():
    events = []
    robot = FakeRobot(events)

    class RecoveringPointNav(FakePointNav):
        def tick(self):
            state = super().tick()
            if robot.session in {"active", "nav_active"}:
                self.state["healthy"] = True
            return state

    point = RecoveringPointNav(events)
    tasks = FakeTaskManager(events)
    arbiter = load_arbiter()(
        point, tasks, robot, transition_timeout=0.05)
    assert arbiter.start_tasks(["room"], reason="search_room")["ok"] is True

    events.clear()
    robot.session = "parked"
    robot.owner = None
    robot.sport_mode = 6
    point.state["healthy"] = False

    result = arbiter.recover_task_motion("motion_unhealthy")

    assert result["ok"] is True
    assert ("drive_start", "nav") in events
    assert ("drive_wait", "nav", 6.0) in events
    assert ("point_tick",) in events
    assert not any(event[0] == "task_cancel" for event in events)
    assert arbiter.get_motion_owner() == "tasks"


def test_recover_task_motion_refuses_when_tasks_do_not_own_motion():
    events = []
    arbiter = load_arbiter()(
        FakePointNav(events), FakeTaskManager(events), FakeRobot(events),
        transition_timeout=0.05,
    )

    result = arbiter.recover_task_motion("motion_unhealthy")

    assert result == {"ok": False, "reason": "task_motion_not_owned"}
    assert not any(event[0] == "drive_start" for event in events)


def test_room_task_rejects_when_health_stays_bad_after_drive_activation():
    events = []
    point = FakePointNav(events)
    point.state["healthy"] = False
    tasks = FakeTaskManager(events)
    arbiter = load_arbiter()(
        point, tasks, FakeRobot(events), transition_timeout=0.05)

    result = arbiter.start_tasks(["room"], reason="search_room")

    assert result == {"ok": False, "reason": "point_nav_unhealthy"}
    assert tasks.enqueued == []
    assert ("drive_park", "task_preflight_failed") in events


def test_new_task_is_not_enqueued_until_point_goal_is_confirmed_drained():
    events = []
    point = FakePointNav(events, drained=False)
    tasks = FakeTaskManager(events)
    arbiter = load_arbiter()(
        point,
        tasks,
        FakeRobot(events),
        transition_timeout=0.02,
        poll_interval=0.005,
    )

    result = arbiter.start_tasks(["room"], reason="search_room")

    assert result["ok"] is False
    assert result["reason"] == "point_nav_not_drained"
    assert not tasks.enqueued
    assert not any(event[0] == "task_cancel" for event in events)


def test_shutdown_parks_when_room_task_does_not_drain():
    events = []
    point = FakePointNav(events, drained=True)
    tasks = FakeTaskManager(events, drains=False)
    robot = FakeRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.02)

    result = arbiter.shutdown()

    assert result["ok"] is False
    assert result["tasks_drained"] is False
    assert ("drive_park", "drain_timeout") in events
    assert ("robot_e_stop",) not in events


def test_emergency_stop_is_not_blocked_by_an_admission_drain_lock():
    events = []
    entered_drain = threading.Event()
    release_drain = threading.Event()
    estopped = threading.Event()

    class BlockingTasks(FakeTaskManager):
        def wait_drained(self, timeout):
            self.events.append(("task_wait", timeout))
            entered_drain.set()
            release_drain.wait(1.0)
            return True

    class ObservableRobot(FakeRobot):
        def e_stop(self):
            super().e_stop()
            estopped.set()

    point = FakePointNav(events)
    tasks = BlockingTasks(events)
    robot = ObservableRobot(events)
    arbiter = load_arbiter()(point, tasks, robot, transition_timeout=0.5)

    admission = threading.Thread(
        target=lambda: arbiter.start_point_goal(1.0, 2.0, 0.0)
    )
    admission.start()
    assert entered_drain.wait(0.2)

    emergency = threading.Thread(target=arbiter.emergency_stop)
    emergency.start()
    try:
        assert estopped.wait(0.2), "physical e-stop must bypass the admission mutex"
    finally:
        release_drain.set()
        admission.join(1.0)
        emergency.join(1.0)

    assert not admission.is_alive()
    assert not emergency.is_alive()
    assert not any(event[0] == "point_submit" for event in events)
