"""Read-only NX process that persists raw Go2W safety state."""

from __future__ import annotations

import os
import signal
import threading
from typing import Any

try:
    from .safety_event_recorder import SafetyEventRecorder
except ImportError:  # Direct-file compatibility deployment on the NX.
    from safety_event_recorder import SafetyEventRecorder


class RawSafetyObserver:
    """Merge read-only SportModeState and LowState into durable snapshots."""

    def __init__(self, recorder: SafetyEventRecorder) -> None:
        self._recorder = recorder
        self._lock = threading.RLock()
        self._state: dict[str, Any] = {}

    def on_sport(self, message: Any) -> None:
        self._update({
            "error_code": int(message.error_code),
            "mode": int(message.mode),
            "gait_type": int(message.gait_type),
            "progress": float(message.progress),
            "velocity": [float(value) for value in message.velocity],
            "yaw_speed": float(message.yaw_speed),
            "position": [float(value) for value in message.position],
        })

    def on_low(self, message: Any) -> None:
        motors = list(message.motor_state)
        rpy = list(message.imu_state.rpy)
        self._update({
            "wheel_dq": [float(motors[index].dq)
                         for index in (12, 13, 14, 15)],
            # ``lost`` is a historical communication-loss counter, retained
            # as evidence but deliberately not classified as a live fault.
            "motor_lost": [int(motor.lost) for motor in motors],
            "motor_mode": [int(motor.mode) for motor in motors],
            "motor_temp": [int(motor.temperature) for motor in motors],
            "roll": float(rpy[0]),
            "pitch": float(rpy[1]),
            "foot_force": [int(value) for value in message.foot_force],
            "battery_soc": int(message.bms_state.soc),
            "bms_status": int(message.bms_state.status),
            "power_v": float(message.power_v),
            "power_a": float(message.power_a),
            "level_flag": int(message.level_flag),
            "bit_flag": int(message.bit_flag),
        })

    def _update(self, update: dict[str, Any]) -> None:
        with self._lock:
            self._state.update(update)
            snapshot = dict(self._state)
        self._recorder.record_state(snapshot)


def main() -> None:
    from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_._SportModeState_ import (
        SportModeState_,
    )

    interface = os.environ.get("DOG_INTERFACE", "enxc8a362616c4c")
    event_log = os.environ.get(
        "GO2W_SAFETY_EVENT_LOG",
        "/home/nx/go2w/safety-events/events.jsonl",
    )
    recorder = SafetyEventRecorder(event_log)
    recorder.record_event("safety_observer_process_start", {
        "interface": interface,
    })
    stop = threading.Event()

    def request_stop(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    factory = ChannelFactory()
    if not factory.Init(0, interface):
        raise RuntimeError("ChannelFactory initialization failed")

    observer = RawSafetyObserver(recorder)
    sport_subscriber = ChannelSubscriber(
        "rt/lf/sportmodestate", SportModeState_)
    low_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    sport_subscriber.Init(observer.on_sport, 1)
    low_subscriber.Init(observer.on_low, 1)
    try:
        while not stop.wait(0.5):
            pass
    except Exception as exc:
        recorder.record_event(
            "safety_observer_process_error", {"error": str(exc)[:512]})
        raise
    finally:
        sport_subscriber.Close()
        low_subscriber.Close()
        recorder.record_event("safety_observer_process_stop")


if __name__ == "__main__":
    main()
