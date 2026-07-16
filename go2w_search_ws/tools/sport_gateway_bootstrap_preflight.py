#!/usr/bin/env python3
"""Read-only safety gate for the one-time direct-lease to gateway handoff."""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
from typing import Any, Mapping


def evaluate_snapshot(snapshot: Mapping[str, Any]) -> list[str]:
    """Return closed-form reasons why a lease handoff is not currently safe."""

    required = {
        "mode", "error_code", "wheel_dq", "roll", "pitch",
        "motor_lost", "battery_soc",
    }
    if not isinstance(snapshot, Mapping) or not required.issubset(snapshot):
        return ["incomplete_state"]
    try:
        mode = int(snapshot["mode"])
        error_code = int(snapshot["error_code"])
        wheel_dq = tuple(float(value) for value in snapshot["wheel_dq"])
        roll = float(snapshot["roll"])
        pitch = float(snapshot["pitch"])
        motor_lost = tuple(int(value) for value in snapshot["motor_lost"])
        battery_soc = float(snapshot["battery_soc"])
    except (TypeError, ValueError, OverflowError):
        return ["incomplete_state"]
    numeric = (*wheel_dq, roll, pitch, battery_soc)
    if (len(wheel_dq) != 4 or not motor_lost
            or not all(math.isfinite(value) for value in numeric)):
        return ["incomplete_state"]

    failures = []
    if mode != 6:
        failures.append("physical_mode_not_joint_lock")
    if error_code != 0:
        failures.append("robot_error")
    if max(abs(value) for value in wheel_dq) > 0.12:
        failures.append("wheels_not_stopped")
    if abs(roll) > 0.70 or abs(pitch) > 0.70:
        failures.append("attitude_unsafe")
    if battery_soc < 20.0:
        failures.append("battery_low")
    return failures


def observe_safe_state(
    interface: str,
    *,
    timeout: float,
    required_samples: int,
) -> tuple[dict[str, Any], list[str]]:
    """Observe consecutive raw DDS samples without constructing a SportClient."""

    from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_._SportModeState_ import (
        SportModeState_,
    )

    lock = threading.RLock()
    shared: dict[str, Any] = {"sport_generation": 0, "low_generation": 0}

    def on_sport(message: Any) -> None:
        with lock:
            shared.update({
                "mode": int(message.mode),
                "error_code": int(message.error_code),
                "sport_generation": int(shared["sport_generation"]) + 1,
            })

    def on_low(message: Any) -> None:
        motors = list(message.motor_state)
        rpy = list(message.imu_state.rpy)
        if len(motors) < 16 or len(rpy) < 2:
            return
        with lock:
            shared.update({
                "wheel_dq": [float(motors[index].dq)
                             for index in (12, 13, 14, 15)],
                "roll": float(rpy[0]),
                "pitch": float(rpy[1]),
                "motor_lost": [int(motor.lost) for motor in motors],
                "battery_soc": int(message.bms_state.soc),
                "low_generation": int(shared["low_generation"]) + 1,
            })

    factory = ChannelFactory()
    factory.Init(0, interface)
    sport_subscriber = ChannelSubscriber(
        "rt/lf/sportmodestate", SportModeState_)
    low_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    sport_subscriber.Init(on_sport, 1)
    low_subscriber.Init(on_low, 1)

    deadline = time.monotonic() + max(0.5, float(timeout))
    required_samples = max(2, int(required_samples))
    accepted_sport = 0
    accepted_low = 0
    consecutive = 0
    latest: dict[str, Any] = {}
    failures = ["incomplete_state"]
    while time.monotonic() < deadline:
        with lock:
            latest = dict(shared)
        failures = evaluate_snapshot(latest)
        sport_generation = int(latest.get("sport_generation", 0))
        low_generation = int(latest.get("low_generation", 0))
        if not failures:
            if (sport_generation > accepted_sport
                    and low_generation > accepted_low):
                consecutive += 1
                accepted_sport = sport_generation
                accepted_low = low_generation
                if consecutive >= required_samples:
                    break
        else:
            consecutive = 0
            accepted_sport = sport_generation
            accepted_low = low_generation
        time.sleep(0.05)
    else:
        if not failures:
            failures = ["insufficient_consecutive_samples"]

    latest.pop("sport_generation", None)
    latest.pop("low_generation", None)
    return latest, failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--interface",
        default=os.environ.get("DOG_INTERFACE", "enxc8a362616c4c"),
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--samples", type=int, default=3)
    args = parser.parse_args()
    snapshot, failures = observe_safe_state(
        args.interface,
        timeout=args.timeout,
        required_samples=args.samples,
    )
    print(json.dumps({
        "safe": not failures,
        "failures": failures,
        "snapshot": snapshot,
    }, ensure_ascii=False, separators=(",", ":")), flush=True)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
