"""Stable NX process that owns the Unitree Sport lease across app restarts."""

from __future__ import annotations

import os
import signal
import threading

try:
    from .safety_event_recorder import SafetyEventRecorder
    from .sport_gateway_server import SportGatewayServer
except ImportError:  # Direct-file compatibility deployment on the NX.
    from safety_event_recorder import SafetyEventRecorder
    from sport_gateway_server import SportGatewayServer


def main() -> None:
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
        MotionSwitcherClient,
    )
    from unitree_sdk2py.core.channel import ChannelFactory
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    interface = os.environ.get("DOG_INTERFACE", "enxc8a362616c4c")
    socket_path = os.environ.get(
        "GO2W_SPORT_GATEWAY_SOCKET",
        "/run/go2w-sport-gateway/sport.sock",
    )
    event_log = os.environ.get(
        "GO2W_GATEWAY_EVENT_LOG",
        "/home/nx/go2w/safety-events/gateway.jsonl",
    )
    timeout = float(os.environ.get("GO2W_SDK_CALL_TIMEOUT", "0.8"))
    recorder = SafetyEventRecorder(event_log)
    recorder.record_event("gateway_process_start", {"interface": interface})
    stop = threading.Event()

    def request_stop(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    factory = ChannelFactory()
    factory.Init(0, interface)
    sport = SportClient(enableLease=True)
    sport.SetTimeout(timeout)
    sport.Init()
    sport.WaitLeaseApplied()
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(timeout)
    switcher.Init()

    server = SportGatewayServer(
        sport,
        switcher,
        socket_path=socket_path,
        command_timeout=0.25,
        zero_period=0.05,
        recorder=recorder,
    )
    try:
        server.start()
        while not stop.wait(0.5):
            pass
    except Exception as exc:
        recorder.record_event(
            "gateway_process_error", {"error": str(exc)[:512]})
        raise
    finally:
        server.close()
        recorder.record_event("gateway_process_stop")


if __name__ == "__main__":
    main()
