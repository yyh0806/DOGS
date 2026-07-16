#!/usr/bin/env python3
"""Print raw Go2W wheel motor velocities from rt/lowstate."""

import os
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_


lock = threading.Lock()
latest = None
DOG_INTERFACE = os.environ.get("DOG_INTERFACE", "enxc8a362616c4c")


def on_state(message):
    global latest
    motors = message.motor_state
    if len(motors) >= 16:
        with lock:
            latest = tuple(float(motors[index].dq) for index in (12, 13, 14, 15))


factory = ChannelFactory()
factory.Init(0, DOG_INTERFACE)
subscriber = ChannelSubscriber("rt/lowstate", LowState_)
subscriber.Init(on_state, 1)
print("READY", DOG_INTERFACE, flush=True)
deadline = time.monotonic() + 15.0
while time.monotonic() < deadline:
    with lock:
        values = latest
    if values is not None:
        print("DQ", *(round(value, 4) for value in values), flush=True)
    time.sleep(0.1)
