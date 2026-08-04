#!/usr/bin/env python3
"""Print Go2W sport-mode feedback and remote-controller state without commanding motion."""

import json
import os
import struct
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_._SportModeState_ import SportModeState_


lock = threading.Lock()
sport = None
remote = None
DOG_INTERFACE = os.environ.get("DOG_INTERFACE", "enxc8a362616c4c")


def on_sport(message):
    global sport
    with lock:
        sport = {
            "error_code": int(message.error_code),
            "mode": int(message.mode),
            "gait_type": int(message.gait_type),
            "progress": round(float(message.progress), 4),
            "velocity": [round(float(v), 4) for v in message.velocity],
            "yaw_speed": round(float(message.yaw_speed), 4),
            "position": [round(float(v), 4) for v in message.position],
        }


def on_low(message):
    global remote
    data = bytes(message.wireless_remote)
    button_names = (
        "left", "down", "right", "up", "Y", "X", "B", "A",
        "LT", "RT", "back", "start", "LB", "RB",
    )
    bits = [int(bit) for bit in f"{data[3]:08b}"] + [
        int(bit) for bit in f"{data[2]:08b}"[2:]
    ]
    with lock:
        remote = {
            "buttons": {
                name: bool(value) for name, value in zip(button_names, bits)
            },
            "axes": {
                "lx": round(struct.unpack("f", data[4:8])[0], 4),
                "rx": round(struct.unpack("f", data[8:12])[0], 4),
                "ry": round(struct.unpack("f", data[12:16])[0], 4),
                "ly": round(struct.unpack("f", data[20:24])[0], 4),
            },
            "wheel_dq": [
                round(float(message.motor_state[index].dq), 4)
                for index in (12, 13, 14, 15)
            ],
            "motor_mode": [int(motor.mode) for motor in message.motor_state],
            "motor_lost": [int(motor.lost) for motor in message.motor_state],
            "motor_temp": [int(motor.temperature) for motor in message.motor_state],
            "joint_q": [round(float(message.motor_state[index].q), 4) for index in range(12)],
            "imu_rpy": [round(float(value), 4) for value in message.imu_state.rpy],
            "foot_force": [int(value) for value in message.foot_force],
            "bms": {
                "status": int(message.bms_state.status),
                "soc": int(message.bms_state.soc),
                "current": int(message.bms_state.current),
            },
            "level_flag": int(message.level_flag),
            "bit_flag": int(message.bit_flag),
            "power_v": round(float(message.power_v), 3),
            "power_a": round(float(message.power_a), 3),
        }


factory = ChannelFactory()
factory.Init(0, DOG_INTERFACE)
switcher = MotionSwitcherClient()
switcher.SetTimeout(1.0)
switcher.Init()
mode_code, mode_data = switcher.CheckMode()
print(json.dumps({
    "interface": DOG_INTERFACE,
    "check_mode_code": mode_code,
    "check_mode": mode_data,
}), flush=True)
sport_subscriber = ChannelSubscriber("rt/lf/sportmodestate", SportModeState_)
low_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
sport_subscriber.Init(on_sport, 1)
low_subscriber.Init(on_low, 1)
print("READY", flush=True)
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    with lock:
        snapshot = {"sport": sport, "remote": remote}
    print(json.dumps(snapshot, separators=(",", ":")), flush=True)
    time.sleep(0.25)
