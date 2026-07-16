import threading
import time

import pytest

from go2w_bridge.motion_types import Effect


class FakeSport:
    def __init__(self, calls, codes=None):
        self.calls = calls
        self.codes = dict(codes or {})

    def Init(self):
        self.calls.append(("Init", ()))

    def Move(self, vx, vy, vyaw):
        self.calls.append(("Move", (vx, vy, vyaw)))
        return self.codes.get("Move", 0)

    def StandUp(self):
        self.calls.append(("StandUp", ()))
        return self.codes.get("StandUp", 0)

    def BalanceStand(self):
        self.calls.append(("BalanceStand", ()))
        return self.codes.get("BalanceStand", 0)

    def StopMove(self):
        self.calls.append(("StopMove", ()))
        return self.codes.get("StopMove", 0)


class FakeSwitcher:
    def __init__(self, calls, mode="ai-w"):
        self.calls = calls
        self.mode = mode

    def Init(self):
        self.calls.append(("SwitcherInit", ()))

    def CheckMode(self):
        self.calls.append(("CheckMode", ()))
        return 0, {"name": self.mode}

    def SelectMode(self, _name):
        raise AssertionError("SelectMode must never be called automatically")


def test_adapter_uses_one_client_and_preserves_effect_order():
    from go2w_bridge.unitree_sport_adapter import UnitreeSportAdapter

    calls = []
    adapter = UnitreeSportAdapter(FakeSport(calls), FakeSwitcher(calls))
    adapter.execute(Effect("MoveZero", sequence=1))
    adapter.execute(Effect("StandUp", sequence=2))

    assert calls == [
        ("Move", (0.0, 0.0, 0.0)),
        ("StandUp", ()),
    ]


def test_initialize_zeros_before_and_after_lease_settle_then_checks_mode():
    from go2w_bridge.unitree_sport_adapter import UnitreeSportAdapter

    calls = []
    sleeps = []
    adapter = UnitreeSportAdapter(
        FakeSport(calls), FakeSwitcher(calls),
        sleep=lambda seconds: sleeps.append(seconds),
        lease_settle_seconds=0.25,
    )

    result = adapter.initialize()

    assert result.code == 0
    assert result.motion_service == "ai-w"
    assert sleeps == [0.25]
    assert calls == [
        ("Init", ()),
        ("Move", (0.0, 0.0, 0.0)),
        ("Move", (0.0, 0.0, 0.0)),
        ("Move", (0.0, 0.0, 0.0)),
        ("Move", (0.0, 0.0, 0.0)),
        ("SwitcherInit", ()),
        ("CheckMode", ()),
    ]


def test_move_receipt_is_transport_only_even_when_code_is_zero():
    from go2w_bridge.unitree_sport_adapter import UnitreeSportAdapter

    adapter = UnitreeSportAdapter(FakeSport([]), FakeSwitcher([]))
    receipt = adapter.execute(Effect(
        "Move", sequence=7, arguments=(0.2, 0.0, -0.1)))

    assert receipt.transport_ok is True
    assert receipt.physical_confirmed is False
    assert receipt.sequence == 7


def test_adapter_preserves_sdk_failure_code():
    from go2w_bridge.unitree_sport_adapter import UnitreeSportAdapter

    adapter = UnitreeSportAdapter(
        FakeSport([], codes={"BalanceStand": 3104}), FakeSwitcher([]))
    receipt = adapter.execute(Effect("BalanceStand", sequence=9))

    assert receipt.code == 3104
    assert receipt.transport_ok is False


def test_unknown_effect_is_rejected_without_sdk_call():
    from go2w_bridge.unitree_sport_adapter import UnitreeSportAdapter

    calls = []
    adapter = UnitreeSportAdapter(FakeSport(calls), FakeSwitcher(calls))

    with pytest.raises(ValueError, match="unsupported SDK effect"):
        adapter.execute(Effect("Backflip", sequence=1))
    assert calls == []


@pytest.mark.parametrize("operation", ["Damp", "RecoveryStand", "StandDown"])
def test_autonomous_adapter_rejects_support_changing_sdk_operations(operation):
    from go2w_bridge.unitree_sport_adapter import UnitreeSportAdapter

    calls = []
    sport = FakeSport(calls)
    setattr(sport, operation, lambda: calls.append((operation, ())))
    adapter = UnitreeSportAdapter(sport, FakeSwitcher(calls))

    with pytest.raises(ValueError, match="unsupported SDK effect"):
        adapter.execute(Effect(operation, sequence=1))
    assert calls == []


def test_execute_calls_do_not_overlap_between_threads():
    from go2w_bridge.unitree_sport_adapter import UnitreeSportAdapter

    active = 0
    overlap = []
    guard = threading.Lock()

    class SlowSport(FakeSport):
        def Move(self, vx, vy, vyaw):
            nonlocal active
            with guard:
                active += 1
                overlap.append(active)
            time.sleep(0.01)
            with guard:
                active -= 1
            return 0

    adapter = UnitreeSportAdapter(SlowSport([]), FakeSwitcher([]))
    threads = [
        threading.Thread(
            target=adapter.execute,
            args=(Effect("MoveZero", sequence=index),),
        )
        for index in range(1, 5)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert overlap == [1, 1, 1, 1]
