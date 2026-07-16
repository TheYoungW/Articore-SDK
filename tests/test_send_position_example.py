import pytest

from arx_d_can.examples import example_04_send_position as example


class FakeArm:
    def __init__(self, *, interrupt_after: int | None = None):
        self.targets = []
        self.interrupt_after = interrupt_after

    def send_joint_positions(self, target):
        self.targets.append(tuple(target))
        if self.interrupt_after is not None and len(self.targets) >= self.interrupt_after:
            raise KeyboardInterrupt


def test_zero_hold_seconds_keeps_refreshing_until_interrupted(monkeypatch):
    arm = FakeArm(interrupt_after=3)
    monkeypatch.setattr(example.time, "sleep", lambda _: None)

    with pytest.raises(KeyboardInterrupt):
        example.hold_target(arm, (1.0,) * 6, seconds=0.0, hz=100.0)

    assert arm.targets == [(1.0,) * 6] * 3


def test_positive_hold_seconds_stops_after_deadline(monkeypatch):
    arm = FakeArm()
    clock = {"now": 0.0}

    monkeypatch.setattr(example.time, "monotonic", lambda: clock["now"])
    monkeypatch.setattr(
        example.time,
        "sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )

    example.hold_target(arm, (2.0,) * 6, seconds=0.025, hz=100.0)

    assert arm.targets == [(2.0,) * 6] * 3
