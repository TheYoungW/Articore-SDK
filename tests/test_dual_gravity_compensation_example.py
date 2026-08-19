from __future__ import annotations

from types import SimpleNamespace

from arx_d_can import GravityCompensationPhase
from arx_d_can.examples.example_15_gravity_compensation import (
    _stop_and_close,
)


class _Robot:
    def __init__(self, phase: GravityCompensationPhase) -> None:
        self.connected = True
        self.enabled = True
        self.phase = phase
        self.calls: list[str] = []

    @property
    def gravity_compensation_status(self) -> SimpleNamespace:
        return SimpleNamespace(phase=self.phase)

    def stop_gravity_compensation(self) -> None:
        self.calls.append("stop_gravity_compensation")
        self.phase = GravityCompensationPhase.INACTIVE

    def disable(self) -> None:
        self.calls.append("disable")
        self.enabled = False

    def disconnect(self) -> None:
        self.calls.append("close")
        self.connected = False


def test_stop_and_close_exits_gravity_before_disabling() -> None:
    robot = _Robot(GravityCompensationPhase.ACTIVE)

    _stop_and_close(robot)

    assert robot.calls == ["stop_gravity_compensation", "disable", "close"]
    assert not robot.enabled
    assert not robot.connected


def test_stop_and_close_skips_inactive_gravity() -> None:
    robot = _Robot(GravityCompensationPhase.INACTIVE)

    _stop_and_close(robot)

    assert robot.calls == ["disable", "close"]
