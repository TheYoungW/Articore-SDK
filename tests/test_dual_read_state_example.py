from __future__ import annotations

from types import SimpleNamespace

import pytest

from arx_d_can.examples.diagnostics import example_01_read_state as example


def _state():
    joint = SimpleNamespace(
        positions=(0.0,),
        velocities=(0.0,),
        torques=(0.0,),
    )
    side = SimpleNamespace(
        positions=(0.0,),
        arm=joint,
        gripper=SimpleNamespace(opening=500.0),
    )
    return SimpleNamespace(left=side, right=side)


def test_parser_selects_once_or_continuous_mode() -> None:
    parser = example.build_parser()

    assert parser.parse_args([]).mode == "once"
    assert parser.parse_args(["--mode", "continuous"]).mode == "continuous"
    assert parser.parse_args([]).display_hz == 10.0
    with pytest.raises(SystemExit):
        parser.parse_args(["--display-hz", "101"])


def test_once_mode_reads_one_feedback_sample(monkeypatch) -> None:
    calls: list[object] = []

    class Robot:
        def connect(self) -> None:
            calls.append("connect")

        def read_state(self):
            calls.append("read")
            return _state()

        def disconnect(self) -> None:
            calls.append("close")

    monkeypatch.setattr(example, "ArxDCanDualArm", Robot)

    example.main(example.build_parser().parse_args([]))

    assert calls == ["connect", "read", "close"]


def test_continuous_mode_uses_100_hz_until_interrupted(monkeypatch) -> None:
    reads = 0
    sleeps: list[float] = []
    displays = 0
    now = 0.0

    class Robot:
        def connect(self) -> None:
            pass

        def read_state(self):
            nonlocal reads
            reads += 1
            if reads == 3:
                raise KeyboardInterrupt
            return _state()

        def disconnect(self) -> None:
            pass

    monkeypatch.setattr(example, "ArxDCanDualArm", Robot)
    monkeypatch.setattr(example.time, "perf_counter", lambda: now)

    def sleep(seconds: float) -> None:
        nonlocal now
        sleeps.append(seconds)
        now += seconds

    def display(_state) -> None:
        nonlocal displays
        displays += 1

    monkeypatch.setattr(example.time, "sleep", sleep)
    monkeypatch.setattr(example, "_print_state", display)

    args = example.build_parser().parse_args(["--mode", "continuous"])
    example.main(args)

    assert reads == 3
    assert sleeps == [0.01, 0.01]
    assert displays == 1
