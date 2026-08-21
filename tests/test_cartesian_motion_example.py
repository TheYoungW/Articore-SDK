from __future__ import annotations

from types import SimpleNamespace

from arx_d_can import CartesianMotionState
from arx_d_can.examples.control import example_07_cartesian_motion as example


def _status(state: CartesianMotionState, progress: float):
    return SimpleNamespace(
        state=state,
        progress=progress,
        error=None,
    )


def test_example_waits_for_completed_after_progress_reaches_one(
    monkeypatch,
) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        statuses = iter((
            _status(CartesianMotionState.RUNNING, 1.0),
            _status(CartesianMotionState.COMPLETED, 1.0),
        ))

        def __init__(self, *, control_mode: str) -> None:
            calls.append(("create", control_mode))

        def connect(self) -> None:
            calls.append(("connect",))

        def enable(self) -> None:
            calls.append(("enable",))

        def move_linear(self, **kwargs) -> int:
            calls.append(("move_linear", kwargs))
            return 7

        @property
        def cartesian_motion_status(self):
            calls.append(("status",))
            return next(self.statuses)

        def disconnect(self) -> None:
            calls.append(("disconnect",))

    monkeypatch.setattr(example, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr(example.time, "sleep", lambda _seconds: None)
    args = example.build_parser().parse_args([
        "--side", "right",
        "--motion", "linear",
        "--target", "0.3,0.2,0.4,0,0,0",
        "--speed", "20",
    ])

    example.main(args)

    assert calls.count(("status",)) == 2
    assert calls[-1] == ("disconnect",)
    assert calls[3] == (
        "move_linear",
        {
            "side": "right",
            "target_pose": (0.3, 0.2, 0.4, 0.0, 0.0, 0.0),
            "speed_percent": 20.0,
        },
    )


def test_circular_example_forwards_three_poses_without_second_side_call(
    monkeypatch,
) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            pass

        def connect(self) -> None:
            pass

        def enable(self) -> None:
            pass

        def move_circular(self, **kwargs) -> int:
            calls.append(("move_circular", kwargs))
            return 8

        @property
        def cartesian_motion_status(self):
            return _status(CartesianMotionState.COMPLETED, 1.0)

        def disconnect(self) -> None:
            pass

    monkeypatch.setattr(example, "ArxDCanDualArm", FakeRobot)
    args = example.build_parser().parse_args([
        "--side", "left", "--motion", "circular",
        "--via", "0.25,0.15,0.35,0,0,0",
        "--target", "0.3,0.1,0.3,0,0,0",
        "--speed", "15",
    ])

    example.main(args)

    assert len(calls) == 1
    assert calls[0][0] == "move_circular"
    assert calls[0][1]["side"] == "left"
    assert calls[0][1]["speed_percent"] == 15.0
    assert "start_pose" not in calls[0][1]
