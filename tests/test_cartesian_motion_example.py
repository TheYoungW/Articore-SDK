from __future__ import annotations

from types import SimpleNamespace

import pytest

from arx_d_can import CartesianMotionState
from arx_d_can.examples.control import example_07_cartesian_circular as circular
from arx_d_can.examples.control import example_07_cartesian_linear as linear
from arx_d_can.examples.control import example_07_cartesian_ptp as ptp


def _status(state: CartesianMotionState, progress: float):
    return SimpleNamespace(
        state=state,
        progress=progress,
        error=None,
    )


def test_ptp_example_submits_mirrored_dual_arm_targets(monkeypatch) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            calls.append(("create", control_mode))

        def connect(self) -> None:
            calls.append(("connect",))

        def enable(self) -> None:
            calls.append(("enable",))

        def move_poses(self, **kwargs) -> None:
            calls.append(("move_poses", kwargs))

        def disconnect(self) -> None:
            calls.append(("disconnect",))

    monkeypatch.setattr(ptp, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr(
        ptp.time,
        "sleep",
        lambda _seconds: (_ for _ in ()).throw(KeyboardInterrupt),
    )
    args = ptp.build_parser().parse_args([])

    ptp.main(args)

    assert calls == [
        ("create", "pv"),
        ("connect",),
        ("enable",),
        (
            "move_poses",
            {
                "left_target_pose": ptp.DEFAULT_LEFT_TARGET_POSE,
                "right_target_pose": ptp.DEFAULT_RIGHT_TARGET_POSE,
                "speed_percent": 50.0,
            },
        ),
        ("disconnect",),
    ]
    assert ptp.DEFAULT_LEFT_TARGET_POSE[1] > 0.0
    assert ptp.DEFAULT_RIGHT_TARGET_POSE[1] < 0.0


def test_ptp_example_defaults_to_fifty_and_accepts_zero_speed() -> None:
    parser = ptp.build_parser()

    assert parser.parse_args([]).speed == pytest.approx(50.0)
    assert parser.parse_args(["--speed", "0"]).speed == pytest.approx(0.0)
    with pytest.raises(SystemExit):
        parser.parse_args(["--speed", "100.1"])


@pytest.mark.parametrize(
    ("side", "start", "end", "expected_y_delta"),
    (
        (
            "left",
            linear.DEFAULT_LEFT_START_POSE,
            linear.DEFAULT_LEFT_END_POSE,
            0.1,
        ),
        (
            "right",
            linear.DEFAULT_RIGHT_START_POSE,
            linear.DEFAULT_RIGHT_END_POSE,
            -0.1,
        ),
    ),
)
def test_linear_example_uses_mirrored_defaults_and_waits_for_completed(
    monkeypatch, side, start, end, expected_y_delta,
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

    monkeypatch.setattr(linear, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr(linear.time, "sleep", lambda _seconds: None)
    args = linear.build_parser().parse_args([
        "--side", side,
        "--speed", "20",
    ])

    linear.main(args)

    assert calls.count(("status",)) == 2
    assert calls[-1] == ("disconnect",)
    assert calls[3] == (
        "move_linear",
        {
            "side": side,
            "start_pose": start,
            "end_pose": end,
            "speed_percent": 20.0,
        },
    )
    assert args.end[1] - args.start[1] == pytest.approx(expected_y_delta)


@pytest.mark.parametrize(
    ("side", "start", "via", "end", "expected_y_delta"),
    (
        (
            "left",
            circular.DEFAULT_LEFT_START_POSE,
            circular.DEFAULT_LEFT_VIA_POSE,
            circular.DEFAULT_LEFT_END_POSE,
            0.06,
        ),
        (
            "right",
            circular.DEFAULT_RIGHT_START_POSE,
            circular.DEFAULT_RIGHT_VIA_POSE,
            circular.DEFAULT_RIGHT_END_POSE,
            -0.06,
        ),
    ),
)
def test_circular_example_uses_mirrored_yz_semicircles(
    monkeypatch, side, start, via, end, expected_y_delta,
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

    monkeypatch.setattr(circular, "ArxDCanDualArm", FakeRobot)
    args = circular.build_parser().parse_args([
        "--side", side,
        "--speed", "15",
    ])

    circular.main(args)

    assert calls == [
        (
            "move_circular",
            {
                "side": side,
                "start_pose": start,
                "via_pose": via,
                "end_pose": end,
                "speed_percent": 15.0,
            },
        )
    ]
    assert args.via[1] - args.start[1] == pytest.approx(expected_y_delta)
    assert args.via[2] - args.start[2] == pytest.approx(0.06)
    assert args.end[2] - args.via[2] == pytest.approx(0.06)
