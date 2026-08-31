from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from arx_d_can import MotionState
from arx_d_can.examples.control import example_10_cartesian_circular_trajectory as circular
from arx_d_can.examples.control import example_09_cartesian_linear_trajectory as linear
from arx_d_can.examples.control import example_11_cartesian_orientation_ptp as orientation
from arx_d_can.examples.control import example_07_cartesian_ptp as ptp


def _status(
    state: MotionState,
    progress: float,
    error: str | None = None,
):
    return SimpleNamespace(
        state=state,
        progress=progress,
        error=error,
    )


def _health():
    return SimpleNamespace(
        last_operation_error="global operation error",
        safety_reason="global safety reason",
        fault_reason="global fault reason",
    )


def test_ptp_example_submits_mirrored_dual_arm_targets(monkeypatch) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            calls.append(("create", control_mode))

        def connect(self) -> None:
            calls.append(("connect",))

        def solve_ik(self, **kwargs):
            calls.append(("solve_ik", kwargs))
            return (1.0,) * 7, (2.0,) * 7

        def enable(self) -> None:
            calls.append(("enable",))

        def set_joint_pv(self, **kwargs) -> None:
            calls.append(("set_joint_pv", kwargs))

        def disable(self) -> None:
            calls.append(("disable",))

        def disconnect(self) -> None:
            calls.append(("disconnect",))

    monkeypatch.setattr(ptp, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr("builtins.input", lambda _prompt: "")
    args = ptp.build_parser().parse_args([])

    ptp.main(args)

    assert calls == [
        ("create", "pv"),
        ("connect",),
        (
            "solve_ik",
            {
                "left_target_pose": ptp.DEFAULT_LEFT_TARGET_POSE,
                "right_target_pose": ptp.DEFAULT_RIGHT_TARGET_POSE,
            },
        ),
        ("enable",),
        (
            "set_joint_pv",
            {
                "left": (1.0,) * 7,
                "right": (2.0,) * 7,
                "velocity": 50.0,
            },
        ),
        ("disable",),
        ("disconnect",),
    ]
    assert ptp.DEFAULT_LEFT_TARGET_POSE[1] > 0.0
    assert ptp.DEFAULT_RIGHT_TARGET_POSE[1] < 0.0


def test_ptp_example_defaults_to_fifty_and_requires_positive_speed() -> None:
    parser = ptp.build_parser()

    assert parser.parse_args([]).speed == pytest.approx(50.0)
    with pytest.raises(SystemExit):
        parser.parse_args(["--speed", "0"])
    with pytest.raises(SystemExit):
        parser.parse_args(["--speed", "100.1"])


def test_orientation_ptp_example_sweeps_both_arms_and_all_axes(monkeypatch) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            calls.append(("create", control_mode))

        def connect(self) -> None:
            calls.append(("connect",))

        def enable(self) -> None:
            calls.append(("enable",))

        def disable(self) -> None:
            calls.append(("disable",))

        def disconnect(self) -> None:
            calls.append(("disconnect",))

    def record_move(_robot, **kwargs) -> None:
        calls.append(("move", kwargs))

    monkeypatch.setattr(orientation, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr(orientation, "_move_and_wait", record_move)
    monkeypatch.setattr("builtins.input", lambda _prompt: "")

    orientation.main(orientation.build_parser().parse_args([]))

    move_calls = [call[1] for call in calls if call[0] == "move"]
    assert len(move_calls) == 10
    assert move_calls[0]["left"] == orientation.BASE_LEFT_POSE
    assert move_calls[0]["right"] == orientation.BASE_RIGHT_POSE
    assert [move_calls[index]["label"] for index in (1, 4, 7)] == [
        "Pitch 负方向端点",
        "Roll 负方向端点",
        "Yaw 负方向端点",
    ]
    assert all(call["speed"] == 50.0 for call in move_calls)
    assert calls[-2:] == [("disable",), ("disconnect",)]
    for call in move_calls:
        assert call["left"][1] > 0.0
        assert call["right"][1] < 0.0


def test_orientation_error_handles_equivalent_rpy_at_singularity() -> None:
    first = (0.0, 0.0, 0.0, 0.0, -math.pi / 2.0, 0.0)
    equivalent = (
        0.0,
        0.0,
        0.0,
        math.pi,
        -math.pi / 2.0,
        math.pi,
    )

    assert orientation._orientation_error(first, equivalent) == pytest.approx(0.0)
    assert orientation.build_parser().parse_args([]).speed == pytest.approx(50.0)
    with pytest.raises(SystemExit):
        orientation.build_parser().parse_args(["--speed", "0"])


@pytest.mark.parametrize(
    ("side", "center", "outward_sign"),
    (
        (
            "left",
            linear.DEFAULT_LEFT_CENTER_POSE,
            1.0,
        ),
        (
            "right",
            linear.DEFAULT_RIGHT_CENTER_POSE,
            -1.0,
        ),
    ),
)
def test_linear_example_uses_one_blended_path_for_a_mirrored_equilateral_triangle(
    monkeypatch, side, center, outward_sign,
) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            calls.append(("create", control_mode))

        def connect(self) -> None:
            calls.append(("connect",))

        def enable(self) -> None:
            calls.append(("enable",))

        def move_linear_trajectory(self, **kwargs) -> int:
            calls.append(("move_linear_trajectory", kwargs))
            return 7

        def get_motion_status(self, motion_id: int):
            assert motion_id == 7
            return _status(MotionState.COMPLETED, 1.0)

        def cancel_all_motions(self) -> None:
            calls.append(("cancel",))

        def disconnect(self) -> None:
            calls.append(("disconnect",))

    monkeypatch.setattr(linear, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr(linear.time, "sleep", lambda _seconds: None)
    args = linear.build_parser().parse_args([
        "--side", side,
        "--duration", "20",
    ])

    linear.main(args)

    assert calls[-1] == ("disconnect",)
    assert calls[0] == ("create", "pv")
    move_calls = [call[1] for call in calls if call[0] == "move_linear_trajectory"]
    assert len(move_calls) == 1
    assert move_calls[0]["duration_s"] == 20
    path = move_calls[0]["poses"]
    assert len(path) == 4
    assert path[-1] == path[0]
    vertices = path[:-1]
    for first, second in zip(vertices, vertices[1:] + vertices[:1]):
        distance = math.dist(first[:3], second[:3])
        assert distance == pytest.approx(linear.TRIANGLE_SIDE_M)
    centroid = tuple(
        sum(vertex[index] for vertex in vertices) / 3.0
        for index in range(3)
    )
    assert centroid == pytest.approx(center[:3])
    assert (vertices[0][1] - center[1]) * outward_sign > 0.0


def test_linear_example_reports_fault_before_cancelled_queue_tail(
    monkeypatch, capsys,
) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            assert control_mode == "pv"

        def connect(self) -> None:
            pass

        def enable(self) -> None:
            pass

        def move_linear_trajectory(self, **_kwargs) -> int:
            motion_id = 1 + len(calls)
            calls.append(("move", motion_id))
            return motion_id

        def get_motion_status(self, motion_id: int):
            if motion_id == 1:
                return _status(
                    MotionState.FAULT, 0.657,
                    "first linear segment missed its deadline",
                )
            return _status(MotionState.CANCELLED, 0.0)

        def get_health(self):
            return _health()

        def cancel_all_motions(self) -> None:
            calls.append(("cancel",))

        def disconnect(self) -> None:
            calls.append(("disconnect",))

    monkeypatch.setattr(linear, "ArxDCanDualArm", FakeRobot)
    args = linear.build_parser().parse_args([
        "--side", "left",
        "--duration", "3",
    ])

    with pytest.raises(
        RuntimeError,
        match="motion_id=1.*first linear segment missed its deadline",
    ):
        linear.main(args)

    assert "运动已取消" not in capsys.readouterr().out
    assert calls[-1] == ("disconnect",)


@pytest.mark.parametrize(
    ("side", "start", "via", "end", "return_via", "expected_y_delta"),
    (
        (
            "left",
            circular.DEFAULT_LEFT_START_POSE,
            circular.DEFAULT_LEFT_VIA_POSE,
            circular.DEFAULT_LEFT_END_POSE,
            circular.DEFAULT_LEFT_RETURN_VIA_POSE,
            circular.CIRCLE_RADIUS_M,
        ),
        (
            "right",
            circular.DEFAULT_RIGHT_START_POSE,
            circular.DEFAULT_RIGHT_VIA_POSE,
            circular.DEFAULT_RIGHT_END_POSE,
            circular.DEFAULT_RIGHT_RETURN_VIA_POSE,
            -circular.CIRCLE_RADIUS_M,
        ),
    ),
)
def test_circular_example_uses_two_mirrored_yz_semicircles_for_a_full_circle(
    monkeypatch, side, start, via, end, return_via, expected_y_delta,
) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            pass

        def connect(self) -> None:
            pass

        def enable(self) -> None:
            pass

        def move_circular_trajectory(self, **kwargs) -> int:
            calls.append(("move_circular_trajectory", kwargs))
            return 8 + len(calls) - 1

        def get_motion_status(self, motion_id: int):
            assert motion_id in (8, 9)
            return _status(MotionState.COMPLETED, 1.0)

        def cancel_all_motions(self) -> None:
            calls.append(("cancel",))

        def disconnect(self) -> None:
            pass

    monkeypatch.setattr(circular, "ArxDCanDualArm", FakeRobot)
    args = circular.build_parser().parse_args([
        "--side", side,
        "--duration", "15",
    ])

    circular.main(args)

    assert calls == [
        (
            "move_circular_trajectory",
            {
                "side": side,
                "start_pose": start,
                "via_pose": via,
                "end_pose": end,
                "duration_s": 15.0,
            },
        ),
        (
            "move_circular_trajectory",
            {
                "side": side,
                "start_pose": end,
                "via_pose": return_via,
                "end_pose": start,
                "duration_s": 15.0,
            },
        ),
    ]
    assert args.via[1] - args.start[1] == pytest.approx(expected_y_delta)
    assert args.via[2] - args.start[2] == pytest.approx(
        circular.CIRCLE_RADIUS_M
    )
    assert args.end[2] - args.via[2] == pytest.approx(
        circular.CIRCLE_RADIUS_M
    )
    assert args.return_via[1] - args.start[1] == pytest.approx(-expected_y_delta)
    assert args.return_via[2] - args.start[2] == pytest.approx(
        circular.CIRCLE_RADIUS_M
    )


def test_circular_example_reports_fault_before_cancelled_queue_tail(
    monkeypatch, capsys,
) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            assert control_mode == "pv"

        def connect(self) -> None:
            pass

        def enable(self) -> None:
            pass

        def move_circular_trajectory(self, **_kwargs) -> int:
            motion_id = 11 + len(calls)
            calls.append(("move", motion_id))
            return motion_id

        def get_motion_status(self, motion_id: int):
            if motion_id == 11:
                return _status(
                    MotionState.FAULT, 0.4,
                    "first circular segment failed arrival",
                )
            return _status(MotionState.CANCELLED, 0.0)

        def get_health(self):
            return _health()

        def cancel_all_motions(self) -> None:
            calls.append(("cancel",))

        def disconnect(self) -> None:
            calls.append(("disconnect",))

    monkeypatch.setattr(circular, "ArxDCanDualArm", FakeRobot)
    args = circular.build_parser().parse_args([
        "--side", "left",
        "--duration", "3",
    ])

    with pytest.raises(
        RuntimeError,
        match="motion_id=11.*first circular segment failed arrival",
    ):
        circular.main(args)

    assert "运动已取消" not in capsys.readouterr().out
    assert calls[-1] == ("disconnect",)
