from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from arx_d_can import MotionState
from arx_d_can.examples.control import example_08_joint_trajectory as trajectory


def _status(state: MotionState, progress: float):
    return SimpleNamespace(state=state, progress=progress, error=None)


def _dual_arm_state(left: tuple[float, ...], right: tuple[float, ...]):
    return SimpleNamespace(
        left=SimpleNamespace(arm=SimpleNamespace(positions=left)),
        right=SimpleNamespace(arm=SimpleNamespace(positions=right)),
    )


def test_example_calls_native_move_joint_trajectory(monkeypatch) -> None:
    calls: list[tuple] = []
    current_left = tuple(0.01 * index for index in range(7))
    current_right = tuple(-0.01 * index for index in range(7))

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            calls.append(("create", control_mode))

        def connect(self) -> None:
            calls.append(("connect",))

        def enable(self) -> None:
            calls.append(("enable",))

        def read_state(self):
            calls.append(("read_state",))
            return _dual_arm_state(current_left, current_right)

        def move_joint_trajectory(self, **kwargs) -> int:
            calls.append(("move_joint_trajectory", kwargs))
            return 41

        def get_motion_status(self, motion_id: int):
            calls.append(("get_motion_status", motion_id))
            return _status(MotionState.COMPLETED, 1.0)

        def disable(self) -> None:
            calls.append(("disable",))

        def disconnect(self) -> None:
            calls.append(("disconnect",))

    monkeypatch.setattr(trajectory, "ArxDCanDualArm", FakeRobot)
    def raise_eof(_prompt: str) -> str:
        raise EOFError

    monkeypatch.setattr("builtins.input", raise_eof)
    args = trajectory.build_parser().parse_args([
        "--left", "0,0,0,90,0,0,0",
        "--right", "0,0,0,90,0,0,0",
        "--duration", "4",
    ])
    assert not hasattr(args, "pv_velocity_limit")

    trajectory.main(args)

    submission = next(call[1] for call in calls if call[0] == "move_joint_trajectory")
    assert submission == {
        "timestamps": [0.0, 4.0],
        "left_positions": [
            current_left,
            pytest.approx((0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0)),
        ],
        "right_positions": [
            current_right,
            pytest.approx((0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0)),
        ],
    }
    assert not any(call[0] == "set_pose" for call in calls)
    assert calls[-2:] == [("disable",), ("disconnect",)]


def test_mit_example_forwards_explicit_gains(monkeypatch) -> None:
    captured: dict[str, object] = {}

    class FakeRobot:
        def __init__(self, *, control_mode: str) -> None:
            captured["mode"] = control_mode

        def connect(self) -> None:
            pass

        def enable(self) -> None:
            pass

        def read_state(self):
            return _dual_arm_state((0.0,) * 7, (0.0,) * 7)

        def move_joint_trajectory(self, **kwargs) -> int:
            captured["submission"] = kwargs
            return 42

        def get_motion_status(self, motion_id: int):
            assert motion_id == 42
            return _status(MotionState.COMPLETED, 1.0)

        def disable(self) -> None:
            pass

        def disconnect(self) -> None:
            pass

    monkeypatch.setattr(trajectory, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr("builtins.input", lambda _prompt: "")

    trajectory.main(trajectory.build_parser().parse_args(["--mode", "mit"]))

    submission = captured["submission"]
    assert captured["mode"] == "mit"
    assert submission["kp"] == trajectory.DEFAULT_MIT_KP
    assert submission["kd"] == trajectory.DEFAULT_MIT_KD
    assert submission["feedforward_torque"] == trajectory.DEFAULT_MIT_FEEDFORWARD_TORQUE
    assert "pv_velocity_limits" not in submission
