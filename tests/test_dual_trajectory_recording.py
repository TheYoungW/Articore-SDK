from __future__ import annotations

import pytest

from arx_d_can import MotionState

from arx_d_can.service_tools.dual_trajectory_recording import (
    DEFAULT_MIT_FEEDFORWARD_TORQUES,
    DEFAULT_MIT_KD,
    DEFAULT_MIT_KP,
    DEFAULT_PV_VELOCITY_LIMITS,
    DualArmTrajectorySample,
    load_trajectory,
    record,
    replay,
    save_trajectory,
)


def test_dual_trajectory_round_trip(tmp_path) -> None:
    path = tmp_path / "dual.json"
    samples = [
        DualArmTrajectorySample(
            left_positions=(0.0, 0.1),
            right_positions=(0.2, 0.3),
            left_gripper=1000.0,
            right_gripper=500.0,
        ),
        DualArmTrajectorySample(
            left_positions=(0.4, 0.5),
            right_positions=(0.6, 0.7),
            left_gripper=900.0,
            right_gripper=None,
        ),
    ]
    save_trajectory(
        path,
        hz=100.0,
        timestamps=[0.0, 0.01],
        samples=samples,
        left_joint_names=("l1", "l2"),
        right_joint_names=("r1", "r2"),
    )

    timestamps, loaded = load_trajectory(
        path,
        expected_left_joint_names=("l1", "l2"),
        expected_right_joint_names=("r1", "r2"),
    )

    assert timestamps == [0.0, 0.01]
    assert loaded == samples


def test_dual_trajectory_rejects_wrong_product_joints(tmp_path) -> None:
    path = tmp_path / "dual.json"
    sample = DualArmTrajectorySample((0.0,), (0.0,), None, None)
    save_trajectory(
        path,
        hz=100.0,
        timestamps=[0.0],
        samples=[sample],
        left_joint_names=("l1",),
        right_joint_names=("r1",),
    )

    with pytest.raises(ValueError, match="left joints"):
        load_trajectory(
            path,
            expected_left_joint_names=("other",),
            expected_right_joint_names=("r1",),
        )


def test_dual_replay_submits_once_to_native_runtime(monkeypatch) -> None:
    commands: list[tuple] = []

    class Robot:
        has_grippers = True
        control_mode = "pv"

        def __init__(self) -> None:
            self.status_reads = 0

        def set_grippers(self, **kwargs) -> None:
            commands.append(("grippers", kwargs))

        def start_trajectory(self, **kwargs) -> int:
            commands.append(("trajectory", kwargs))
            return 7

        def get_motion_status(self, motion_id: int):
            assert motion_id == 7
            self.status_reads += 1
            state = (
                MotionState.RUNNING
                if self.status_reads == 1
                else MotionState.COMPLETED
            )
            return type("Status", (), {"state": state, "error": None})()

        def cancel_motion(self, motion_id: int) -> None:
            commands.append(("cancel", motion_id))

    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.sleep",
        lambda _seconds: None,
    )
    samples = [
        DualArmTrajectorySample((0.1,), (0.2,), 1000.0, 500.0),
        DualArmTrajectorySample((0.3,), (0.4,), 1000.0, 500.0),
    ]
    replay(Robot(), timestamps=[5.0, 5.35], samples=samples)

    assert commands[0] == (
        "grippers",
        {"left": 1000.0, "right": 500.0, "gripper_level": 5},
    )
    assert commands[1][0] == "trajectory"
    request = commands[1][1]
    assert request["timestamps"] == pytest.approx([0.0, 0.35])
    assert request["left_positions"] == [(0.1,), (0.3,)]
    assert request["right_positions"] == [(0.2,), (0.4,)]
    assert request["pv_velocity_limits"] == DEFAULT_PV_VELOCITY_LIMITS
    assert not any(command[0] == "arms" for command in commands)


def test_dual_mit_replay_submits_native_gains_once(monkeypatch) -> None:
    commands = []

    class Robot:
        has_grippers = False
        control_mode = "mit"

        def start_trajectory(self, **kwargs) -> int:
            commands.append(kwargs)
            return 8

        def get_motion_status(self, motion_id: int):
            assert motion_id == 8
            return type(
                "Status", (),
                {"state": MotionState.COMPLETED, "error": None},
            )()

    replay(
        Robot(),
        timestamps=[0.0, 1.0],
        samples=[
            DualArmTrajectorySample((0.1,), (-0.2,), None, None),
            DualArmTrajectorySample((0.2,), (-0.1,), None, None),
        ],
    )

    assert len(commands) == 1
    assert commands[0]["kp"] == DEFAULT_MIT_KP
    assert commands[0]["kd"] == DEFAULT_MIT_KD
    assert commands[0]["feedforward_torque"] == DEFAULT_MIT_FEEDFORWARD_TORQUES


def test_dual_replay_rejects_python_interpolation_and_dynamic_grippers() -> None:
    class Robot:
        has_grippers = True
        control_mode = "pv"

    samples = [
        DualArmTrajectorySample((0.1,), (0.2,), 1000.0, 500.0),
        DualArmTrajectorySample((0.3,), (0.4,), 900.0, 500.0),
    ]
    with pytest.raises(ValueError, match="only supports quintic"):
        replay(Robot(), timestamps=[0.0, 1.0], samples=samples, interpolation="linear")
    with pytest.raises(ValueError, match="time-varying gripper"):
        replay(Robot(), timestamps=[0.0, 1.0], samples=samples)


def test_dual_record_uses_cached_runtime_state_and_grippers(
    monkeypatch,
) -> None:
    now = 0.0

    def fake_sleep(seconds: float) -> None:
        nonlocal now
        now += seconds

    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.perf_counter",
        lambda: now,
    )
    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.sleep",
        fake_sleep,
    )

    class Robot:
        health = type(
            "Health",
            (),
            {"safe_holding": False, "fault_reason": None},
        )()

        def get_health(self):
            return self.health

        def read_cached_state(self):
            side = lambda opening: type(
                "Side",
                (),
                {
                    "arm": type("Arm", (), {"positions": (0.1,)})(),
                    "gripper": type("Gripper", (), {"opening": opening})(),
                },
            )()
            state = type(
                "State",
                (),
                {"left": side(1000.0), "right": side(500.0)},
            )()
            state.right.arm.positions = (-0.2,)
            return state

    timestamps, samples = record(
        Robot(),
        seconds=1.0,
        hz=2.0,
    )

    assert timestamps == pytest.approx([0.0, 0.5])
    assert samples == [
        DualArmTrajectorySample((0.1,), (-0.2,), 1000.0, 500.0),
        DualArmTrajectorySample((0.1,), (-0.2,), 1000.0, 500.0),
    ]
