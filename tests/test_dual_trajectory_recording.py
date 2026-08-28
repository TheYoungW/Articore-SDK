from __future__ import annotations

import pytest

from arx_d_can.service_tools.dual_trajectory_recording import (
    DualArmTrajectorySample,
    MAX_RECORDING_HZ,
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


def test_dual_pv_replay_sends_each_sample_at_its_recorded_timestamp(
    monkeypatch,
) -> None:
    commands: list[tuple] = []
    now = 10.0

    def fake_sleep(seconds: float) -> None:
        nonlocal now
        now += seconds

    class Robot:
        has_grippers = True
        control_mode = "pv"

        def set_grippers(self, **kwargs) -> None:
            commands.append(("grippers", now, kwargs))

        def set_joint_pv(self, **kwargs) -> None:
            commands.append(("pv", now, kwargs))

    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.perf_counter",
        lambda: now,
    )
    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.sleep",
        fake_sleep,
    )
    samples = [
        DualArmTrajectorySample((0.1,), (0.2,), 1000.0, 500.0),
        DualArmTrajectorySample((0.3,), (0.4,), 900.0, 400.0),
    ]
    replay(Robot(), timestamps=[5.0, 5.35], samples=samples)

    pv_commands = [command for command in commands if command[0] == "pv"]
    assert [command[1] for command in pv_commands] == pytest.approx([10.0, 10.35])
    assert pv_commands[0][2] == {
        "left": (0.1,), "right": (0.2,), "velocity": 50.0,
    }
    assert pv_commands[1][2] == {
        "left": (0.3,), "right": (0.4,), "velocity": 50.0,
    }
    gripper_commands = [command for command in commands if command[0] == "grippers"]
    assert [command[2] for command in gripper_commands] == [
        {"left": 1000.0, "right": 500.0, "gripper_level": 5},
        {"left": 900.0, "right": 400.0, "gripper_level": 5},
    ]


def test_dual_mit_replay_uses_only_normal_mit_commands(monkeypatch) -> None:
    commands = []
    now = 0.0

    def fake_sleep(seconds: float) -> None:
        nonlocal now
        now += seconds

    class Robot:
        has_grippers = False
        control_mode = "mit"

        def set_joint_mit(self, **kwargs) -> None:
            commands.append((now, kwargs))

    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.perf_counter",
        lambda: now,
    )
    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.sleep",
        fake_sleep,
    )

    replay(
        Robot(),
        timestamps=[2.0, 2.5],
        samples=[
            DualArmTrajectorySample((0.1,), (-0.2,), None, None),
            DualArmTrajectorySample((0.2,), (-0.1,), None, None),
        ],
        velocity=75.0,
    )

    assert [command[0] for command in commands] == pytest.approx([0.0, 0.5])
    assert commands[0][1] == {
        "left": (0.1,), "right": (-0.2,), "velocity": 75.0,
    }
    assert commands[1][1] == {
        "left": (0.2,), "right": (-0.1,), "velocity": 75.0,
    }


def test_dual_replay_rejects_invalid_timestamps_velocity_and_grippers() -> None:
    class Robot:
        has_grippers = True
        control_mode = "pv"

        def set_joint_pv(self, **_kwargs) -> None:
            raise AssertionError("invalid replay must fail before sending")

    valid = DualArmTrajectorySample((0.1,), (0.2,), None, None)
    partial_gripper = DualArmTrajectorySample((0.1,), (0.2,), 500.0, None)
    with pytest.raises(ValueError, match="strictly increasing"):
        replay(Robot(), timestamps=[0.0, 0.0], samples=[valid, valid])
    with pytest.raises(ValueError, match="velocity"):
        replay(Robot(), timestamps=[0.0], samples=[valid], velocity=100.1)
    with pytest.raises(ValueError, match="both sides"):
        replay(Robot(), timestamps=[0.0], samples=[partial_gripper])


def test_recording_frequency_is_limited_to_500_hz(tmp_path) -> None:
    sample = DualArmTrajectorySample((0.0,), (0.0,), None, None)
    save_trajectory(
        tmp_path / "at_limit.json",
        hz=MAX_RECORDING_HZ,
        timestamps=[0.0],
        samples=[sample],
        left_joint_names=("l1",),
        right_joint_names=("r1",),
    )
    with pytest.raises(ValueError, match="must not exceed 500"):
        save_trajectory(
            tmp_path / "too_fast.json",
            hz=500.01,
            timestamps=[0.0],
            samples=[sample],
            left_joint_names=("l1",),
            right_joint_names=("r1",),
        )
    with pytest.raises(ValueError, match="must not exceed 500"):
        record(object(), seconds=1.0, hz=500.01)


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
