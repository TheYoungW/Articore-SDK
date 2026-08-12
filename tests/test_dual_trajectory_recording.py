from __future__ import annotations

import pytest

from arx_d_can.service_tools.dual_trajectory_recording import (
    DualArmTrajectorySample,
    load_trajectory,
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


def test_dual_replay_keeps_arm_and_gripper_commands_separate(monkeypatch) -> None:
    commands: list[tuple] = []

    class Robot:
        left = type("Side", (), {"has_gripper": True})()
        right = type("Side", (), {"has_gripper": True})()
        left.has_gripper = True
        right.has_gripper = True

        def _submit_joint_positions(self, *, left, right) -> None:
            commands.append(("arms", tuple(left), tuple(right)))

        def set_gripper_openings(self, *, left, right) -> None:
            commands.append(("grippers", left, right))

    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.perf_counter",
        lambda: 0.0,
    )
    replay(
        Robot(),
        timestamps=[0.0],
        samples=[DualArmTrajectorySample((0.1,), (0.2,), 1000.0, 0.0)],
    )

    assert commands == [
        ("arms", (0.1,), (0.2,)),
        ("grippers", 1000.0, 0.0),
    ]
