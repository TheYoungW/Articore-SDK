from __future__ import annotations

import pytest

from arx_d_can.service_tools.dual_trajectory_recording import (
    DualArmTrajectorySample,
    interpolate_sample,
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


def test_dual_replay_keeps_arm_and_gripper_commands_separate(monkeypatch) -> None:
    commands: list[tuple] = []

    class Robot:
        left = type(
            "Side",
            (),
            {
                "has_gripper": True,
                "_mode": "pv",
                "config": type("Config", (), {"command_timeout_s": 0.25})(),
            },
        )()
        right = type(
            "Side",
            (),
            {
                "has_gripper": True,
                "_mode": "pv",
                "config": type("Config", (), {"command_timeout_s": 0.25})(),
            },
        )()
        left.has_gripper = True
        right.has_gripper = True

        def _submit_joint_positions(
            self,
            *,
            left,
            right,
            left_velocity_limits,
            right_velocity_limits,
        ) -> None:
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


def test_dual_replay_refreshes_raw_target_across_long_sample_gap(
    monkeypatch,
) -> None:
    now = 0.0
    positions = []

    def sleep(seconds: float) -> None:
        nonlocal now
        now += seconds

    class Robot:
        left = type(
            "Side",
            (),
            {
                "has_gripper": False,
                "_mode": "pv",
                "config": type("Config", (), {"command_timeout_s": 0.25})(),
            },
        )()
        right = left

        def _submit_joint_positions(self, *, left, right, **_kwargs) -> None:
            positions.append((now, tuple(left), tuple(right)))

    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.perf_counter",
        lambda: now,
    )
    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.sleep",
        sleep,
    )
    replay(
        Robot(),
        timestamps=[0.0, 0.35],
        samples=[
            DualArmTrajectorySample((0.1,), (0.2,), None, None),
            DualArmTrajectorySample((0.3,), (0.4,), None, None),
        ],
    )

    assert len(positions) == 176
    assert [command[0] for command in positions[:3]] == pytest.approx(
        [0.0, 0.002, 0.004]
    )
    assert positions[-1][0] == pytest.approx(0.35)
    assert positions[-1][1:] == ((0.3,), (0.4,))


@pytest.mark.parametrize(
    ("mode", "expected"),
    [
        ("none", 0.0),
        ("linear", 0.25),
        ("quintic", 0.103515625),
    ],
)
def test_dual_interpolation_modes(mode, expected) -> None:
    start = DualArmTrajectorySample((0.0,), (0.0,), 0.0, 0.0)
    end = DualArmTrajectorySample((1.0,), (-1.0,), 1000.0, 500.0)

    sample = interpolate_sample(start, end, progress=0.25, mode=mode)

    assert sample.left_positions == pytest.approx((expected,))
    assert sample.right_positions == pytest.approx((-expected,))
    assert sample.left_gripper == pytest.approx(1000.0 * expected)
    assert sample.right_gripper == pytest.approx(500.0 * expected)


def test_dual_mit_replay_uses_yaml_gains_and_zero_dynamic_targets(
    monkeypatch,
) -> None:
    commands = []

    class Robot:
        left = type("Side", (), {"has_gripper": False, "_mode": "mit"})()
        right = type("Side", (), {"has_gripper": False, "_mode": "mit"})()

        def _submit_joint_positions(self, **kwargs) -> None:
            commands.append(kwargs)

    monkeypatch.setattr(
        "arx_d_can.service_tools.dual_trajectory_recording.time.perf_counter",
        lambda: 0.0,
    )
    replay(
        Robot(),
        timestamps=[0.0],
        samples=[DualArmTrajectorySample((0.1,), (-0.2,), None, None)],
        interpolation="none",
        velocity_limit=None,
    )

    assert commands == [{"left": (0.1,), "right": (-0.2,)}]


def test_dual_record_uses_atomic_gravity_sample_and_cached_grippers(
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

    class Gravity:
        def step(self):
            return type(
                "Sample",
                (),
                {
                    "left": type("Side", (), {"positions": (0.1,)})(),
                    "right": type("Side", (), {"positions": (-0.2,)})(),
                },
            )()

    class Robot:
        def read_cached_state(self):
            side = lambda opening: type(
                "Side",
                (),
                {
                    "gripper": type("Gripper", (), {"opening": opening})(),
                },
            )()
            return type(
                "State",
                (),
                {"left": side(1000.0), "right": side(500.0)},
            )()

    timestamps, samples = record(
        Robot(),
        seconds=1.0,
        hz=2.0,
        gravity_mode=Gravity(),
    )

    assert timestamps == pytest.approx([0.0, 0.5])
    assert samples == [
        DualArmTrajectorySample((0.1,), (-0.2,), 1000.0, 500.0),
        DualArmTrajectorySample((0.1,), (-0.2,), 1000.0, 500.0),
    ]
