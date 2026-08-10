import argparse
from types import SimpleNamespace

import pytest

from arx_d_can.service_tools import trajectory_recording as example


def test_frequency_defaults_to_100_hz_and_is_limited_to_500_hz():
    args = example.build_parser().parse_args(["record", "trajectory.json"])
    assert args.hz == 100.0
    assert example.parse_hz("500") == 500.0
    with pytest.raises(argparse.ArgumentTypeError):
        example.parse_hz("501")


def test_trajectory_round_trip(tmp_path):
    path = tmp_path / "trajectory.json"
    positions = [[0.0] * 7, [0.1] * 7]
    timestamps = [0.0, 0.012]

    example.save_trajectory(path, 200.0, positions, timestamps=timestamps)

    assert example.load_trajectory(path) == (200.0, timestamps, positions)


def test_legacy_trajectory_without_timestamps_uses_nominal_frequency(tmp_path):
    path = tmp_path / "legacy.json"
    positions = [[0.0] * 7, [0.1] * 7, [0.2] * 7]

    example.save_trajectory(path, 100.0, positions)

    hz, timestamps, loaded_positions = example.load_trajectory(path)
    assert hz == 100.0
    assert timestamps == pytest.approx([0.0, 0.01, 0.02])
    assert loaded_positions == positions


def test_replay_sends_every_position_at_recorded_frequency(monkeypatch):
    now = 0.0
    sleeps = []

    def fake_sleep(seconds):
        nonlocal now
        sleeps.append(seconds)
        now += seconds

    monkeypatch.setattr(example.time, "perf_counter", lambda: now)
    monkeypatch.setattr(example.time, "sleep", fake_sleep)

    class FakeArm:
        def __init__(self):
            self.arm_positions = []
            self.gripper_positions = []

        def send_joint_positions(self, positions):
            self.arm_positions.append(positions)

        def set_gripper_motor_value(self, position):
            self.gripper_positions.append(position)

    arm = FakeArm()
    positions = [[0.0] * 7, [0.1] * 7, [0.2] * 7]

    example.replay(arm, timestamps=[0.0, 0.01, 0.03], positions=positions)

    assert arm.arm_positions == [point[:6] for point in positions]
    assert arm.gripper_positions == [point[6] for point in positions]
    assert sleeps == pytest.approx([0.01, 0.02])


def test_replay_uses_selected_model_joint_count(monkeypatch):
    monkeypatch.setattr(example.time, "perf_counter", lambda: 0.0)

    class TwoJointArm:
        joint_names = ("shoulder", "elbow")

        def __init__(self):
            self.arm_positions = []
            self.gripper_positions = []

        def send_joint_positions(self, positions):
            self.arm_positions.append(positions)

        def set_gripper_motor_value(self, position):
            self.gripper_positions.append(position)

    arm = TwoJointArm()
    positions = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]

    example.replay(arm, timestamps=[0.0, 0.0], positions=positions)

    assert arm.arm_positions == [[0.1, 0.2], [0.4, 0.5]]
    assert arm.gripper_positions == [0.3, 0.6]


def test_record_parser_accepts_model_profile_selection():
    args = example.build_parser().parse_args(
        ["record", "trajectory.json", "--arm-model", "arx_d_can"]
    )

    assert args.arm_model == "arx_d_can"
    assert args.config_path is None


def test_record_parser_accepts_enable_mode():
    args = example.build_parser().parse_args(
        ["record", "trajectory.json", "--enable"]
    )

    assert args.enable is True


def test_record_parser_accepts_gravity_compensation_mode():
    args = example.build_parser().parse_args(
        ["record", "trajectory.json", "--gravity-compensation"]
    )

    assert args.gravity_compensation is True
    assert args.enable is False


def test_record_uses_gravity_sample_and_cached_gripper(monkeypatch):
    now = 0.0

    def fake_sleep(seconds):
        nonlocal now
        now += seconds

    monkeypatch.setattr(example.time, "perf_counter", lambda: now)
    monkeypatch.setattr(example.time, "sleep", fake_sleep)

    class FakeGravityMode:
        def step(self):
            return SimpleNamespace(positions=(0.1, 0.2))

    class FakeArm:
        def __init__(self):
            self.feedback_requests = []

        def read_cached_state(self):
            self.feedback_requests.append(False)
            return SimpleNamespace(
                arm=SimpleNamespace(positions=(9.0, 9.0)),
                gripper=SimpleNamespace(position=0.3),
            )

    arm = FakeArm()
    timestamps, samples = example.record(
        arm,
        seconds=1.0,
        hz=2.0,
        gravity_mode=FakeGravityMode(),
    )

    assert timestamps == pytest.approx([0.0, 0.5])
    assert samples == [[0.1, 0.2, 0.3], [0.1, 0.2, 0.3]]
    assert arm.feedback_requests == [False, False]


def test_record_stops_by_wall_clock_and_skips_expired_periods(monkeypatch):
    now = 0.0

    def fake_sleep(seconds):
        nonlocal now
        now += seconds

    class SlowArm:
        def read_state(self):
            nonlocal now
            now += 0.023
            return SimpleNamespace(
                arm=SimpleNamespace(positions=(0.1, 0.2)),
                gripper=SimpleNamespace(position=0.3),
            )

    monkeypatch.setattr(example.time, "perf_counter", lambda: now)
    monkeypatch.setattr(example.time, "sleep", fake_sleep)

    timestamps, samples = example.record(
        SlowArm(),
        seconds=0.05,
        hz=200.0,
    )

    assert len(samples) == 2
    assert timestamps == pytest.approx([0.0, 0.025])
    assert now == pytest.approx(0.048)


def test_zero_stiffness_command_has_no_position_velocity_or_torque_gain():
    class FakeArm:
        def __init__(self):
            self.commands = []

        def send_joint_positions(self, positions, **kwargs):
            self.commands.append((positions, kwargs))

    arm = FakeArm()
    example.send_zero_stiffness(
        arm,
        (0.1, -0.2),
        require_enabled=False,
    )

    positions, command = arm.commands[0]
    assert positions == (0.1, -0.2)
    assert command == {
        "velocities": (0.0, 0.0),
        "torques": (0.0, 0.0),
        "mit_kp": (0.0, 0.0),
        "mit_kd": (0.0, 0.0),
        "mode": "mit",
        "require_enabled": False,
    }
