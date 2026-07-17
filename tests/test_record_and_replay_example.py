import argparse

import pytest

from arx_d_can.examples import example_11_record_and_replay_trajectory as example


def test_frequency_defaults_to_100_hz_and_is_limited_to_500_hz():
    args = example.build_parser().parse_args(["record", "trajectory.json"])
    assert args.hz == 100.0
    assert example.parse_hz("500") == 500.0
    with pytest.raises(argparse.ArgumentTypeError):
        example.parse_hz("501")


def test_trajectory_round_trip(tmp_path):
    path = tmp_path / "trajectory.json"
    positions = [[0.0] * 7, [0.1] * 7]

    example.save_trajectory(path, 200.0, positions)

    assert example.load_trajectory(path) == (200.0, positions)


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

    example.replay(arm, hz=100.0, positions=positions)

    assert arm.arm_positions == [point[:6] for point in positions]
    assert arm.gripper_positions == [point[6] for point in positions]
    assert sleeps == pytest.approx([0.01, 0.01])
