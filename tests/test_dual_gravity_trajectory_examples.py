from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

import arx_d_can
import arx_d_can.actuator as actuator_module
from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm import (
    example_16_record_gravity_trajectory as record_example,
)
from arx_d_can.examples.dual_arm import (
    example_17_replay_trajectory as replay_example,
)
from arx_d_can.service_tools.dual_trajectory_recording import (
    DualArmTrajectorySample,
)


def test_record_parser_only_exposes_dual_product_inputs() -> None:
    args = record_example.build_parser().parse_args(
        ["--output", "dual.json", "--seconds", "12", "--hz", "1000"]
    )

    assert args.seconds == 12.0
    assert args.hz == 1000.0
    assert not hasattr(args, "arm_model")


def test_public_api_only_exposes_dual_arm_gravity_compensation() -> None:
    assert "DualArmGravityCompensationMode" in arx_d_can.__all__
    assert "GravityCompensationMode" not in arx_d_can.__all__
    assert not hasattr(arx_d_can, "GravityCompensationMode")
    for legacy_name in ("ArxDCan", "JointCfg", "JointGroup", "ArxDCanEndPose"):
        assert legacy_name not in arx_d_can.__all__
        assert not hasattr(arx_d_can, legacy_name)
    assert {"ArxDCan", "JointCfg", "JointGroup"}.isdisjoint(
        actuator_module.__all__
    )
    for legacy_name in ("ArxDCan", "JointCfg", "JointGroup"):
        assert not hasattr(actuator_module, legacy_name)


def test_replay_parser_defaults_to_safe_atomic_start() -> None:
    args = replay_example.build_parser().parse_args(["--input", "dual.json"])

    assert math.degrees(args.start_velocity) == pytest.approx(30.0)
    assert math.degrees(args.pv_velocity_limit) == pytest.approx(100.0)
    assert args.mode == "pv"
    assert args.interpolation == "quintic"
    assert math.degrees(args.position_tolerance) == pytest.approx(1.0)
    assert math.degrees(args.velocity_tolerance) == pytest.approx(2.0)


def test_replay_clips_both_sides_to_full_joint_limits() -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    left = [0.0] * len(robot.left.joint_names)
    right = [0.0] * len(robot.right.joint_names)
    left[5] = math.radians(45.1)
    right[5] = math.radians(-45.1)
    sample = DualArmTrajectorySample(tuple(left), tuple(right), None, None)

    safe, clipped = replay_example._safe_samples(robot, [sample])

    assert clipped == 1
    left_expected = robot.left.config.arm_joints[5].upper_limit
    right_expected = robot.right.config.arm_joints[5].lower_limit
    assert safe[0].left_positions[5] == pytest.approx(left_expected)
    assert safe[0].right_positions[5] == pytest.approx(right_expected)


def test_move_to_start_uses_runtime_effective_rate_for_raw_pv(monkeypatch) -> None:
    now = 0.0

    def sleep(seconds: float) -> None:
        nonlocal now
        now += seconds

    monkeypatch.setattr(replay_example.time, "monotonic", lambda: now)
    monkeypatch.setattr(replay_example.time, "perf_counter", lambda: now)
    monkeypatch.setattr(replay_example.time, "sleep", sleep)

    class Robot:
        def __init__(self):
            joint = SimpleNamespace(
                model="4340P",
                lower_limit=-1.0,
                upper_limit=1.0,
            )
            config = SimpleNamespace(arm_joints=(joint,))
            self.left = SimpleNamespace(config=config, _mode="pv")
            self.right = SimpleNamespace(config=config, _mode="pv")
            self.raw_commands = []

        def _submit_joint_positions(self, **kwargs):
            self.raw_commands.append(kwargs)

        def read_cached_state(self):
            side = SimpleNamespace(
                arm=SimpleNamespace(positions=(0.0,), velocities=(0.0,))
            )
            return SimpleNamespace(left=side, right=side)

    robot = Robot()
    target = DualArmTrajectorySample((0.0,), (0.0,), None, None)
    replay_example._move_to_start(
        robot,
        target,
        start_velocity=0.5,
        velocity_limit=1.0,
        timeout=1.0,
        position_tolerance=0.01,
        velocity_tolerance=0.01,
        control_hz=400.0,
    )

    assert now >= 0.5
    assert len(robot.raw_commands) >= 200
    assert all(
        command["left_velocity_limits"] == (1.0,)
        and command["right_velocity_limits"] == (1.0,)
        for command in robot.raw_commands
    )
