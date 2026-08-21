from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pytest

import arx_d_can
from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.control import (
    example_09_record_gravity_trajectory as record_example,
)
from arx_d_can.examples.control import (
    example_10_replay_trajectory as replay_example,
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


def test_public_api_exposes_runtime_gravity_compensation_only() -> None:
    assert "DualArmGravityCompensationMode" not in arx_d_can.__all__
    assert not hasattr(arx_d_can, "DualArmGravityCompensationMode")
    assert "GravityCompensationMode" not in arx_d_can.__all__
    assert not hasattr(arx_d_can, "GravityCompensationMode")
    for legacy_name in ("ArxDCan", "JointCfg", "JointGroup", "ArxDCanEndPose"):
        assert legacy_name not in arx_d_can.__all__
        assert not hasattr(arx_d_can, legacy_name)
    assert not hasattr(arx_d_can, "ArxDCanArm")
    assert not hasattr(arx_d_can, "actuator")


def test_replay_parser_defaults_to_safe_atomic_start() -> None:
    args = replay_example.build_parser().parse_args(["--input", "dual.json"])

    assert math.degrees(args.start_velocity) == pytest.approx(30.0)
    assert args.max_speed == pytest.approx(70.0)
    assert args.mode == "pv"
    assert args.interpolation == "quintic"
    assert args.mit_target_velocity == (0.0,) * 7
    assert args.mit_kp == (190.0, 190.0, 70.0, 125.0, 10.0, 22.0, 28.0)
    assert args.mit_kd == (4.55, 4.5, 2.0, 2.9, 0.7, 0.89, 0.84)
    assert args.mit_feedforward_torque == (0.0,) * 7
    assert math.degrees(args.position_tolerance) == pytest.approx(1.0)
    assert math.degrees(args.velocity_tolerance) == pytest.approx(2.0)


def test_replay_does_not_duplicate_product_limit_logic_in_python() -> None:
    assert not hasattr(replay_example, "_safe_samples")
    source = Path(replay_example.__file__).read_text(encoding="utf-8")
    assert "clamp_joint_positions" not in source
    assert "robot.pv_velocity_limit" not in source


def test_move_to_start_uses_runtime_stepping_for_pv(monkeypatch) -> None:
    now = 0.0

    def sleep(seconds: float) -> None:
        nonlocal now
        now += seconds

    monkeypatch.setattr(replay_example.time, "monotonic", lambda: now)
    monkeypatch.setattr(replay_example.time, "perf_counter", lambda: now)
    monkeypatch.setattr(replay_example.time, "sleep", sleep)

    class Robot:
        def __init__(self):
            self.control_mode = "pv"
            self.max_speeds = []
            self.position_commands = []

        def set_max_speed(self, value):
            self.max_speeds.append(value)

        def set_joint_pv(self, **kwargs):
            self.position_commands.append(kwargs)

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
        max_speed_percent=70.0,
        timeout=1.0,
        position_tolerance=0.01,
        velocity_tolerance=0.01,
    )

    assert now >= 0.5
    assert robot.max_speeds == [70.0]
    assert robot.position_commands == [{"left": (0.0,), "right": (0.0,)}]
