from __future__ import annotations

import math

import numpy as np
import pytest

from arx_d_can.controllers.gravity_compensation import GravityCompensationMode
from arx_d_can.sdk import (
    ArxDCanConfig,
    ArxDCanState,
    JointMotorConfig,
    JointState,
)
from arx_d_can.examples import example_12_gravity_compensation as example


def joint(name: str, *, torque_range: float) -> JointMotorConfig:
    return JointMotorConfig(
        name=name,
        motor_id=1,
        feedback_id=0x11,
        model="4340P",
        mit_kp=10.0,
        mit_kd=1.0,
        pv_vel_kp=0.01,
        pv_vel_ki=0.001,
        pv_pos_kp=50.0,
        pv_pos_ki=0.5,
        pv_vlim=2.0,
        torque_range=torque_range,
        lower_limit=-2.0,
        upper_limit=2.0,
    )


class FakeArm:
    def __init__(self) -> None:
        self.config = ArxDCanConfig(
            arm_control_mode="mit",
            arm_joints=(
                joint("joint1", torque_range=10.0),
                joint("joint2", torque_range=20.0),
            ),
            watchdog_enabled=False,
        )
        self.joint_names = self.config.joint_names
        self.connected = False
        self.enabled = False
        self.faulted = False
        self.safe_holding = False
        self.fault_reason = None
        self.positions = (0.1, -0.2)
        self.velocities = (0.0, 0.0)
        self.calls: list[tuple[str, object]] = []
        self.commands: list[dict[str, object]] = []

    def connect(self) -> None:
        self.connected = True
        self.calls.append(("connect", None))

    def configure(self, mode: str) -> None:
        self.calls.append(("configure", mode))

    def enable(self) -> None:
        self.enabled = True
        self.calls.append(("enable", None))

    def disable(self) -> None:
        self.enabled = False
        self.calls.append(("disable", None))

    def close(self) -> None:
        self.connected = False
        self.enabled = False
        self.calls.append(("close", None))

    def read_state(self, *, request_feedback: bool = True) -> ArxDCanState:
        return ArxDCanState(
            arm=JointState(
                names=self.joint_names,
                positions=self.positions,
                velocities=self.velocities,
                torques=(0.0, 0.0),
            )
        )

    def send_joint_positions(self, positions, **kwargs) -> None:
        self.commands.append(
            {
                "positions": np.asarray(positions, dtype=float).copy(),
                **{
                    key: (
                        np.asarray(value, dtype=float).copy()
                        if isinstance(value, (tuple, list, np.ndarray))
                        else value
                    )
                    for key, value in kwargs.items()
                },
            }
        )


def make_mode(arm: FakeArm, **kwargs) -> GravityCompensationMode:
    return GravityCompensationMode(
        arm,
        transition_seconds=0.0,
        settle_seconds=0.0,
        gravity_provider=lambda _positions: np.array([1.0, -2.0]),
        **kwargs,
    )


def test_mode_sends_zero_kp_kd_and_gravity_torque() -> None:
    arm = FakeArm()
    mode = make_mode(arm)

    sample = mode.start()

    assert mode.active
    assert arm.connected
    assert arm.enabled
    assert arm.calls[:3] == [
        ("connect", None),
        ("configure", "mit"),
        ("enable", None),
    ]
    assert arm.commands[0]["require_enabled"] is False
    np.testing.assert_allclose(arm.commands[0]["mit_kp"], [10.0, 10.0])
    np.testing.assert_allclose(arm.commands[0]["mit_kd"], [1.0, 1.0])
    np.testing.assert_allclose(arm.commands[-1]["mit_kp"], [0.0, 0.0])
    np.testing.assert_allclose(arm.commands[-1]["mit_kd"], [0.0, 0.0])
    np.testing.assert_allclose(arm.commands[-1]["torques"], [1.0, -2.0])
    assert sample.commanded_torques == pytest.approx((1.0, -2.0))

    mode.shutdown()

    assert not arm.connected
    assert not arm.enabled
    assert ("disable", None) in arm.calls
    assert arm.calls[-1] == ("close", None)


def test_mode_supports_joint_scales_global_scale_and_damping() -> None:
    arm = FakeArm()
    mode = make_mode(
        arm,
        gravity_scale=0.5,
        joint_scales=(2.0, 0.5),
        damping=(0.3, 0.1),
    )

    mode.start()
    try:
        np.testing.assert_allclose(arm.commands[-1]["torques"], [1.0, -0.5])
        np.testing.assert_allclose(arm.commands[-1]["mit_kp"], [0.0, 0.0])
        np.testing.assert_allclose(arm.commands[-1]["mit_kd"], [0.3, 0.1])
    finally:
        mode.shutdown()


def test_mode_rejects_gravity_torque_above_configured_fraction() -> None:
    arm = FakeArm()
    mode = GravityCompensationMode(
        arm,
        transition_seconds=0.0,
        settle_seconds=0.0,
        torque_limit_ratio=0.1,
        gravity_provider=lambda _positions: np.array([1.1, 0.0]),
    )

    with pytest.raises(RuntimeError, match="joint1=.*limit"):
        mode.start()

    assert not arm.connected
    assert not arm.enabled
    assert ("enable", None) not in arm.calls
    assert arm.calls[-1] == ("close", None)


def test_mode_aborts_step_when_velocity_is_too_high() -> None:
    arm = FakeArm()
    mode = make_mode(arm, max_velocity=math.radians(5.0))
    mode.start()
    arm.velocities = (math.radians(6.0), 0.0)
    try:
        with pytest.raises(RuntimeError, match="velocity exceeded"):
            mode.step()
    finally:
        mode.shutdown()


def test_example_parser_defaults_to_pure_torque_mode() -> None:
    args = example.build_parser().parse_args([])

    assert args.seconds == 0.0
    assert args.hz == 100.0
    assert args.transition_seconds == 3.0
    assert args.damping == 0.0
    assert args.gravity_scale == 1.0
    assert args.joint_scales is None


def test_example_parses_per_joint_scales() -> None:
    assert example.parse_joint_values(
        "1,1.55,1.55,1,1,1,1",
        expected_count=7,
        name="joint scale",
    ) == pytest.approx((1.0, 1.55, 1.55, 1.0, 1.0, 1.0, 1.0))
