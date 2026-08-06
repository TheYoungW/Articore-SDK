from __future__ import annotations

import math
import threading

import numpy as np
import pytest

from arx_d_can.controllers.gravity_compensation import GravityCompensationMode
from arx_d_can.sdk import (
    ArxDCanConfig,
    ArxDCanState,
    JointMotorConfig,
    JointState,
    MotorState,
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
        self.enable_gripper = False
        self.positions = (0.1, -0.2)
        self.velocities = (0.0, 0.0)
        self.calls: list[tuple[str, object]] = []
        self.commands: list[dict[str, object]] = []
        self.feedback_requests: list[bool] = []
        self.background_feedback_event = threading.Event()

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
        self.feedback_requests.append(request_feedback)
        return ArxDCanState(
            arm=JointState(
                names=self.joint_names,
                positions=self.positions,
                velocities=self.velocities,
                torques=(0.0, 0.0),
            )
        )

    def refresh_feedback_background(self) -> ArxDCanState:
        self.background_feedback_event.set()
        return self.read_state(request_feedback=True)

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


class FakeArmWithGripper(FakeArm):
    def __init__(self) -> None:
        super().__init__()
        gripper = joint("gripper", torque_range=2.0)
        self.config = ArxDCanConfig(
            arm_control_mode="mit",
            arm_joints=self.config.arm_joints,
            gripper=gripper,
            watchdog_enabled=False,
        )
        self.joint_names = self.config.joint_names
        self.enable_gripper = True
        self.gripper_position = 0.3
        self.gripper_commands: list[float] = []

    def read_state(self, *, request_feedback: bool = True) -> ArxDCanState:
        self.feedback_requests.append(request_feedback)
        return ArxDCanState(
            arm=JointState(
                names=self.joint_names,
                positions=self.positions,
                velocities=self.velocities,
                torques=(0.0, 0.0),
            ),
            gripper=MotorState(
                name="gripper",
                motor_id=1,
                feedback_id=0x11,
                position=self.gripper_position,
                velocity=0.0,
            ),
        )

    def set_gripper_motor_value(
        self,
        value: float,
        *,
        require_enabled: bool = True,
    ) -> None:
        assert not require_enabled or self.enabled
        self.gripper_commands.append(float(value))
        self.calls.append(("gripper", float(value)))


def make_mode(arm: FakeArm, **kwargs) -> GravityCompensationMode:
    kwargs.setdefault("feedback_check_hz", 0.0)
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
    assert arm.feedback_requests[-1] is False

    mode.shutdown()

    assert not arm.connected
    assert not arm.enabled
    assert ("disable", None) in arm.calls
    assert arm.calls[-1] == ("close", None)


def test_mode_checks_complete_feedback_in_background() -> None:
    arm = FakeArm()
    mode = make_mode(arm, feedback_check_hz=1000.0)

    mode.start()
    try:
        assert arm.background_feedback_event.wait(0.2)
        cached_reads_before = arm.feedback_requests.count(False)

        mode.step()

        assert arm.feedback_requests.count(False) == cached_reads_before + 1
        assert any(arm.feedback_requests)
    finally:
        mode.shutdown()


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


def test_mode_refreshes_enabled_gripper_from_feedback() -> None:
    arm = FakeArmWithGripper()
    mode = make_mode(arm)

    mode.start()
    try:
        assert arm.gripper_commands
        assert arm.gripper_commands[0] == pytest.approx(0.3)
        enable_index = arm.calls.index(("enable", None))
        first_gripper_index = arm.calls.index(("gripper", 0.3))
        assert first_gripper_index == enable_index + 1

        arm.gripper_position = 0.45
        mode.step()

        assert arm.gripper_commands[-1] == pytest.approx(0.45)
        assert all(len(command["positions"]) == 2 for command in arm.commands)
    finally:
        mode.shutdown()


def test_mode_refreshes_enabled_gripper_during_transition(monkeypatch) -> None:
    arm = FakeArmWithGripper()
    mode = GravityCompensationMode(
        arm,
        hz=10.0,
        transition_seconds=0.1,
        settle_seconds=0.0,
        gravity_provider=lambda _positions: np.array([1.0, -2.0]),
    )
    monkeypatch.setattr("arx_d_can.controllers.gravity_compensation.time.sleep", lambda _: None)

    mode.start()
    try:
        assert len(arm.gripper_commands) >= 4
        assert arm.gripper_commands == pytest.approx(
            [arm.gripper_position] * len(arm.gripper_commands)
        )
    finally:
        mode.shutdown()


def test_mode_supports_negative_joint_scale_for_direction_calibration() -> None:
    arm = FakeArm()
    mode = make_mode(arm, joint_scales=(1.0, -0.25))

    mode.start()
    try:
        np.testing.assert_allclose(arm.commands[-1]["torques"], [1.0, 0.5])
    finally:
        mode.shutdown()


def test_mode_does_not_apply_motion_threshold_checks() -> None:
    arm = FakeArm()
    mode = GravityCompensationMode(
        arm,
        transition_seconds=0.0,
        settle_seconds=0.0,
        gravity_provider=lambda _positions: np.array([50.0, -50.0]),
    )
    arm.positions = (3.0, -3.0)
    arm.velocities = (math.radians(100.0), -math.radians(100.0))

    mode.start()
    try:
        sample = mode.step()
        assert sample.commanded_torques == pytest.approx((50.0, -50.0))
        assert sample.velocities == pytest.approx(arm.velocities)
    finally:
        mode.shutdown()


def test_example_parser_defaults_to_pure_torque_mode() -> None:
    args = example.build_parser().parse_args([])

    assert args.seconds == 0.0
    assert args.hz == 100.0
    assert args.report_hz == 1.0
    assert args.transition_seconds == 0.0
    assert args.settle_seconds == 0.0
    assert args.damping == 0.0
    assert args.gravity_scale == 1.0
    assert args.joint_scales is None


def test_example_parses_per_joint_scales() -> None:
    assert example.parse_joint_values(
        "1,1.55,1.55,1,1,1,1",
        expected_count=7,
        name="joint scale",
    ) == pytest.approx((1.0, 1.55, 1.55, 1.0, 1.0, 1.0, 1.0))

    assert example.parse_joint_values(
        "1,1,1,-0.25,1,1,1",
        expected_count=7,
        name="joint scale",
        allow_negative=True,
    ) == pytest.approx((1.0, 1.0, 1.0, -0.25, 1.0, 1.0, 1.0))
