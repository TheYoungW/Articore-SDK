from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from arx_d_can.controllers.gravity_compensation import GravityCompensationMode
from arx_d_can.sdk import (
    ArxDCanArm,
    ArxDCanConfig,
    ArxDCanState,
    JointMotorConfig,
    JointState,
)
from arx_d_can.service_tools import gravity_compensation_cli


def joint(name: str, *, effort_limit: float | None = None) -> JointMotorConfig:
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
        effort_limit=effort_limit,
        lower_limit=-2.0,
        upper_limit=2.0,
    )


class FakeArm:
    def __init__(self, *, mode: str = "mit", effort_limit: float | None = None) -> None:
        self.config = ArxDCanConfig(
            arm_control_mode=mode,
            feedback_check_hz=100.0,
            arm_joints=(
                joint("joint1", effort_limit=effort_limit),
                joint("joint2", effort_limit=effort_limit),
            ),
        )
        self.joint_names = self.config.joint_names
        self._mode = mode
        self.connected = False
        self.enabled = False
        self.faulted = False
        self.safe_holding = False
        self.fault_reason = None
        self.positions = (0.1, -0.2)
        self.velocities = (0.0, 0.0)
        self.calls: list[tuple[str, object]] = []
        self.commands: list[dict[str, object]] = []
        self.fresh_reads = 0
        self.cached_reads = 0

    def connect(self) -> None:
        self.connected = True
        self.calls.append(("connect", None))

    def enable(self, **kwargs) -> None:
        self.enabled = True
        self.calls.append(("enable", kwargs))

    def disable(self) -> None:
        self.enabled = False
        self.calls.append(("disable", None))

    def close(self) -> None:
        self.connected = False
        self.enabled = False
        self.calls.append(("close", None))

    def _state(self) -> ArxDCanState:
        return ArxDCanState(
            arm=JointState(
                names=self.joint_names,
                positions=self.positions,
                velocities=self.velocities,
                torques=(0.0, 0.0),
            )
        )

    def read_state(self) -> ArxDCanState:
        self.fresh_reads += 1
        return self._state()

    def read_cached_state(self) -> ArxDCanState:
        self.cached_reads += 1
        return self._state()

    def _submit_joint_positions(self, positions, **kwargs) -> None:
        assert self.enabled
        self.commands.append(
            {
                "positions": np.asarray(positions, dtype=float).copy(),
                **{
                    key: np.asarray(value, dtype=float).copy()
                    for key, value in kwargs.items()
                },
            }
        )


def make_mode(arm: FakeArm, **kwargs) -> GravityCompensationMode:
    return GravityCompensationMode(
        arm,
        transition_seconds=0.0,
        gravity_provider=lambda _positions: np.array([1.0, -2.0]),
        **kwargs,
    )


def test_mode_seeds_current_position_and_submits_gravity_to_runtime() -> None:
    arm = FakeArm()
    mode = make_mode(arm)

    sample = mode.start()

    assert mode.active
    assert mode.hz == pytest.approx(arm.config.control_hz)
    assert arm.fresh_reads == 1
    assert arm.cached_reads == 1
    assert [name for name, _ in arm.calls[:2]] == ["connect", "enable"]
    assert arm.calls[1][1] == {}
    np.testing.assert_allclose(arm.commands[-1]["mit_kp"], [0.0, 0.0])
    np.testing.assert_allclose(arm.commands[-1]["mit_kd"], [0.0, 0.0])
    assert not bool(arm.commands[-1]["enforce_position_limits"])
    np.testing.assert_allclose(arm.commands[-1]["torques"], [1.0, -2.0])
    assert sample.commanded_torques == pytest.approx((1.0, -2.0))

    mode.stop()

    assert not arm.connected
    assert not arm.enabled
    assert ("disable", None) in arm.calls
    assert arm.calls[-1] == ("close", None)


def test_mode_uses_native_cache_without_python_feedback_thread() -> None:
    arm = FakeArm()
    mode = make_mode(arm)
    mode.start()
    try:
        fresh_before = arm.fresh_reads
        cached_before = arm.cached_reads

        mode.step()

        assert arm.fresh_reads == fresh_before
        assert arm.cached_reads == cached_before + 1
    finally:
        mode.stop()


def test_mode_supports_positive_scale_and_damping() -> None:
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
        np.testing.assert_allclose(arm.commands[-1]["mit_kd"], [0.3, 0.1])
    finally:
        mode.stop()


def test_mode_rejects_negative_joint_scale() -> None:
    with pytest.raises(ValueError, match="joint_scales"):
        make_mode(FakeArm(), joint_scales=(1.0, -1.0))


def test_mode_rejects_update_rate_too_close_to_watchdog_timeout() -> None:
    with pytest.raises(ValueError, match="hz 过低"):
        make_mode(FakeArm(), hz=5.0)


def test_mode_reports_urdf_effort_limiting() -> None:
    arm = FakeArm(effort_limit=1.5)
    mode = GravityCompensationMode(
        arm,
        transition_seconds=0.0,
        gravity_provider=lambda _positions: np.array([3.0, -1.0]),
    )

    sample = mode.start()
    try:
        assert sample.commanded_torques == pytest.approx((1.5, -1.0))
        assert sample.limited_joints == ("joint1",)
    finally:
        mode.stop()


def test_mode_requires_mit_selected_at_construction() -> None:
    arm = FakeArm(mode="pv")
    mode = make_mode(arm)

    with pytest.raises(RuntimeError, match="control_mode='mit'"):
        mode.start()

    assert not arm.connected
    assert not arm.enabled


def test_mode_accepts_feedback_outside_urdf_position_limits() -> None:
    arm = FakeArm()
    arm.positions = (2.1, 0.0)
    mode = make_mode(arm)

    sample = mode.start()
    assert sample.positions == pytest.approx((2.1, 0.0))
    assert arm.enabled

    mode.stop()
    assert not arm.enabled
    assert not arm.connected


def test_sdk_internal_validation_can_skip_only_position_limits() -> None:
    arm = ArxDCanArm(model="yunyi_v1_0_left", enable_gripper=False)
    positions = [0.0] * len(arm.joint_names)
    assert arm.config.arm_joints[-1].upper_limit is not None
    positions[-1] = arm.config.arm_joints[-1].upper_limit + 0.01

    with pytest.raises(ValueError, match="l-joint7"):
        arm._validated_joint_positions(positions)

    assert arm._validated_joint_positions(
        positions,
        enforce_limits=False,
    ) == pytest.approx(positions)


def test_mode_does_not_send_python_gripper_commands() -> None:
    arm = FakeArm()
    arm.enable_gripper = True
    mode = make_mode(arm)

    mode.start()
    try:
        mode.step()
    finally:
        mode.stop()

    assert not hasattr(arm, "set_gripper_motor_value")


def test_example_parser_only_exposes_connection_options() -> None:
    args = gravity_compensation_cli.build_parser().parse_args([])

    assert args.arm_model == "yunyi_v1_0_right"
    assert args.port is None
    assert args.transport is None
    assert not hasattr(args, "joint_scales")
    assert not hasattr(args, "damping")


def test_mode_stops_immediately_when_runtime_faults() -> None:
    arm = FakeArm()
    mode = make_mode(arm)
    mode.start()
    arm.faulted = True
    arm.fault_reason = "feedback timeout"

    with pytest.raises(RuntimeError, match="feedback timeout"):
        mode.step()

    mode.stop()
    assert not arm.enabled
