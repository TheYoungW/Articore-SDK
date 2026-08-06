from __future__ import annotations

import math
from pathlib import Path
import time

import numpy as np
import pytest

from arx_d_can import ArxDCanArm
from arx_d_can.kinematics.coupled_joint_transform import CoupledJointTransform


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "arx_d_can"
    / "models"
    / "corina_v2_joint_transform.json"
)
JOINT_NAMES = tuple(
    [f"left_leg_joint{index}" for index in range(1, 7)]
    + [f"right_leg_joint{index}" for index in range(1, 7)]
)
PAIR_INDICES = (4, 5, 10, 11)


def test_trained_transform_round_trips_virtual_positions() -> None:
    transform = CoupledJointTransform.load(MODEL_PATH, joint_names=JOINT_NAMES)
    virtual = np.zeros(12)
    virtual[4:6] = np.radians((10.0, 4.0))
    virtual[10:12] = np.radians((-10.0, -4.0))

    motor = transform.virtual_positions_to_motor(virtual)
    recovered = transform.motor_positions_to_virtual(motor)

    assert motor[list(PAIR_INDICES)] != pytest.approx(virtual[list(PAIR_INDICES)])
    assert recovered[list(PAIR_INDICES)] == pytest.approx(
        virtual[list(PAIR_INDICES)],
        abs=math.radians(0.7),
    )


def test_trained_transform_preserves_velocity_and_power_coordinates() -> None:
    transform = CoupledJointTransform.load(MODEL_PATH, joint_names=JOINT_NAMES)
    virtual_position = np.zeros(12)
    virtual_position[10:12] = np.radians((8.0, -3.0))
    virtual_velocity = np.zeros(12)
    virtual_velocity[10:12] = np.radians((2.0, 1.0))
    virtual_torque = np.zeros(12)
    virtual_torque[10:12] = (1.0, 0.5)

    motor_position = transform.virtual_positions_to_motor(virtual_position)
    motor_velocity = transform.virtual_velocities_to_motor(
        virtual_position,
        virtual_velocity,
    )
    motor_torque = transform.virtual_torques_to_motor(
        virtual_position,
        virtual_torque,
    )

    assert float(motor_torque @ motor_velocity) == pytest.approx(
        float(virtual_torque @ virtual_velocity),
        rel=1e-12,
        abs=1e-12,
    )
    recovered_velocity = transform.motor_velocities_to_virtual(
        motor_position,
        motor_velocity,
    )
    recovered_torque = transform.motor_torques_to_virtual(
        motor_position,
        motor_torque,
    )
    assert recovered_velocity[10:12] == pytest.approx(
        virtual_velocity[10:12],
        abs=math.radians(0.05),
    )
    assert recovered_torque[10:12] == pytest.approx(
        virtual_torque[10:12],
        abs=0.02,
    )


def test_corina_sdk_transparently_converts_joint_commands(monkeypatch) -> None:
    arm = ArxDCanArm(model="corina_v2", control_mode="mit")
    arm._connected = True
    arm._configured = True
    arm._enabled = True
    captured = {}

    def capture(position, **kwargs):
        captured["position"] = np.asarray(position)
        captured.update(kwargs)

    monkeypatch.setattr(arm.robot.arm, "send_mit", capture)
    actual_virtual = np.zeros(12)
    actual_motor = arm._joint_transform.virtual_positions_to_motor(actual_virtual)
    monkeypatch.setattr(
        arm.robot,
        "get_state",
        lambda **_kwargs: (actual_motor, np.zeros(12), np.zeros(12)),
    )
    monkeypatch.setattr(
        arm.robot,
        "get_status_codes",
        lambda **_kwargs: {name: 1 for name in arm.joint_names},
    )
    virtual_position = np.zeros(12)
    virtual_position[10:12] = np.radians((1.0, 0.5))
    virtual_velocity = np.zeros(12)
    virtual_velocity[10:12] = np.radians((2.0, 1.0))
    virtual_torque = np.zeros(12)
    virtual_torque[10:12] = (1.0, 0.5)

    arm.send_joint_positions(
        virtual_position,
        velocities=virtual_velocity,
        torques=virtual_torque,
        mode="mit",
    )

    command = arm._last_mit_command
    expected_position, expected_velocity, expected_kp, expected_kd, expected_torque = (
        arm._compose_mit_motor_command(
            command,
            motor_positions=actual_motor,
            motor_velocities=np.zeros(12),
        )
    )
    assert captured["position"] == pytest.approx(expected_position)
    assert captured["vel"] == pytest.approx(expected_velocity)
    assert captured["tau"] == pytest.approx(expected_torque)
    assert captured["kp"] == pytest.approx(expected_kp)
    assert captured["kd"] == pytest.approx(expected_kd)
    assert captured["kp"][list(PAIR_INDICES)] == pytest.approx(0.0)
    assert captured["kd"][list(PAIR_INDICES)] == pytest.approx(0.0)
    assert arm._last_joint_command == pytest.approx(tuple(virtual_position))
    assert command.positions == pytest.approx(tuple(virtual_position))
    assert command.velocities == pytest.approx(tuple(virtual_velocity))
    assert command.feedforward_torques == pytest.approx(tuple(virtual_torque))


@pytest.mark.parametrize("axis", (0, 1))
def test_virtual_pd_single_axis_does_not_create_cross_axis_torque(axis: int) -> None:
    arm = ArxDCanArm(model="corina_v2", control_mode="mit")
    indices = (10, 11)
    actual_virtual = np.zeros(12)
    actual_motor = arm._joint_transform.virtual_positions_to_motor(actual_virtual)
    desired = actual_virtual.copy()
    desired[indices[axis]] = math.radians(1.0)
    kp = np.zeros(12)
    kp[list(indices)] = (60.0, 30.0)
    command = arm._make_mit_command(
        desired,
        np.zeros(12),
        kp,
        np.zeros(12),
        np.zeros(12),
    )

    _, _, motor_kp, motor_kd, motor_tau = arm._compose_mit_motor_command(
        command,
        motor_positions=actual_motor,
        motor_velocities=np.zeros(12),
    )
    recovered = arm._joint_transform.motor_torques_to_virtual(
        actual_motor,
        motor_tau,
    )

    expected = np.zeros(2)
    expected[axis] = kp[indices[axis]] * math.radians(1.0)
    assert recovered[list(indices)] == pytest.approx(expected, abs=0.03)
    assert motor_kp[list(indices)] == pytest.approx(0.0)
    assert motor_kd[list(indices)] == pytest.approx(0.0)


def test_virtual_pd_matches_logical_formula_at_random_states() -> None:
    arm = ArxDCanArm(model="corina_v2", control_mode="mit")
    rng = np.random.default_rng(20260807)
    indices = np.asarray((10, 11))
    for _ in range(40):
        actual = np.zeros(12)
        actual[indices] = np.radians(rng.uniform((-12.0, -5.0), (12.0, 5.0)))
        actual_velocity = np.zeros(12)
        actual_velocity[indices] = np.radians(rng.uniform(-2.0, 2.0, size=2))
        desired = actual.copy()
        desired[indices] += np.radians(rng.uniform(-0.5, 0.5, size=2))
        desired_velocity = np.zeros(12)
        desired_velocity[indices] = np.radians(rng.uniform(-1.0, 1.0, size=2))
        kp = np.zeros(12)
        kd = np.zeros(12)
        kp[indices] = (60.0, 30.0)
        kd[indices] = (1.5, 0.8)
        feedforward = np.zeros(12)
        feedforward[indices] = rng.uniform(-0.2, 0.2, size=2)
        motor_position = arm._joint_transform.virtual_positions_to_motor(actual)
        motor_velocity = arm._joint_transform.virtual_velocities_to_motor(
            actual,
            actual_velocity,
        )
        command = arm._make_mit_command(
            desired,
            desired_velocity,
            kp,
            kd,
            feedforward,
        )

        *_, motor_tau = arm._compose_mit_motor_command(
            command,
            motor_positions=motor_position,
            motor_velocities=motor_velocity,
        )
        observed_position = arm._joint_transform.motor_positions_to_virtual(
            motor_position,
        )
        observed_velocity = arm._joint_transform.motor_velocities_to_virtual(
            motor_position,
            motor_velocity,
        )
        expected_virtual = np.zeros(12)
        expected_virtual[indices] = (
            kp[indices] * (desired[indices] - observed_position[indices])
            + kd[indices] * (desired_velocity[indices] - observed_velocity[indices])
            + feedforward[indices]
        )
        expected_motor = arm._joint_transform.virtual_torques_to_motor(
            observed_position,
            expected_virtual,
        )
        assert motor_tau == pytest.approx(expected_motor, abs=1e-12)
        assert not arm.coupled_torque_saturation.active


def test_corina_coupled_motor_torque_is_limited_and_reported() -> None:
    arm = ArxDCanArm(model="corina_v2", control_mode="mit")
    actual = np.zeros(12)
    motor_position = arm._joint_transform.virtual_positions_to_motor(actual)
    desired = actual.copy()
    desired[10:12] = np.radians((20.0, 10.0))
    kp = np.zeros(12)
    kp[10:12] = (60.0, 30.0)
    command = arm._make_mit_command(
        desired,
        np.zeros(12),
        kp,
        np.zeros(12),
        np.zeros(12),
    )

    *_, motor_tau = arm._compose_mit_motor_command(
        command,
        motor_positions=motor_position,
        motor_velocities=np.zeros(12),
    )

    assert np.max(np.abs(motor_tau[list(PAIR_INDICES)])) <= 7.0
    status = arm.coupled_torque_saturation
    assert status.active
    assert status.motor_names
    assert max(abs(value) for value in status.applied_torques) <= 7.0


def test_corina_uses_500_hz_virtual_control_rate() -> None:
    arm = ArxDCanArm(model="corina_v2", control_mode="mit")
    assert arm.config.control_hz == pytest.approx(500.0)
    assert arm.config.safe_hold_hz == pytest.approx(500.0)


def test_coupled_enable_never_sends_virtual_gains_to_motors(monkeypatch) -> None:
    arm = ArxDCanArm(model="corina_v2", control_mode="mit")
    captured: dict[str, np.ndarray] = {}

    def capture(**kwargs):
        captured.update(
            {
                name: np.asarray(value).copy()
                for name, value in kwargs.items()
                if value is not None
            }
        )

    monkeypatch.setattr(arm.robot.arm, "enable", capture)
    monkeypatch.setattr(arm, "_start_watchdog", lambda: None)
    monkeypatch.setattr(arm, "_start_coupled_control", lambda: None)
    arm._connected = True
    arm._configured = True
    arm.enable(
        initial_positions=np.zeros(12),
        initial_velocities=np.zeros(12),
        initial_torques=np.zeros(12),
    )

    assert captured["mit_kp"][list(PAIR_INDICES)] == pytest.approx(0.0)
    assert captured["mit_kd"][list(PAIR_INDICES)] == pytest.approx(0.0)
    assert captured["mit_tau"][list(PAIR_INDICES)] == pytest.approx(0.0)
    assert arm._last_mit_command is not None
    assert np.asarray(arm._last_mit_command.kp)[10:12] == pytest.approx(
        (60.0, 30.0)
    )


def test_coupled_control_loop_uses_cached_feedback_without_active_requests(
    monkeypatch,
) -> None:
    arm = ArxDCanArm(model="corina_v2", control_mode="mit")
    actual = np.zeros(12)
    motor_position = arm._joint_transform.virtual_positions_to_motor(actual)
    feedback_calls: list[dict] = []
    sent: list[np.ndarray] = []

    def cached_feedback(**kwargs):
        feedback_calls.append(kwargs)
        return motor_position, np.zeros(12), np.zeros(12)

    def capture(_position, **kwargs):
        sent.append(np.asarray(kwargs["tau"]).copy())

    monkeypatch.setattr(arm.robot, "get_state", cached_feedback)
    monkeypatch.setattr(
        arm.robot,
        "get_status_codes",
        lambda **_kwargs: {name: 1 for name in arm.joint_names},
    )
    monkeypatch.setattr(arm.robot.arm, "send_mit", capture)
    arm._connected = True
    arm._configured = True
    arm._enabled = True
    desired = actual.copy()
    desired[10] = math.radians(0.5)
    command = arm._make_mit_command(
        desired,
        np.zeros(12),
        np.asarray([joint.mit_kp for joint in arm.config.arm_joints]),
        np.asarray([joint.mit_kd for joint in arm.config.arm_joints]),
        np.zeros(12),
    )
    arm._last_mit_command = command
    arm._last_joint_command = command.positions

    try:
        arm._start_coupled_control()
        deadline = time.monotonic() + 0.1
        while len(sent) < 3 and time.monotonic() < deadline:
            time.sleep(0.001)
    finally:
        arm._stop_coupled_control()

    assert len(sent) >= 3
    assert feedback_calls
    assert all(call["request_feedback"] is False for call in feedback_calls)
    assert all(call["require_complete"] is True for call in feedback_calls)


def test_corina_sdk_returns_virtual_joint_feedback(monkeypatch) -> None:
    arm = ArxDCanArm(model="corina_v2", control_mode="mit")
    arm._connected = True
    virtual_position = np.zeros(12)
    virtual_position[4:6] = np.radians((8.0, 3.0))
    virtual_position[10:12] = np.radians((-8.0, -3.0))
    motor_position = arm._joint_transform.virtual_positions_to_motor(
        virtual_position
    )
    motor_velocity = np.zeros(12)
    motor_torque = np.zeros(12)

    monkeypatch.setattr(
        arm.robot,
        "get_state",
        lambda **_kwargs: (motor_position, motor_velocity, motor_torque),
    )
    monkeypatch.setattr(
        arm.robot,
        "get_status_codes",
        lambda **_kwargs: {name: 0 for name in arm.joint_names},
    )

    state = arm.read_state(request_feedback=True)

    assert np.asarray(state.arm.positions)[list(PAIR_INDICES)] == pytest.approx(
        virtual_position[list(PAIR_INDICES)],
        abs=math.radians(0.7),
    )


def test_corina_physical_pair_does_not_reuse_virtual_urdf_limits() -> None:
    arm = ArxDCanArm(model="corina_v2")
    logical_by_name = {joint.name: joint for joint in arm.config.arm_joints}
    physical_by_name = {joint.name: joint for joint in arm.robot.arm._jcfgs}

    for side in ("left", "right"):
        for index in (5, 6):
            name = f"{side}_leg_joint{index}"
            assert logical_by_name[name].lower_limit is not None
            assert logical_by_name[name].upper_limit is not None
            assert physical_by_name[name].lower_limit is None
            assert physical_by_name[name].upper_limit is None
            assert (physical_by_name[name].kp, physical_by_name[name].kd) == (
                0.0,
                0.0,
            )


def test_corina_coupled_model_reference_cannot_be_motor_zeroed() -> None:
    arm = ArxDCanArm(model="corina_v2")
    arm._connected = True

    with pytest.raises(RuntimeError, match="cannot be motor-zeroed"):
        arm.set_zero(joint_names=["right_leg_joint5"])
