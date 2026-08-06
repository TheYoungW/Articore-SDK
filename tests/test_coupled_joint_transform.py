from __future__ import annotations

import math
from pathlib import Path

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
    virtual_position = np.zeros(12)
    virtual_position[10:12] = np.radians((10.0, 4.0))
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

    expected_position, expected_velocity, expected_torque, _ = (
        arm._transform_command_vectors(
            virtual_position,
            velocities=virtual_velocity,
            torques=virtual_torque,
        )
    )
    assert captured["position"] == pytest.approx(expected_position)
    assert captured["vel"] == pytest.approx(expected_velocity)
    assert captured["tau"] == pytest.approx(expected_torque)
    assert arm._last_joint_command == pytest.approx(tuple(virtual_position))


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


def test_corina_coupled_model_reference_cannot_be_motor_zeroed() -> None:
    arm = ArxDCanArm(model="corina_v2")
    arm._connected = True

    with pytest.raises(RuntimeError, match="cannot be motor-zeroed"):
        arm.set_zero(joint_names=["right_leg_joint5"])
