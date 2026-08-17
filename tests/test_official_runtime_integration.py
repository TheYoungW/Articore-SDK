from __future__ import annotations

import motor_drive_layer

import arx_d_can
from arx_d_can import ArxDCanArm


def test_public_runtime_reports_are_motor_drive_layer_types() -> None:
    assert arx_d_can.SafetyHealth is motor_drive_layer.SafetyHealth
    assert arx_d_can.SafetyState is motor_drive_layer.SafetyState
    assert arx_d_can.EnableReport is motor_drive_layer.EnableReport
    assert arx_d_can.DisableReport is motor_drive_layer.DisableReport
    assert (
        arx_d_can.RuntimeTransactionError
        is motor_drive_layer.RuntimeTransactionError
    )


def test_yunyi_builds_official_runtime_configuration_types() -> None:
    arm = ArxDCanArm(model="yunyi_v1_0_left", enable_gripper=True)
    arm.robot._motor_map.update(
        {joint.name: object() for joint in arm.robot._all_joints}
    )

    config = arm._runtime_config()
    assert isinstance(config, motor_drive_layer.RuntimeConfig)
    assert config.command_timeout_ms == 250
    assert config.enable_grace_ms == 2000
    assert config.gripper_control_hz == round(arm.config.control_hz)

    assert all(
        isinstance(item, motor_drive_layer.RuntimeMotor)
        for item in arm._runtime_motors()
    )
    assert all(
        isinstance(item, motor_drive_layer.JointControlConfig)
        for item in arm._runtime_joint_configs()
    )
    assert all(
        isinstance(item, motor_drive_layer.JointSafetyLimits)
        for item in arm._runtime_joint_limits()
    )
    assert all(
        isinstance(item, motor_drive_layer.GripperProductBinding)
        for item in arm._runtime_gripper_bindings()
    )
