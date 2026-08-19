from __future__ import annotations

from arx_d_can import _motor_abi as native_binding

import arx_d_can
from arx_d_can import ArxDCanArm


def test_runtime_supports_connect_feedback_barrier() -> None:
    assert native_binding.get_version() == "0.10.21"
    assert native_binding.articore_runtime_abi_version() == "2.15"
    capabilities = native_binding.articore_runtime_capabilities()
    assert capabilities["native_robot_model"]
    assert capabilities["native_gravity_compensation"]
    assert capabilities["runtime_maintenance"]
    assert capabilities["product_runtime_factory"]
    assert capabilities["unified_operation_health"]
    assert capabilities["product_command_frames"]
    assert capabilities["product_state"]
    assert capabilities["optional_product_grippers"]
    assert capabilities["graded_feedback_safety"]
    assert capabilities["normalized_ordinary_speed"]
    assert capabilities["runtime_motor_power"]
    assert capabilities["product_pose"]
    assert (
        native_binding.articore_runtime_capabilities()[
            "transport_aware_control_rate"
        ]
        is True
    )
    assert (
        native_binding.articore_runtime_capabilities()[
            "connect_feedback_barrier"
        ]
        is True
    )
    assert (
        native_binding.articore_runtime_capabilities()[
            "per_cycle_mit_torque_limit"
        ]
        is True
    )


def test_public_runtime_reports_are_native_binding_types() -> None:
    assert arx_d_can.ConnectReport is native_binding.ConnectReport
    assert arx_d_can.ConnectChannelResult is native_binding.ConnectChannelResult
    assert arx_d_can.ConnectErrorCode is native_binding.ConnectErrorCode
    assert arx_d_can.ConnectMotorResult is native_binding.ConnectMotorResult
    assert arx_d_can.SafetyHealth is native_binding.SafetyHealth
    assert arx_d_can.SafetyState is native_binding.SafetyState
    assert (
        arx_d_can.RuntimeTransactionError
        is native_binding.RuntimeTransactionError
    )
    assert (
        arx_d_can.GravityCompensationPhase
        is native_binding.GravityCompensationPhase
    )
    assert (
        arx_d_can.GravityCompensationStatus
        is native_binding.GravityCompensationStatus
    )
    assert arx_d_can.NativeRobotModel is native_binding.NativeRobotModel
    assert arx_d_can.RobotSide is native_binding.RobotSide
    assert arx_d_can.JacobianReference is native_binding.JacobianReference


def test_yunyi_builds_official_runtime_configuration_types() -> None:
    arm = ArxDCanArm(model="yunyi_v1_0_left", enable_gripper=True)
    arm.robot._motor_map.update(
        {joint.name: object() for joint in arm.robot._all_joints}
    )

    config = arm._runtime_config()
    assert isinstance(config, native_binding.RuntimeConfig)
    assert config.command_timeout_ms == 250
    assert config.enable_grace_ms == 2000
    assert config.feedback_max_age_ms == 300
    assert config.gripper_control_hz == round(arm.config.control_hz)

    assert all(
        isinstance(item, native_binding.RuntimeMotor)
        for item in arm._runtime_motors()
    )
    assert all(
        isinstance(item, native_binding.JointControlConfig)
        for item in arm._runtime_joint_configs()
    )
    assert all(
        isinstance(item, native_binding.GripperProductBinding)
        for item in arm._runtime_gripper_bindings()
    )
    gravity = arm._runtime_gravity_bindings()
    assert len(gravity) == 1
    assert isinstance(gravity[0], native_binding.GravityProductBinding)
    assert gravity[0].runtime_side == 0
    assert gravity[0].robot_side == native_binding.RobotSide.LEFT
    assert gravity[0].product_id == "yunyi_v1_0"


def test_yunyi_right_native_gravity_binding_uses_right_model_side() -> None:
    arm = ArxDCanArm(model="yunyi_v1_0_right", enable_gripper=False)

    binding = arm._runtime_gravity_bindings(runtime_side=1)[0]

    assert binding.runtime_side == 1
    assert binding.robot_side == native_binding.RobotSide.RIGHT
