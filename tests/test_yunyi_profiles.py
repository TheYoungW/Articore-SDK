from pathlib import Path

import pytest

from arx_d_can import (
    ArxDCanArm,
    NativeRobotModel,
    RobotSide,
    available_models,
    load_cfg,
)
from arx_d_can.actuator import arx_d_can as actuator_module
from arx_d_can.driver import damiao_model_limits


MODELS_DIR = Path(__file__).resolve().parents[1] / "arx_d_can" / "models"


def test_native_protocol_ranges_only_cover_yunyi_motors() -> None:
    expected_models = {"4310", "4340P", "8009"}

    assert set(actuator_module._NATIVE_TORQUE_RANGES) == expected_models
    assert set(actuator_module._NATIVE_VELOCITY_RANGES) == expected_models


def test_yunyi_profiles_are_registered_as_independent_arms() -> None:
    assert {"yunyi_v1_0_right", "yunyi_v1_0_left"} <= set(available_models())

    right = ArxDCanArm(model="yunyi_v1_0_right", enable_gripper=True)
    left = ArxDCanArm(model="yunyi_v1_0_left", enable_gripper=True)

    assert right.joint_names == tuple(f"r-joint{i}" for i in range(1, 8))
    assert left.joint_names == tuple(f"l-joint{i}" for i in range(1, 8))
    assert right.config.gripper is not None
    assert left.config.gripper is not None
    assert right.config.gripper.name == "r-gripper"
    assert left.config.gripper.name == "l-gripper"
    assert right.config.transport == "socketcanfd"
    assert left.config.transport == "socketcanfd"
    assert right.config.port == "can-right"
    assert left.config.port == "can-left"
    assert right.config.max_cached_feedback_age_s == pytest.approx(0.3)
    assert left.config.max_cached_feedback_age_s == pytest.approx(0.3)


def test_yunyi_motor_models_and_ids_match_each_single_can_bus() -> None:
    expected_models = [
        "8009",
        "8009",
        "4340P",
        "4340P",
        "4310",
        "4310",
        "4310",
        "4310",
    ]
    for model in ("yunyi_v1_0_right", "yunyi_v1_0_left"):
        joints = load_cfg(model=model)["joints"]
        assert [joint.model for joint in joints] == expected_models
        expected_motor_ids = list(range(0x01, 0x09))
        expected_feedback_ids = list(range(0x11, 0x19))
        assert [joint.motor_id for joint in joints] == expected_motor_ids
        assert [joint.feedback_id for joint in joints] == expected_feedback_ids


def test_yunyi_joint_directions_match_verified_tf_convention() -> None:
    left = load_cfg(model="yunyi_v1_0_left")["joints"]
    right = load_cfg(model="yunyi_v1_0_right")["joints"]

    assert [joint.direction for joint in left] == [1, 1, 1, -1, -1, 1, 1, 1]
    assert [joint.direction for joint in right] == [1, 1, 1, 1, -1, -1, 1, 1]


def test_yunyi_grippers_use_motor_builtin_product_profile() -> None:
    for model in ("yunyi_v1_0_left", "yunyi_v1_0_right"):
        loaded = load_cfg(model=model)
        assert loaded["gripper_profile"] == "yunyi_gripper_v1"
        assert "gripper_mapping" not in loaded
        assert "gripper_protection" not in loaded


def test_yunyi_joint_torque_ranges_match_product_limits() -> None:
    expected = [40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0, None]
    for model in ("yunyi_v1_0_right", "yunyi_v1_0_left"):
        joints = load_cfg(model=model)["joints"]
        assert [joint.torque_range for joint in joints] == expected


def test_yunyi_joint_velocity_ranges_match_hardware_registers() -> None:
    expected_by_model = {
        "yunyi_v1_0_right": [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 30.0],
        "yunyi_v1_0_left": [20.0, 20.0, 10.0, 10.0, 10.0, 10.0, 10.0, 30.0],
    }
    for model, expected in expected_by_model.items():
        joints = load_cfg(model=model)["joints"]
        assert [joint.velocity_range for joint in joints] == expected


def test_yunyi_control_gains_and_velocity_limits_match_product_profile() -> None:
    expected_mit = (
        (190.0, 4.55),
        (190.0, 4.5),
        (70.0, 2.0),
        (125.0, 2.9),
        (10.0, 0.7),
        (22.0, 0.89),
        (28.0, 0.84),
    )
    expected_pv = (
        (0.010, 0.0, 60.0, 0.0, 16.0),
        (0.008, 0.0, 50.0, 0.0, 16.0),
        (0.0125, 0.0, 165.0, 0.0, 5.0),
        (0.0125, 0.0, 180.0, 0.0, 5.0),
        (0.006, 0.0, 50.0, 0.0, 20.0),
        (0.006, 0.0, 50.0, 0.0, 20.0),
        (0.006, 0.0, 50.0, 0.0, 20.0),
    )

    for model in ("yunyi_v1_0_left", "yunyi_v1_0_right"):
        joints = load_cfg(model=model)["joints"][:7]
        assert tuple((joint.kp, joint.kd) for joint in joints) == expected_mit
        assert tuple(
            (
                joint.vel_kp,
                joint.vel_ki,
                joint.pos_kp,
                joint.pos_ki,
                joint.vlim,
            )
            for joint in joints
        ) == expected_pv


def test_yunyi_runtime_ordinary_mit_gains_come_from_yaml() -> None:
    expected_kp = (190.0, 190.0, 70.0, 125.0, 10.0, 22.0, 28.0)
    expected_kd = (4.55, 4.5, 2.0, 2.9, 0.7, 0.89, 0.84)

    for model in ("yunyi_v1_0_left", "yunyi_v1_0_right"):
        arm = ArxDCanArm(model=model, enable_gripper=False)
        arm.robot._motor_map.update(
            {joint.name: object() for joint in arm.config.arm_joints}
        )

        native = arm._runtime_joint_configs()

        assert tuple(joint.mit_kp for joint in native) == expected_kp
        assert tuple(joint.mit_kd for joint in native) == expected_kd


def test_yunyi_runtime_torque_limits_match_product_effort_limits() -> None:
    for model in ("yunyi_v1_0_left", "yunyi_v1_0_right"):
        arm = ArxDCanArm(model=model, enable_gripper=False)
        arm.robot._motor_map.update(
            {joint.name: object() for joint in arm.config.arm_joints}
        )

        native_configs = arm._runtime_joint_configs()
        for joint, native in zip(arm.config.arm_joints, native_configs):
            _, _, native_torque_range = damiao_model_limits(joint.model)
            configured_torque_range = joint.torque_range or native_torque_range
            logical_torque_limit = (
                native.torque_limit
                * configured_torque_range
                / native_torque_range
            )

            assert joint.effort_limit is not None
            assert logical_torque_limit == pytest.approx(joint.effort_limit)


def test_yunyi_gripper_uses_fixed_default_mit_gains() -> None:
    for model in ("yunyi_v1_0_right", "yunyi_v1_0_left"):
        arm = ArxDCanArm(model=model, enable_gripper=True)

        assert arm.config.gripper is not None
        assert arm.config.gripper.mit_kp == pytest.approx(4.0)
        assert arm.config.gripper.mit_kd == pytest.approx(0.5)


def test_yunyi_arm_and_gripper_share_500_hz_normal_control_rate() -> None:
    for model in ("yunyi_v1_0_right", "yunyi_v1_0_left"):
        arm = ArxDCanArm(model=model, enable_gripper=True)

        assert arm.config.control_hz == pytest.approx(500.0)
        assert arm.config.safe_hold_hz == pytest.approx(100.0)


def test_yunyi_runtime_uses_full_calibrated_joint_limits() -> None:
    for model in ("yunyi_v1_0_right", "yunyi_v1_0_left"):
        arm = ArxDCanArm(model=model, enable_gripper=False)
        arm.robot._motor_map.update(
            {joint.name: object() for joint in arm.config.arm_joints}
        )

        native = arm._runtime_joint_configs()
        for joint, limits in zip(arm.config.arm_joints, native):
            assert joint.lower_limit is not None
            assert joint.upper_limit is not None
            expected = (
                joint.direction * joint.lower_limit,
                joint.direction * joint.upper_limit,
            )
            assert limits.lower_position == pytest.approx(min(expected))
            assert limits.upper_position == pytest.approx(max(expected))


def test_yunyi_runtime_gripper_binding_contains_only_motor_profile_id() -> None:
    arm = ArxDCanArm(model="yunyi_v1_0_right", enable_gripper=True)
    assert arm.config.gripper is not None
    motor = object()
    arm.robot._motor_map[arm.config.gripper.name] = motor

    bindings = arm._runtime_gripper_bindings()

    assert len(bindings) == 1
    assert bindings[0].motor is motor
    assert bindings[0].profile_id == "yunyi_gripper_v1"


def test_yunyi_profiles_take_authoritative_limits_from_native_model() -> None:
    expected_efforts = [40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0]
    expected_velocities = [16.0, 16.0, 5.0, 5.0, 20.0, 20.0, 20.0]

    for model in ("yunyi_v1_0_left", "yunyi_v1_0_right"):
        loaded = load_cfg(model=model)
        profile_joints = loaded["joints"][:7]
        side = RobotSide.LEFT if model.endswith("_left") else RobotSide.RIGHT
        with NativeRobotModel(side=side) as native:
            info = native.info
            assert Path(loaded["urdf_path"]) == MODELS_DIR / "yunyi_v1_0.urdf"
            assert tuple(joint.name for joint in profile_joints) == info.joint_names
            assert [joint.lower_limit for joint in profile_joints] == pytest.approx(
                info.lower_limits
            )
            assert [joint.upper_limit for joint in profile_joints] == pytest.approx(
                info.upper_limits
            )
            assert loaded["end_effector_frame"] == info.end_effector_frame
        assert [joint.effort_limit for joint in profile_joints] == expected_efforts
        assert [joint.vlim for joint in profile_joints] == expected_velocities

    assert (MODELS_DIR / "yunyi_v1_0.urdf").is_file()
    assert not (MODELS_DIR / "yunyi_v1_0_left.urdf").exists()
    assert not (MODELS_DIR / "yunyi_v1_0_right.urdf").exists()
