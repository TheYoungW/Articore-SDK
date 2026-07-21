from __future__ import annotations

from pathlib import Path

import pytest

from arx_d_can import ArxDCanArm, available_models, default_config, load_cfg
from arx_d_can import sdk as sdk_module


CUSTOM_PROFILE = """
name: Test Two Joint Arm
channel: /dev/test-arm
baud: 800000
rate: 200
groups:
  arm:
    joints: [shoulder, elbow]
joints:
  - name: shoulder
    motor_id: 0x21
    feedback_id: 0x31
    model: "4340P"
    MIT: {kp: 20.0, kd: 2.0}
    POS_VEL: {vel_kp: 0.01, vel_ki: 0.001, pos_kp: 40.0, pos_ki: 0.2, vlim: 1.5}
  - name: elbow
    motor_id: 0x22
    feedback_id: 0x32
    model: "4310"
    MIT: {kp: 10.0, kd: 1.0}
    POS_VEL: {vel_kp: 0.02, vel_ki: 0.002, pos_kp: 30.0, pos_ki: 0.1, vlim: 1.0}
"""


def write_profile(tmp_path: Path, text: str = CUSTOM_PROFILE) -> Path:
    path = tmp_path / "two_joint_arm.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def test_builtin_model_registry_exposes_default_profile() -> None:
    assert "arx_d_can" in available_models()
    config = default_config(model="arx_d_can")
    assert config.model == "arx_d_can"
    assert config.hardware_config_path is not None
    assert config.urdf_path is not None
    assert config.end_effector_frame == "tool0"


def test_custom_profile_drives_sdk_and_low_level_from_same_values(tmp_path: Path) -> None:
    profile = write_profile(tmp_path)

    arm = ArxDCanArm(config_path=profile, port="/dev/override")

    assert arm.config.name == "Test Two Joint Arm"
    assert arm.config.model == "Test Two Joint Arm"
    assert arm.config.port == "/dev/override"
    assert arm.joint_names == ("shoulder", "elbow")
    assert arm.robot.model == "Test Two Joint Arm"
    assert arm.robot.joint_names == ["shoulder", "elbow"]
    assert arm.robot._all_joints[0].vel_kp == pytest.approx(0.01)
    assert arm.robot._all_joints[1].pos_kp == pytest.approx(30.0)


def test_arm_loads_selected_profile_only_once(monkeypatch, tmp_path: Path) -> None:
    profile = write_profile(tmp_path)
    original = sdk_module.load_cfg
    calls = []

    def counted(*args, **kwargs):
        calls.append((args, kwargs))
        return original(*args, **kwargs)

    monkeypatch.setattr(sdk_module, "load_cfg", counted)

    ArxDCanArm(config_path=profile)

    assert len(calls) == 1


def test_unknown_builtin_model_lists_available_choices() -> None:
    with pytest.raises(ValueError, match=r"unknown arm model.*arx_d_can"):
        load_cfg(model="missing")


def test_model_and_custom_config_are_mutually_exclusive(tmp_path: Path) -> None:
    profile = write_profile(tmp_path)
    with pytest.raises(ValueError, match="mutually exclusive"):
        load_cfg(profile, model="arx_d_can")


def test_profile_rejects_duplicate_motor_ids(tmp_path: Path) -> None:
    profile = write_profile(tmp_path, CUSTOM_PROFILE.replace("motor_id: 0x22", "motor_id: 0x21"))
    with pytest.raises(ValueError, match="duplicate motor_id"):
        load_cfg(profile)


def test_profile_rejects_unknown_group_joint(tmp_path: Path) -> None:
    profile = write_profile(
        tmp_path,
        CUSTOM_PROFILE.replace("joints: [shoulder, elbow]", "joints: [shoulder, wrist]"),
    )
    with pytest.raises(ValueError, match="references unknown joints: wrist"):
        load_cfg(profile)
