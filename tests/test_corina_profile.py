from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from arx_d_can import ArxDCanArm, available_models, load_cfg


MODELS_DIR = Path(__file__).resolve().parents[1] / "arx_d_can" / "models"
RIGHT_JOINTS = [f"right_leg_joint{i}" for i in range(1, 7)]
LEFT_JOINTS = [f"left_leg_joint{i}" for i in range(1, 7)]


def test_corina_v2_profile_follows_pinocchio_joint_order() -> None:
    assert "corina_v2" in available_models()

    robot = ArxDCanArm(model="corina_v2")

    assert robot.joint_names == tuple(LEFT_JOINTS + RIGHT_JOINTS)
    assert robot.config.gripper is None
    assert robot.config.end_effector_frame == "right_leg_link6"


def test_corina_v2_motor_ids_match_each_leg_on_one_bus() -> None:
    joints = load_cfg(model="corina_v2")["joints"]
    by_name = {joint.name: joint for joint in joints}

    assert [by_name[name].motor_id for name in RIGHT_JOINTS] == list(range(0x01, 0x07))
    assert [by_name[name].motor_id for name in LEFT_JOINTS] == list(
        range(0x07, 0x0D)
    )
    assert [by_name[name].feedback_id for name in RIGHT_JOINTS] == list(
        range(0x21, 0x27)
    )
    assert [by_name[name].feedback_id for name in LEFT_JOINTS] == list(
        range(0x27, 0x2D)
    )
    assert [joint.model for joint in joints] == [
        "4340P",
        "4340P",
        "4310",
        "4340P",
        "4310",
        "4310",
    ] * 2


def test_corina_v2_uses_existing_conservative_gain_defaults() -> None:
    joints = load_cfg(model="corina_v2")["joints"]

    for joint in joints:
        if joint.model == "4340P":
            assert (joint.kp, joint.kd) == (120.0, 8.0)
            assert (
                joint.vel_kp,
                joint.vel_ki,
                joint.pos_kp,
                joint.pos_ki,
                joint.vlim,
            ) == (0.0125, 0.004, 150.0, 0.5, 5.0)
        else:
            assert (joint.kp, joint.kd) == (18.0, 2.0)
            assert (
                joint.vel_kp,
                joint.vel_ki,
                joint.pos_kp,
                joint.pos_ki,
                joint.vlim,
            ) == (0.005, 0.002, 50.0, 1.0, 3.0)


def test_corina_v2_urdf_supplies_all_controlled_joint_limits() -> None:
    urdf_path = MODELS_DIR / "Corinav2.urdf"
    root = ET.parse(urdf_path).getroot()
    configured = load_cfg(model="corina_v2")["joints"]
    by_name = {joint.name: joint for joint in configured}

    assert Path(load_cfg(model="corina_v2")["urdf_path"]) == urdf_path
    assert root.attrib["name"] == "Corinav2"
    revolute_names = [
        joint.attrib["name"]
        for joint in root.findall("joint")
        if joint.attrib["type"] == "revolute"
    ]
    assert revolute_names == RIGHT_JOINTS + LEFT_JOINTS
    assert all(joint.lower_limit is not None for joint in configured)
    assert all(joint.upper_limit is not None for joint in configured)
    assert by_name["right_leg_joint2"].lower_limit == pytest.approx(
        -0.1989675347273536
    )
    assert by_name["left_leg_joint2"].upper_limit == pytest.approx(
        0.1989675347273536
    )
