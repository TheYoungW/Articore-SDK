from pathlib import Path
import xml.etree.ElementTree as ET

from arx_d_can import ArxDCanArm, available_models, load_cfg


MODELS_DIR = Path(__file__).resolve().parents[1] / "arx_d_can" / "models"


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
    assert right.config.port == "/dev/ttyACM1"
    assert left.config.port == "/dev/ttyACM0"


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
        expected_motor_ids = (
            list(range(0x09, 0x11))
            if model == "yunyi_v1_0_left"
            else list(range(1, 9))
        )
        expected_feedback_ids = (
            list(range(0x19, 0x21))
            if model == "yunyi_v1_0_left"
            else list(range(0x11, 0x19))
        )
        assert [joint.motor_id for joint in joints] == expected_motor_ids
        assert [joint.feedback_id for joint in joints] == expected_feedback_ids


def test_yunyi_left_joint_directions_match_hardware() -> None:
    left = load_cfg(model="yunyi_v1_0_left")["joints"]
    right = load_cfg(model="yunyi_v1_0_right")["joints"]

    assert [joint.direction for joint in left] == [-1, 1, 1, -1, 1, 1, 1, 1]
    assert [joint.direction for joint in right] == [1] * 8


def test_yunyi_profiles_share_one_authoritative_dual_arm_urdf() -> None:
    dual_path = MODELS_DIR / "yunyi_v1_0.urdf"
    root = ET.parse(dual_path).getroot()
    joints = root.findall("joint")
    names = {joint.attrib["name"] for joint in joints}

    assert {f"r-joint{i}" for i in range(1, 10)} <= names
    assert {f"l-joint{i}" for i in range(1, 10)} <= names
    assert len([joint for joint in joints if joint.attrib["type"] == "revolute"]) == 14
    assert len([joint for joint in joints if joint.attrib["type"] == "prismatic"]) == 4

    for model in ("yunyi_v1_0_right", "yunyi_v1_0_left"):
        assert Path(load_cfg(model=model)["urdf_path"]) == dual_path

    assert not (MODELS_DIR / "yunyi_v1_0_left.urdf").exists()
    assert not (MODELS_DIR / "yunyi_v1_0_right.urdf").exists()
