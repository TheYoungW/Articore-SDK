from pathlib import Path
from xml.etree import ElementTree


URDF = Path(__file__).parents[1] / "arx_d_can" / "models" / "yunyi_v1_0.urdf"


def test_yunyi_first_joint_positive_axes_match_on_both_arms() -> None:
    root = ElementTree.parse(URDF).getroot()
    axes = {
        name: root.find(f"joint[@name='{name}']/axis").attrib["xyz"]
        for name in ("l-joint1", "r-joint1")
    }
    assert axes == {"l-joint1": "0 1 0", "r-joint1": "0 1 0"}


def test_yunyi_gripper_tool_frames_are_fixed_to_link7_centers() -> None:
    root = ElementTree.parse(URDF).getroot()
    links = {element.attrib["name"] for element in root.findall("link")}
    joints = {
        element.attrib["name"]: element for element in root.findall("joint")
    }

    for prefix in ("l", "r"):
        assert f"{prefix}-tool0" in links
        joint = joints[f"{prefix}-tool0-fixed-joint"]
        assert joint.attrib["type"] == "fixed"
        assert joint.find("parent").attrib["link"] == f"{prefix}-link7"
        assert joint.find("child").attrib["link"] == f"{prefix}-tool0"
        assert joint.find("origin").attrib == {
            "xyz": "-0.004 0 -0.178",
            "rpy": "0 0 0",
        }
