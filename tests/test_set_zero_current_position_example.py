from types import SimpleNamespace

from arx_d_can.examples.single_arm import example_10_set_zero_current_position as example


def test_zero_example_connects_and_zeros_all_active_motors_without_enabling(
    monkeypatch,
) -> None:
    captured = {"calls": []}

    class FakeArm:
        joint_names = ("joint1", "joint2")

        def connect(self) -> None:
            captured["calls"].append("connect")

        def set_zero(self):
            captured["calls"].append("set_zero")
            return ("joint1", "joint2", "gripper")

        def close(self) -> None:
            captured["calls"].append("close")

    def fake_arm(**kwargs):
        captured["port"] = kwargs["port"]
        captured["enable_gripper"] = kwargs["enable_gripper"]
        return FakeArm()

    monkeypatch.setattr(example, "ArxDCanArm", fake_arm)
    args = SimpleNamespace(
        arm_model="yunyi_v1_0_right",
        port="/dev/ttyACM0",
        baud=1_000_000,
        transport="dm-serial",
    )

    example.main(args)

    assert captured["port"] == "/dev/ttyACM0"
    assert captured["enable_gripper"] is True
    assert captured["calls"] == ["connect", "set_zero", "close"]
