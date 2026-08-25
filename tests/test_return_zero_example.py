from __future__ import annotations

import argparse

import pytest

from arx_d_can.examples.control import example_06_return_zero as example


@pytest.mark.parametrize("has_grippers", (False, True))
def test_return_zero_submits_every_installed_product_target_before_waiting(
    monkeypatch, has_grippers: bool
) -> None:
    calls: list[object] = []

    class FakeRobot:
        joint_names = tuple(f"joint{index}" for index in range(1, 8))

        def __init__(self) -> None:
            self.has_grippers = has_grippers

        def connect(self) -> None:
            calls.append("connect")

        def enable(self) -> None:
            calls.append("enable")

        def set_joint_mit(self, *, left, right, velocity) -> None:
            calls.append(("arms", tuple(left), tuple(right), velocity))

        def set_grippers(self, *, left, right, gripper_level) -> None:
            calls.append(("grippers", left, right, gripper_level))

        def disconnect(self) -> None:
            calls.append("disconnect")

    def confirm(prompt: str) -> str:
        expected_prompt = (
            "确认双臂和夹爪回到零位"
            if has_grippers
            else "确认双臂回到零位"
        )
        assert expected_prompt in prompt
        calls.append("confirm")
        return ""

    monkeypatch.setattr(example, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr("builtins.input", confirm)

    example.main(argparse.Namespace(velocity=20.0))

    expected: list[object] = [
        "connect",
        "enable",
        ("arms", (0.0,) * 7, (0.0,) * 7, 20.0),
    ]
    if has_grippers:
        expected.append(("grippers", 0, 0, 5))
    expected.extend(("confirm", "disconnect"))
    assert calls == expected
