from __future__ import annotations

from arx_d_can import GripperForceLevel
from arx_d_can.examples.single_arm import (
    example_05_set_gripper_opening as example,
)


def test_single_gripper_example_submits_speed_and_force_level(monkeypatch) -> None:
    captured: dict[str, object] = {"calls": []}

    class FakeArm:
        def __init__(self, **kwargs) -> None:
            captured["config"] = kwargs

        def connect(self) -> None:
            captured["calls"].append("connect")

        def enable(self) -> None:
            captured["calls"].append("enable")

        def set_gripper_opening(
            self,
            opening: float,
            *,
            speed: float,
            force_level: GripperForceLevel,
        ) -> None:
            captured["command"] = (opening, speed, force_level)

        def close(self) -> None:
            captured["calls"].append("close")

    monkeypatch.setattr(example, "ArxDCanArm", FakeArm)
    monkeypatch.setattr("builtins.input", lambda _prompt: "")
    args = example.build_parser().parse_args(
        [
            "--opening",
            "0",
            "--speed",
            "1000",
            "--force-level",
            "10",
        ]
    )

    example.main(args)

    assert captured["command"] == (
        0.0,
        1000.0,
        GripperForceLevel.LEVEL_10,
    )
    assert captured["calls"] == ["connect", "enable", "close"]


def test_single_gripper_example_profile_defaults() -> None:
    args = example.build_parser().parse_args(["--opening", "500"])

    assert args.speed == 1000.0
    assert args.force_level is GripperForceLevel.LEVEL_5
