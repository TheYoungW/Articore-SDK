from __future__ import annotations

import pytest

from arx_d_can.examples.dual_arm import (
    example_08_set_gripper_openings as example,
)


def test_gripper_example_requires_both_openings() -> None:
    parser = example.build_parser()

    with pytest.raises(SystemExit):
        parser.parse_args([])
    with pytest.raises(SystemExit):
        parser.parse_args(["--left-gripper", "500"])


@pytest.mark.parametrize("value", ("-1", "1001", "nan", "inf"))
def test_gripper_example_rejects_invalid_opening(value: str) -> None:
    parser = example.build_parser()

    with pytest.raises(SystemExit):
        parser.parse_args(
            [
                "--left-gripper",
                value,
                "--right-gripper",
                "500",
            ]
        )


def test_gripper_example_submits_required_openings(monkeypatch) -> None:
    captured: dict[str, object] = {"calls": []}

    class FakeRobot:
        def connect(self) -> None:
            captured["calls"].append("connect")

        def enable(self) -> None:
            captured["calls"].append("enable")

        def set_gripper_openings(self, *, left: float, right: float) -> None:
            captured["openings"] = (left, right)

        def close(self) -> None:
            captured["calls"].append("close")

    monkeypatch.setattr(example, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr("builtins.input", lambda _prompt: "")
    args = example.build_parser().parse_args(
        [
            "--left-gripper",
            "1000",
            "--right-gripper",
            "250.5",
        ]
    )

    example.main(args)

    assert captured["openings"] == (1000.0, 250.5)
    assert captured["calls"] == ["connect", "enable", "close"]
