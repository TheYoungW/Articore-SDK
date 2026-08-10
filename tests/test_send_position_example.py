import math

import pytest

from arx_d_can.examples import example_04_send_position as example


def test_parser_only_exposes_simple_position_options() -> None:
    parser = example.build_parser()

    assert parser.parse_args([]).mode == "pv"
    assert parser.parse_args(["--mode", "mit"]).mode == "mit"
    assert parser.parse_args([]).seconds == 0.0
    assert parser.parse_args([]).hz == 100.0


def test_main_uses_blocking_high_level_hold(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeArm:
        joint_names = tuple(f"joint{index}" for index in range(1, 7))

        def connect(self):
            captured["calls"].append("connect")

        def configure(self):
            captured["calls"].append("configure")

        def enable(self):
            captured["calls"].append("enable")

        def hold_joint_positions(self, positions, *, seconds, hz):
            captured["target"] = tuple(positions)
            captured["seconds"] = seconds
            captured["hz"] = hz

        def close(self):
            captured["calls"].append("close")

    def fake_arm(**kwargs):
        captured["enable_gripper"] = kwargs["enable_gripper"]
        captured["mode"] = kwargs["control_mode"]
        captured["port"] = kwargs["port"]
        return FakeArm()

    monkeypatch.setattr(example, "ArxDCanArm", fake_arm)
    args = example.build_parser().parse_args(
        ["--mode", "mit", "--positions", "0,10,20,30,40,50", "--seconds", "2"]
    )

    example.main(args)

    assert captured["enable_gripper"] is True
    assert captured["mode"] == "mit"
    assert captured["port"] is None
    assert captured["calls"] == ["connect", "configure", "enable", "close"]
    assert captured["target"] == pytest.approx(
        tuple(math.radians(value) for value in (0, 10, 20, 30, 40, 50))
    )
    assert captured["seconds"] == 2.0
    assert captured["hz"] == 100.0
