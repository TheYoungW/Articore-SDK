import math

import pytest

from arx_d_can.examples import example_04_send_position as example


def test_parser_only_exposes_simple_position_options() -> None:
    parser = example.build_parser()
    destinations = {action.dest for action in parser._actions}

    assert parser.parse_args([]).mode == "pv"
    assert parser.parse_args([]).arm_model == "yunyi_v1_0_right"
    assert parser.parse_args(["--mode", "mit"]).mode == "mit"
    assert "config_path" not in destinations


def test_main_uses_blocking_high_level_hold(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeArm:
        joint_names = tuple(f"joint{index}" for index in range(1, 7))

        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def hold_joint_positions(self, positions):
            captured["target"] = tuple(positions)

        def close(self):
            captured["calls"].append("close")

    def fake_arm(**kwargs):
        captured["enable_gripper"] = kwargs["enable_gripper"]
        captured["mode"] = kwargs["control_mode"]
        captured["port"] = kwargs["port"]
        return FakeArm()

    monkeypatch.setattr(example, "ArxDCanArm", fake_arm)
    args = example.build_parser().parse_args(
        ["--mode", "mit", "--positions", "0,10,20,30,40,50"]
    )

    example.main(args)

    assert captured["enable_gripper"] is True
    assert captured["mode"] == "mit"
    assert captured["port"] is None
    assert captured["calls"] == ["connect", "enable", "close"]
    assert captured["target"] == pytest.approx(
        tuple(math.radians(value) for value in (0, 10, 20, 30, 40, 50))
    )
