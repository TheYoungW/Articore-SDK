import math

import pytest

from arx_d_can.examples.control import example_03_send_position_pv as pv_example
from arx_d_can.examples.control import example_04_send_position_mit as mit_example


def test_pv_example_sets_default_positions_then_waits_to_disable(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def set_max_acceleration(self, value):
            captured["max_acceleration"] = value

        def set_joint_pv(self, **kwargs):
            captured.update(kwargs)

        def disable(self):
            captured["calls"].append("disable")

        def disconnect(self):
            captured["calls"].append("disconnect")

    def fake_robot(**kwargs):
        captured["mode"] = kwargs["control_mode"]
        return FakeRobot()

    monkeypatch.setattr(pv_example, "ArxDCanDualArm", fake_robot)
    monkeypatch.setattr(
        "builtins.input",
        lambda prompt: captured["calls"].append("input") or "",
    )
    args = pv_example.build_parser().parse_args([
        "--velocity", "70",
        "--max-acceleration", "4.56",
    ])
    pv_example.main(args)

    assert captured["mode"] == "pv"
    assert captured["calls"] == ["connect", "enable", "input", "disable", "disconnect"]
    assert captured["left"] == pytest.approx(
        tuple(math.radians(value) for value in (0, 0, 0, 90, 0, 0, 0))
    )
    assert captured["right"] == pytest.approx(
        tuple(math.radians(value) for value in (0, 0, 0, 90, 0, 0, 0))
    )
    assert captured["max_acceleration"] == pytest.approx(4.56)
    assert captured["velocity"] == 70.0


def test_pv_example_defaults_to_native_acceleration_limit() -> None:
    parser = pv_example.build_parser()
    defaults = parser.parse_args([])

    assert defaults.left == pv_example.DEFAULT_JOINT_TARGET_DEGREES
    assert defaults.right == pv_example.DEFAULT_JOINT_TARGET_DEGREES
    assert not hasattr(defaults, "max_speed")
    assert defaults.max_acceleration == pytest.approx(6.0)
    assert defaults.velocity == pytest.approx(50.0)
    assert parser.parse_args(
        ["--max-acceleration", "4.565"]
    ).max_acceleration == pytest.approx(4.565)
    assert parser.parse_args(["--velocity", "100"]).velocity == 100.0
    with pytest.raises(SystemExit):
        parser.parse_args(["--velocity", "-0.1"])


def test_mit_example_forwards_positions_and_speed(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def set_joint_mit(self, **kwargs):
            captured.update(kwargs)
            raise KeyboardInterrupt

        def disconnect(self):
            captured["calls"].append("disconnect")

    def fake_robot(**kwargs):
        captured["mode"] = kwargs["control_mode"]
        return FakeRobot()

    monkeypatch.setattr(mit_example, "ArxDCanDualArm", fake_robot)
    args = mit_example.build_parser().parse_args(
        [
            "--left", "0,0,0,90,0,0,0",
            "--right", "0,0,0,90,0,0,0",
            "--velocity", "10",
        ]
    )
    mit_example.main(args)

    expected_positions = tuple(
        math.radians(value) for value in (0, 0, 0, 90, 0, 0, 0)
    )
    assert captured["mode"] == "mit"
    assert captured["calls"] == ["connect", "enable", "disconnect"]
    assert captured["left"] == pytest.approx(expected_positions)
    assert captured["right"] == pytest.approx(expected_positions)
    assert captured["velocity"] == 10.0


def test_mit_example_requires_a_zero_to_one_hundred_speed() -> None:
    parser = mit_example.build_parser()
    base = ["--left", "0,0,0,0,0,0,0", "--right", "0,0,0,0,0,0,0"]

    with pytest.raises(SystemExit):
        parser.parse_args(base)
    assert parser.parse_args([*base, "--velocity", "0"]).velocity == 0.0
    with pytest.raises(SystemExit):
        parser.parse_args([*base, "--velocity", "100.1"])


@pytest.mark.parametrize("example", (pv_example, mit_example))
def test_dual_position_example_has_no_mode_option(example) -> None:
    destinations = {action.dest for action in example.build_parser()._actions}
    assert "mode" not in destinations


def test_mit_example_requires_positions_and_exposes_ordinary_speed() -> None:
    parser = mit_example.build_parser()
    destinations = {action.dest for action in parser._actions}

    with pytest.raises(SystemExit):
        parser.parse_args([])
    assert destinations == {
        "help",
        "left",
        "right",
        "velocity",
    }
