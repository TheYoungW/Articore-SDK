import math

import pytest

from arx_d_can.examples.control import example_03_send_position_pv as pv_example
from arx_d_can.examples.control import example_04_send_position_mit as mit_example
from arx_d_can.examples.control import (
    example_17_send_position_mit_fast_follow as fast_follow_example,
)


def test_pv_example_sets_default_positions_then_waits_to_disable(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def set_max_speed(self, value):
            captured["calls"].append(("set_max_speed", value))

        def get_max_speed(self):
            captured["calls"].append("get_max_speed")
            return 1.25

        def set_max_acceleration(self, value):
            captured["calls"].append(("set_max_acceleration", value))

        def get_max_acceleration(self):
            captured["calls"].append("get_max_acceleration")
            return 2.5

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
        "--max-speed", "1.25",
        "--max-acceleration", "2.5",
    ])
    pv_example.main(args)

    assert captured["mode"] == "pv"
    assert captured["calls"] == [
        "connect",
        ("set_max_speed", 1.25),
        "get_max_speed",
        ("set_max_acceleration", 2.5),
        "get_max_acceleration",
        "enable",
        "input",
        "disable",
        "disconnect",
    ]
    assert captured["left"] == pytest.approx(
        tuple(math.radians(value) for value in (0, 0, 0, 90, 0, 0, 0))
    )
    assert captured["right"] == pytest.approx(
        tuple(math.radians(value) for value in (0, 0, 0, 90, 0, 0, 0))
    )
    assert captured["velocity"] == 70.0


def test_pv_example_uses_runtime_limit_defaults() -> None:
    parser = pv_example.build_parser()
    defaults = parser.parse_args([])

    assert defaults.left == pv_example.DEFAULT_JOINT_TARGET_DEGREES
    assert defaults.right == pv_example.DEFAULT_JOINT_TARGET_DEGREES
    assert defaults.max_speed is None
    assert defaults.max_acceleration is None
    assert defaults.velocity == pytest.approx(50.0)
    assert parser.parse_args(["--velocity", "100"]).velocity == 100.0
    with pytest.raises(SystemExit):
        parser.parse_args(["--velocity", "0"])


@pytest.mark.parametrize(
    ("example", "method"),
    (
        (mit_example, "set_joint_mit"),
        (fast_follow_example, "set_joint_mit_fast_follow"),
    ),
)
def test_mit_examples_forward_positions_without_speed(
    monkeypatch, example, method: str
) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def command(self, **kwargs):
            captured.update(kwargs)
            raise KeyboardInterrupt

        def disconnect(self):
            captured["calls"].append("disconnect")

    def fake_robot(**kwargs):
        captured["mode"] = kwargs["control_mode"]
        return FakeRobot()

    setattr(FakeRobot, method, FakeRobot.command)
    monkeypatch.setattr(example, "ArxDCanDualArm", fake_robot)
    args = example.build_parser().parse_args(
        [
            "--left", "0,0,0,90,0,0,0",
            "--right", "0,0,0,90,0,0,0",
        ]
    )
    example.main(args)

    expected_positions = tuple(
        math.radians(value) for value in (0, 0, 0, 90, 0, 0, 0)
    )
    assert captured["mode"] == "mit"
    assert captured["calls"] == ["connect", "enable", "disconnect"]
    assert captured["left"] == pytest.approx(expected_positions)
    assert captured["right"] == pytest.approx(expected_positions)
    assert "velocity" not in captured


@pytest.mark.parametrize("example", (mit_example, fast_follow_example))
def test_mit_examples_require_positions_and_expose_no_speed(example) -> None:
    parser = example.build_parser()
    base = ["--left", "0,0,0,0,0,0,0", "--right", "0,0,0,0,0,0,0"]

    assert not hasattr(parser.parse_args(base), "velocity")
    with pytest.raises(SystemExit):
        parser.parse_args([*base, "--velocity", "10"])


@pytest.mark.parametrize("example", (pv_example, mit_example, fast_follow_example))
def test_dual_position_example_has_no_mode_option(example) -> None:
    destinations = {action.dest for action in example.build_parser()._actions}
    assert "mode" not in destinations


@pytest.mark.parametrize("example", (mit_example, fast_follow_example))
def test_mit_example_requires_positions_only(example) -> None:
    parser = example.build_parser()
    destinations = {action.dest for action in parser._actions}

    with pytest.raises(SystemExit):
        parser.parse_args([])
    assert destinations == {
        "help",
        "left",
        "right",
    }
