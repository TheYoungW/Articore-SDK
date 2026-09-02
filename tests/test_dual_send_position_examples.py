import math
from types import SimpleNamespace

import pytest

from arx_d_can.examples.control import example_03_send_position_pv as pv_example
from arx_d_can.examples.control import example_04_send_position_mit as mit_example
from arx_d_can.examples.control import (
    example_17_send_position_mit_fast as fast_example,
)


def test_pv_example_sets_default_positions_then_waits_to_disable(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

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
    args = pv_example.build_parser().parse_args(["--velocity", "70"])
    pv_example.main(args)

    assert captured["mode"] == "pv"
    assert captured["calls"] == [
        "connect",
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


def test_pv_example_exposes_only_velocity_control() -> None:
    parser = pv_example.build_parser()
    defaults = parser.parse_args([])

    assert defaults.left == pv_example.DEFAULT_JOINT_TARGET_DEGREES
    assert defaults.right == pv_example.DEFAULT_JOINT_TARGET_DEGREES
    assert not hasattr(defaults, "max_speed")
    assert not hasattr(defaults, "max_acceleration")
    assert defaults.velocity == pytest.approx(50.0)
    assert parser.parse_args(["--velocity", "100"]).velocity == 100.0
    with pytest.raises(SystemExit):
        parser.parse_args(["--velocity", "0"])


def test_ordinary_mit_example_stages_targets_from_feedback(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def read_state(self):
            captured["calls"].append("read_state")
            arm = lambda: SimpleNamespace(positions=(0.0,) * 7)
            return SimpleNamespace(left=arm(), right=arm())

        def enable(self):
            captured["calls"].append("enable")
            return True

        def set_joint_mit(self, **kwargs):
            captured["calls"].append(("set_joint_mit", kwargs))

        def disable(self):
            captured["calls"].append("disable")

        def disconnect(self):
            captured["calls"].append("disconnect")

    def fake_robot(**kwargs):
        captured["mode"] = kwargs["control_mode"]
        return FakeRobot()

    monkeypatch.setattr(mit_example, "ArxDCanDualArm", fake_robot)
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt: captured["calls"].append("input") or "",
    )
    monkeypatch.setattr(
        mit_example.time,
        "sleep",
        lambda seconds: captured["calls"].append(("sleep", seconds)),
    )
    args = mit_example.build_parser().parse_args(
        [
            "--left", "5,0,0,0,0,0,0",
            "--right", "0,0,0,0,0,0,0",
            "--kp", "20",
            "--kd", "1",
            "--max-step-deg", "2",
            "--step-interval", "0.25",
            "--stream-hz", "4",
        ]
    )
    mit_example.main(args)

    commands = [
        call[1]
        for call in captured["calls"]
        if isinstance(call, tuple) and call[0] == "set_joint_mit"
    ]
    assert captured["mode"] == "mit"
    assert len(commands) == 3
    assert [
        math.degrees(command["left_positions"][0]) for command in commands
    ] == pytest.approx(
        [5 / 3, 10 / 3, 5]
    )
    assert all(
        command["right_positions"] == pytest.approx((0.0,) * 7)
        for command in commands
    )
    assert all(command["left_velocities"] == (0.0,) * 7 for command in commands)
    assert all(command["right_velocities"] == (0.0,) * 7 for command in commands)
    assert all(command["kp"] == 20.0 and command["kd"] == 1.0 for command in commands)
    assert captured["calls"] == [
        "connect",
        "read_state",
        "input",
        "enable",
        ("set_joint_mit", commands[0]),
        ("sleep", 0.25),
        ("set_joint_mit", commands[1]),
        ("sleep", 0.25),
        ("set_joint_mit", commands[2]),
        ("sleep", 0.25),
        "disable",
        "disconnect",
    ]
    assert all("velocity" not in command for command in commands)


def test_fast_mit_example_forwards_positions_and_speed(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def set_joint_mit_fast(self, **kwargs):
            captured.update(kwargs)
            raise KeyboardInterrupt

        def disconnect(self):
            captured["calls"].append("disconnect")

    def fake_robot(**kwargs):
        captured["mode"] = kwargs["control_mode"]
        return FakeRobot()

    monkeypatch.setattr(fast_example, "ArxDCanDualArm", fake_robot)
    args = fast_example.build_parser().parse_args(
        [
            "--left", "0,0,0,90,0,0,0",
            "--right", "0,0,0,90,0,0,0",
            "--velocity", "35",
        ]
    )
    fast_example.main(args)

    expected_positions = tuple(
        math.radians(value) for value in (0, 0, 0, 90, 0, 0, 0)
    )
    assert captured["mode"] == "mit"
    assert captured["calls"] == ["connect", "enable", "disconnect"]
    assert captured["left"] == pytest.approx(expected_positions)
    assert captured["right"] == pytest.approx(expected_positions)
    assert captured["velocity"] == 35.0


def test_ordinary_mit_example_exposes_no_speed() -> None:
    parser = mit_example.build_parser()
    base = [
        "--left", "0,0,0,0,0,0,0",
        "--right", "0,0,0,0,0,0,0",
        "--kp", "20", "--kd", "1",
    ]
    assert not hasattr(parser.parse_args(base), "velocity")
    with pytest.raises(SystemExit):
        parser.parse_args([*base, "--velocity", "10"])


@pytest.mark.parametrize("example", (pv_example, mit_example, fast_example))
def test_dual_position_example_has_no_mode_option(example) -> None:
    destinations = {action.dest for action in example.build_parser()._actions}
    assert "mode" not in destinations


def test_fast_mit_example_requires_positions_and_exposes_speed() -> None:
    parser = fast_example.build_parser()
    destinations = {action.dest for action in parser._actions}

    with pytest.raises(SystemExit):
        parser.parse_args([])
    assert destinations == {
        "help",
        "left",
        "right",
        "velocity",
    }
    base = ["--left", "0,0,0,0,0,0,0", "--right", "0,0,0,0,0,0,0"]
    assert parser.parse_args(base).velocity == 100.0
    assert parser.parse_args([*base, "--velocity", "25"]).velocity == 25.0
    with pytest.raises(SystemExit):
        parser.parse_args([*base, "--velocity", "0"])


def test_ordinary_mit_example_exposes_safe_staging_controls() -> None:
    parser = mit_example.build_parser()
    destinations = {action.dest for action in parser._actions}
    base = [
        "--left", "0,0,0,0,0,0,0",
        "--right", "0,0,0,0,0,0,0",
        "--kp", "20", "--kd", "1",
    ]
    defaults = parser.parse_args(base)

    assert destinations == {
        "help",
        "left",
        "right",
        "kp",
        "kd",
        "max_step_deg",
        "max_total_delta_deg",
        "step_interval",
        "stream_hz",
    }
    assert defaults.max_step_deg == pytest.approx(2.0)
    assert defaults.max_total_delta_deg == pytest.approx(20.0)
    assert defaults.step_interval == pytest.approx(0.5)
    assert defaults.stream_hz == pytest.approx(100.0)
    with pytest.raises(SystemExit):
        parser.parse_args([*base, "--max-step-deg", "0"])
    with pytest.raises(SystemExit):
        parser.parse_args([*base, "--step-interval", "0"])
    with pytest.raises(SystemExit):
        parser.parse_args([*base, "--stream-hz", "0"])
    with pytest.raises(SystemExit):
        parser.parse_args([*base, "--max-total-delta-deg", "0"])


def test_ordinary_mit_example_rejects_large_total_delta_before_enable(
    monkeypatch,
) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def read_state(self):
            captured["calls"].append("read_state")
            arm = SimpleNamespace(positions=(0.0,) * 7)
            return SimpleNamespace(left=arm, right=arm)

        def disconnect(self):
            captured["calls"].append("disconnect")

    monkeypatch.setattr(
        mit_example,
        "ArxDCanDualArm",
        lambda **_kwargs: FakeRobot(),
    )
    args = mit_example.build_parser().parse_args(
            [
                "--left", "30,0,0,0,0,0,0",
                "--right", "0,0,0,0,0,0,0",
                "--kp", "20",
                "--kd", "1",
            ]
    )

    with pytest.raises(ValueError, match="30.00°"):
        mit_example.main(args)

    assert captured["calls"] == ["connect", "read_state", "disconnect"]
