import math

import pytest

from arx_d_can.examples import example_06_send_position_pv as pv_example
from arx_d_can.examples import example_07_send_position_mit as mit_example


def test_pv_example_forwards_positions_and_speed(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def set_joint_pv(self, **kwargs):
            captured.update(kwargs)
            raise KeyboardInterrupt

        def disconnect(self):
            captured["calls"].append("disconnect")

    def fake_robot(**kwargs):
        captured["mode"] = kwargs["control_mode"]
        return FakeRobot()

    monkeypatch.setattr(pv_example, "ArxDCanDualArm", fake_robot)
    args = pv_example.build_parser().parse_args(
        [
            "--left", "0,10,20,30,40,50,60",
            "--right", "0,-10,-20,-30,-40,-50,-60",
            "--velocity", "90",
        ]
    )
    pv_example.main(args)

    assert captured["mode"] == "pv"
    assert captured["calls"] == ["connect", "enable", "disconnect"]
    assert captured["left"] == pytest.approx(
        tuple(math.radians(value) for value in (0, 10, 20, 30, 40, 50, 60))
    )
    assert captured["right"] == pytest.approx(
        tuple(math.radians(value) for value in (0, -10, -20, -30, -40, -50, -60))
    )
    assert captured["velocity"] == 90.0


def test_pv_example_requires_a_zero_to_one_hundred_speed() -> None:
    parser = pv_example.build_parser()
    base = ["--left", "0,0,0,0,0,0,0", "--right", "0,0,0,0,0,0,0"]

    with pytest.raises(SystemExit):
        parser.parse_args(base)
    assert parser.parse_args([*base, "--velocity", "0"]).velocity == 0.0
    with pytest.raises(SystemExit):
        parser.parse_args([*base, "--velocity", "100.1"])


def test_mit_example_sends_every_explicit_default(monkeypatch) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def submit_raw_mit(self, **kwargs):
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
        ]
    )
    mit_example.main(args)

    expected_positions = tuple(
        math.radians(value) for value in (0, 0, 0, 90, 0, 0, 0)
    )
    zeros = (0.0,) * 7
    assert captured["mode"] == "mit"
    assert captured["calls"] == ["connect", "enable", "disconnect"]
    assert captured["left_positions"] == pytest.approx(expected_positions)
    assert captured["right_positions"] == pytest.approx(expected_positions)
    assert captured["left_velocities"] == zeros
    assert captured["right_velocities"] == zeros
    assert captured["kp"] == (190.0, 190.0, 70.0, 125.0, 10.0, 22.0, 28.0)
    assert captured["kd"] == (4.55, 4.5, 2.0, 2.9, 0.7, 0.89, 0.84)
    assert captured["left_feedforward_torques"] == zeros
    assert captured["right_feedforward_torques"] == zeros


def test_mit_example_allows_explicit_raw_values() -> None:
    args = mit_example.build_parser().parse_args(
        [
            "--left", "0,0,0,90,0,0,0",
            "--right", "0,0,0,90,0,0,0",
            "--target-velocity", "1,2,3,4,5,6,7",
            "--kp", "1,2,3,4,5,6,7",
            "--kd", "0.1,0.2,0.3,0.4,0.5,0.6,0.7",
            "--feedforward-torque", "0,1,2,3,4,5,6",
        ]
    )

    assert args.target_velocity == pytest.approx(
        tuple(math.radians(value) for value in range(1, 8))
    )
    assert args.kp == tuple(float(value) for value in range(1, 8))
    assert args.kd == pytest.approx((0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7))
    assert args.feedforward_torque == tuple(float(value) for value in range(7))


@pytest.mark.parametrize("example", (pv_example, mit_example))
def test_dual_position_example_has_no_mode_option(example) -> None:
    destinations = {action.dest for action in example.build_parser()._actions}
    assert "mode" not in destinations


def test_mit_example_requires_positions_and_exposes_full_frame() -> None:
    parser = mit_example.build_parser()
    destinations = {action.dest for action in parser._actions}

    with pytest.raises(SystemExit):
        parser.parse_args([])
    assert destinations == {
        "help",
        "left",
        "right",
        "target_velocity",
        "kp",
        "kd",
        "feedforward_torque",
    }
