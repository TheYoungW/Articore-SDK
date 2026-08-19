import math

import pytest

from arx_d_can.examples.dual_arm import (
    example_07_send_position_mit as mit_example,
)
from arx_d_can.examples.dual_arm import (
    example_06_send_position_pv as pv_example,
)


@pytest.mark.parametrize(
    ("example", "expected_mode"),
    ((pv_example, "pv"), (mit_example, "mit")),
)
def test_dual_position_example_fixes_control_mode(
    monkeypatch,
    example,
    expected_mode,
) -> None:
    captured = {"calls": []}

    class FakeRobot:
        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def _set(self, mode, *, left, right, **kwargs):
            captured["method"] = mode
            captured["left"] = tuple(left)
            captured["right"] = tuple(right)
            captured["set_kwargs"] = kwargs
            raise KeyboardInterrupt

        def set_joint_pv(self, **kwargs):
            self._set("pv", **kwargs)

        def set_joint_mit(self, **kwargs):
            self._set("mit", **kwargs)

        def close(self):
            captured["calls"].append("close")

    def fake_robot(**kwargs):
        captured["mode"] = kwargs["control_mode"]
        return FakeRobot()

    monkeypatch.setattr(example, "ArxDCanDualArm", fake_robot)
    monkeypatch.setattr(
        example.time,
        "sleep",
        lambda _seconds: (_ for _ in ()).throw(KeyboardInterrupt),
    )
    args = example.build_parser().parse_args(
        [
            "--left",
            "0,10,20,30,40,50,60",
            "--right",
            "0,-10,-20,-30,-40,-50,-60",
            "--velocity",
            "60",
        ]
    )

    example.main(args)

    assert captured["mode"] == expected_mode
    assert captured["method"] == expected_mode
    assert captured["calls"] == ["connect", "enable", "close"]
    assert captured["left"] == pytest.approx(
        tuple(math.radians(value) for value in (0, 10, 20, 30, 40, 50, 60))
    )
    assert captured["right"] == pytest.approx(
        tuple(math.radians(value) for value in (0, -10, -20, -30, -40, -50, -60))
    )
    assert captured["set_kwargs"] == {"velocity": 60.0}


def test_pv_example_forwards_shared_speed_percent(monkeypatch) -> None:
    captured = {}

    class FakeRobot:
        def connect(self):
            pass

        def enable(self):
            pass

        def set_joint_pv(self, **kwargs):
            captured.update(kwargs)
            raise KeyboardInterrupt

        def close(self):
            pass

    monkeypatch.setattr(pv_example, "ArxDCanDualArm", lambda **_kwargs: FakeRobot())
    args = pv_example.build_parser().parse_args(
        [
            "--left",
            "0,10,20,30,40,50,60",
            "--right",
            "0,-10,-20,-30,-40,-50,-60",
            "--velocity",
            "90",
        ]
    )

    pv_example.main(args)

    assert captured["velocity"] == 90.0


def test_pv_example_requires_a_zero_to_one_hundred_speed() -> None:
    parser = pv_example.build_parser()

    with pytest.raises(SystemExit):
        parser.parse_args(["--left", "0,0,0,0,0,0,0", "--right", "0,0,0,0,0,0,0"])
    paused = parser.parse_args(
        [
            "--left", "0,0,0,0,0,0,0",
            "--right", "0,0,0,0,0,0,0",
            "--velocity", "0",
        ]
    )
    assert paused.velocity == 0.0
    with pytest.raises(SystemExit):
        parser.parse_args(
            [
                "--left", "0,0,0,0,0,0,0",
                "--right", "0,0,0,0,0,0,0",
                "--velocity", "100.1",
            ]
        )


def test_mit_example_uses_the_same_shared_velocity(monkeypatch) -> None:
    captured = {}

    class FakeRobot:
        def connect(self):
            pass

        def enable(self):
            pass

        def set_joint_mit(self, **kwargs):
            captured.update(kwargs)
            raise KeyboardInterrupt

        def close(self):
            pass

    monkeypatch.setattr(mit_example, "ArxDCanDualArm", lambda **_kwargs: FakeRobot())
    monkeypatch.setattr(
        mit_example.time,
        "sleep",
        lambda _seconds: (_ for _ in ()).throw(KeyboardInterrupt),
    )
    args = mit_example.build_parser().parse_args(
        [
            "--left",
            "0,10,20,30,40,50,60",
            "--right",
            "0,-10,-20,-30,-40,-50,-60",
            "--velocity",
            "45",
        ]
    )

    mit_example.main(args)

    assert set(captured) == {"left", "right", "velocity"}
    assert captured["velocity"] == 45.0
    assert "profile" not in captured


def test_mit_example_rejects_speed_above_one_hundred() -> None:
    parser = mit_example.build_parser()

    with pytest.raises(SystemExit):
        parser.parse_args(
            [
                "--left",
                "0,0,0,0,0,0,0",
                "--right",
                "0,0,0,0,0,0,0",
                "--velocity",
                "100.01",
            ]
        )


@pytest.mark.parametrize("example", (pv_example, mit_example))
def test_dual_position_example_has_no_mode_option(example) -> None:
    parser = example.build_parser()
    destinations = {action.dest for action in parser._actions}

    assert "mode" not in destinations


def test_mit_example_exposes_shared_speed_but_not_raw_mit_parameters() -> None:
    parser = mit_example.build_parser()
    destinations = {action.dest for action in parser._actions}

    assert destinations == {"help", "left", "right", "velocity"}
    assert not {"kp", "kd", "torque", "target_velocity"} & destinations
