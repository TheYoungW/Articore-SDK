import math
from types import SimpleNamespace

import pytest

from arx_d_can.examples.dual_arm import (
    example_07_send_position_mit as mit_example,
)


PRODUCT_VELOCITIES_AT_400 = (2.0, 2.0, 3.3, 3.3, 6.3, 6.3, 6.3)


def _fake_arm():
    return SimpleNamespace(
        joint_names=tuple(f"joint{index}" for index in range(1, 8)),
        config=SimpleNamespace(
            product_velocity_at_400=PRODUCT_VELOCITIES_AT_400,
            arm_joints=tuple(
                SimpleNamespace(pv_vlim=20.0)
                for _ in PRODUCT_VELOCITIES_AT_400
            )
        ),
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
        left = _fake_arm()
        right = _fake_arm()

        def connect(self):
            captured["calls"].append("connect")

        def enable(self):
            captured["calls"].append("enable")

        def move_joint_positions(self, *, left, right, **kwargs):
            captured["left"] = tuple(left)
            captured["right"] = tuple(right)
            captured["move_kwargs"] = kwargs
            raise KeyboardInterrupt

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
            "200",
        ]
    )

    example.main(args)

    assert captured["mode"] == expected_mode
    assert captured["calls"] == ["connect", "enable", "close"]
    assert captured["left"] == pytest.approx(
        tuple(math.radians(value) for value in (0, 10, 20, 30, 40, 50, 60))
    )
    assert captured["right"] == pytest.approx(
        tuple(math.radians(value) for value in (0, -10, -20, -30, -40, -50, -60))
    )
    expected = tuple(value * 0.5 for value in PRODUCT_VELOCITIES_AT_400)
    assert captured["move_kwargs"] == {"velocity": pytest.approx(expected)}


def test_pv_example_forwards_product_speed_to_native_trajectory(monkeypatch) -> None:
    captured = {}

    class FakeRobot:
        left = _fake_arm()
        right = _fake_arm()

        def connect(self):
            pass

        def enable(self):
            pass

        def move_joint_positions(self, **kwargs):
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
            "100",
        ]
    )

    pv_example.main(args)

    expected = tuple(value * 0.25 for value in PRODUCT_VELOCITIES_AT_400)
    assert captured["velocity"] == pytest.approx(expected)


@pytest.mark.parametrize(
    ("level", "expected"),
    (
        (100.0, (0.5, 0.5, 0.825, 0.825, 1.575, 1.575, 1.575)),
        (200.0, (1.0, 1.0, 1.65, 1.65, 3.15, 3.15, 3.15)),
        (400.0, PRODUCT_VELOCITIES_AT_400),
    ),
)
def test_product_speed_levels_are_independent_of_urdf_limits(
    level: float,
    expected: tuple[float, ...],
) -> None:
    from arx_d_can.examples.dual_arm.common import scaled_joint_velocities

    assert scaled_joint_velocities(_fake_arm(), level) == pytest.approx(expected)


def test_pv_example_requires_a_zero_to_four_hundred_speed_level() -> None:
    parser = pv_example.build_parser()

    with pytest.raises(SystemExit):
        parser.parse_args(["--left", "0,0,0,0,0,0,0", "--right", "0,0,0,0,0,0,0"])
    with pytest.raises(SystemExit):
        parser.parse_args(
            [
                "--left", "0,0,0,0,0,0,0",
                "--right", "0,0,0,0,0,0,0",
                "--velocity", "401",
            ]
        )


def test_mit_example_uses_the_same_product_trajectory_speed(monkeypatch) -> None:
    captured = {}

    class FakeRobot:
        left = _fake_arm()
        right = _fake_arm()

        def connect(self):
            pass

        def enable(self):
            pass

        def move_joint_positions(self, **kwargs):
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
            "100",
        ]
    )

    mit_example.main(args)

    assert set(captured) == {"left", "right", "velocity"}
    assert captured["velocity"] == pytest.approx(
        tuple(value * 0.25 for value in PRODUCT_VELOCITIES_AT_400)
    )
    assert "profile" not in captured


@pytest.mark.parametrize("example", (pv_example, mit_example))
def test_dual_position_example_has_no_mode_option(example) -> None:
    parser = example.build_parser()
    destinations = {action.dest for action in parser._actions}

    assert "mode" not in destinations


def test_mit_example_exposes_product_speed_but_not_raw_mit_parameters() -> None:
    parser = mit_example.build_parser()
    destinations = {action.dest for action in parser._actions}

    assert destinations == {"help", "left", "right", "velocity"}
    assert not {"kp", "kd", "torque", "target_velocity"} & destinations
