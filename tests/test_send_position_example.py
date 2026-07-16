import pytest

from arx_d_can.examples import example_04_send_position as example


class FakeArm:
    def __init__(self, *, interrupt_after: int | None = None):
        self.targets = []
        self.velocities = []
        self.velocity_limits = []
        self.torques = []
        self.interrupt_after = interrupt_after

    def send_joint_positions(
        self,
        target,
        *,
        velocities=None,
        velocity_limits=None,
        torques=None,
    ):
        self.targets.append(tuple(target))
        self.velocities.append(velocities)
        self.velocity_limits.append(velocity_limits)
        self.torques.append(torques)
        if self.interrupt_after is not None and len(self.targets) >= self.interrupt_after:
            raise KeyboardInterrupt


def test_zero_hold_seconds_keeps_refreshing_until_interrupted(monkeypatch):
    arm = FakeArm(interrupt_after=3)
    monkeypatch.setattr(example.time, "sleep", lambda _: None)

    with pytest.raises(KeyboardInterrupt):
        example.hold_target(arm, (1.0,) * 6, seconds=0.0, hz=100.0)

    assert arm.targets == [(1.0,) * 6] * 3


def test_positive_hold_seconds_stops_after_deadline(monkeypatch):
    arm = FakeArm()
    clock = {"now": 0.0}

    monkeypatch.setattr(example.time, "monotonic", lambda: clock["now"])
    monkeypatch.setattr(
        example.time,
        "sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )

    example.hold_target(arm, (2.0,) * 6, seconds=0.025, hz=100.0)

    assert arm.targets == [(2.0,) * 6] * 3


def test_parser_defaults_to_pv_and_accepts_mit():
    parser = example.build_parser()

    assert parser.parse_args([]).mode == "pv"
    assert parser.parse_args(["--mode", "mit"]).mode == "mit"
    assert parser.parse_args([]).velocities is None
    assert parser.parse_args([]).velocity_limits is None
    assert parser.parse_args([]).torques is None
    assert parser.parse_args(["--torques", "1,2,3,4,5,6"]).torques == "1,2,3,4,5,6"


def test_hold_target_refreshes_mit_torques(monkeypatch):
    arm = FakeArm(interrupt_after=2)
    torques = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    monkeypatch.setattr(example.time, "sleep", lambda _: None)

    with pytest.raises(KeyboardInterrupt):
        example.hold_target(
            arm,
            (1.0,) * 6,
            torques=torques,
            seconds=0.0,
            hz=100.0,
        )

    assert arm.torques == [torques, torques]


def test_velocity_inputs_are_converted_from_degrees_to_radians():
    values = example.parse_velocities_degrees("180,-90,0,45,90,360")
    limits = example.parse_velocities_degrees(
        "180,90,45,45,90,360",
        require_positive=True,
    )

    assert values == pytest.approx((3.141593, -1.570796, 0.0, 0.785398, 1.570796, 6.283185))
    assert limits == pytest.approx((3.141593, 1.570796, 0.785398, 0.785398, 1.570796, 6.283185))


def test_main_configures_requested_control_mode(monkeypatch):
    captured = {}

    class RuntimeArm:
        def __init__(self, **kwargs):
            captured["constructor"] = kwargs
            captured["calls"] = []

        def connect(self):
            captured["calls"].append("connect")

        def configure(self):
            captured["calls"].append("configure")

        def enable(self):
            captured["calls"].append("enable")

        def send_joint_positions(
            self,
            target,
            *,
            velocities=None,
            velocity_limits=None,
            torques=None,
        ):
            captured["target"] = tuple(target)
            captured["velocities"] = velocities
            captured["velocity_limits"] = velocity_limits
            captured["torques"] = torques

        def close(self):
            captured["calls"].append("close")

    monkeypatch.setattr(example, "ArxDCanArm", RuntimeArm)
    monkeypatch.setattr(example, "hold_target", lambda *_args, **_kwargs: None)
    args = example.build_parser().parse_args(
        [
            "--mode",
            "mit",
            "--velocities",
            "10,20,30,40,50,60",
            "--torques",
            "0.1,0.2,0.3,0.4,0.5,0.6",
            "--hold-seconds",
            "0.01",
        ]
    )

    example.main(args)

    assert captured["constructor"]["control_mode"] == "mit"
    assert captured["calls"] == ["connect", "configure", "enable", "close"]
    assert len(captured["target"]) == 6
    assert captured["velocities"] == pytest.approx(
        tuple(example.math.radians(value) for value in (10, 20, 30, 40, 50, 60))
    )
    assert captured["velocity_limits"] is None
    assert captured["torques"] == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
