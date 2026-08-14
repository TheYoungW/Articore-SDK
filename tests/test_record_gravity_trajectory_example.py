from __future__ import annotations

from types import SimpleNamespace

import pytest

from arx_d_can.examples.single_arm import (
    example_13_record_gravity_trajectory as example,
)


def test_parser_exposes_only_recording_inputs(tmp_path) -> None:
    output = tmp_path / "trajectories" / "demo.json"

    args = example.build_parser().parse_args(
        [
            "--arm-model",
            "yunyi_v1_0_left",
            "--output",
            str(output),
            "--seconds",
            "12",
            "--hz",
            "200",
        ]
    )

    assert args.arm_model == "yunyi_v1_0_left"
    assert args.output == output
    assert args.seconds == 12.0
    assert args.hz == 200.0


def test_parser_rejects_non_positive_duration() -> None:
    with pytest.raises(SystemExit):
        example.build_parser().parse_args(
            ["--output", "demo.json", "--seconds", "0"]
        )


def test_main_records_and_creates_output_folder(monkeypatch, tmp_path) -> None:
    events: list[object] = []

    class FakeArm:
        joint_names = ("joint1", "joint2")

        def __init__(self, **kwargs) -> None:
            events.append(("arm", kwargs))

    class FakeGravity:
        def __init__(self, arm, *, hz) -> None:
            events.append(("gravity", arm, hz))

        def __enter__(self):
            events.append("enter")
            return self

        def __exit__(self, *_args) -> None:
            events.append("exit")

    def fake_record(arm, *, seconds, hz, gravity_mode):
        events.append(("record", arm, seconds, hz, gravity_mode))
        return [0.0, 0.01], [[0.1, 0.2, 0.3], [0.2, 0.3, 0.4]]

    def fake_save(path, hz, positions, *, timestamps, joint_names) -> None:
        events.append(
            ("save", path, hz, positions, timestamps, joint_names)
        )

    monkeypatch.setattr(example, "ArxDCanArm", FakeArm)
    monkeypatch.setattr(example, "GravityCompensationMode", FakeGravity)
    monkeypatch.setattr(example, "record", fake_record)
    monkeypatch.setattr(example, "save_trajectory", fake_save)
    monkeypatch.setattr(example.time, "sleep", lambda _seconds: None)

    output = tmp_path / "trajectories" / "demo.json"
    args = SimpleNamespace(
        arm_model="yunyi_v1_0_left",
        port=None,
        transport=None,
        baud=None,
        output=output,
        seconds=1.0,
        hz=100.0,
    )

    example.main(args)

    assert output.parent.is_dir()
    assert "enter" in events
    assert "exit" in events
    assert events[-1] == (
        "save",
        output,
        100.0,
        [[0.1, 0.2, 0.3], [0.2, 0.3, 0.4]],
        [0.0, 0.01],
        ("joint1", "joint2"),
    )
