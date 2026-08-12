from __future__ import annotations

import pytest

from arx_d_can.examples.dual_arm import (
    example_02_switch_control_mode as example,
)


def test_switch_mode_example_requires_supported_mode() -> None:
    parser = example.build_parser()

    with pytest.raises(SystemExit):
        parser.parse_args([])
    with pytest.raises(SystemExit):
        parser.parse_args(["--mode", "velocity"])


@pytest.mark.parametrize("mode", ("pv", "mit"))
def test_switch_mode_example_configures_both_arms(monkeypatch, mode: str) -> None:
    captured: dict[str, object] = {"calls": []}

    class FakeRobot:
        def connect(self) -> None:
            captured["calls"].append("connect")

        def configure_mode(self, requested_mode: str) -> None:
            captured["mode"] = requested_mode

        def close(self, *, disable: bool = True) -> None:
            captured["calls"].append(("close", disable))

    monkeypatch.setattr(example, "ArxDCanDualArm", FakeRobot)
    args = example.build_parser().parse_args(["--mode", mode])

    example.main(args)

    assert captured["mode"] == mode
    assert captured["calls"] == ["connect", ("close", False)]
