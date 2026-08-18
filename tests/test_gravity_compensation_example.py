from __future__ import annotations

import pytest

from arx_d_can.examples.single_arm import example_11_gravity_compensation as example


def test_single_gravity_example_uses_conservative_transition() -> None:
    args = example.build_parser().parse_args([])

    assert args.arm_model == "yunyi_v1_0_right"
    assert args.transition_ms == 1000


@pytest.mark.parametrize("value", ("-1", "60001"))
def test_single_gravity_example_rejects_invalid_transition(value: str) -> None:
    with pytest.raises(SystemExit):
        example.build_parser().parse_args(["--transition-ms", value])
