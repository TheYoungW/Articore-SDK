"""Shared helpers for ARX-D-CAN examples."""
from __future__ import annotations

import math
import time
from argparse import ArgumentParser, Namespace
from collections.abc import Sequence
from pathlib import Path

from arx_d_can import ArxDCanArm


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
DEFAULT_HZ = 100.0
ZERO_ARM_POSITION = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def add_connection_arguments(parser: ArgumentParser) -> None:
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument(
        "--arm-model",
        default=None,
        help="Built-in arm model profile name; default: models.yaml default_model",
    )
    profile.add_argument(
        "--config-path",
        type=Path,
        default=None,
        help="Custom arm hardware YAML; cannot be combined with --arm-model",
    )
    parser.add_argument(
        "--port",
        default=None,
        help=f"USB2CAN serial port; default: profile value ({DEFAULT_PORT} for arx_d_can)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=None,
        help=f"USB2CAN serial baudrate; default: profile value ({DEFAULT_BAUD} for arx_d_can)",
    )


def make_arm(args: Namespace | None = None, *, enable_gripper: bool = False) -> ArxDCanArm:
    if args is None:
        return ArxDCanArm(
            port=DEFAULT_PORT,
            baud=DEFAULT_BAUD,
            control_mode="posvel",
            enable_gripper=enable_gripper,
        )
    return ArxDCanArm(
        model=args.arm_model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        control_mode="posvel",
        enable_gripper=enable_gripper,
    )


def parse_joint_positions(text: str, *, expected_count: int = 6) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != expected_count:
        raise ValueError(
            f"expected {expected_count} comma-separated joint positions, got {len(values)}"
        )
    return values


def parse_joint_positions_degrees(
    text: str,
    *,
    expected_count: int = 6,
) -> tuple[float, ...]:
    """Parse user-facing degree values and return SDK-facing radians."""
    return tuple(
        math.radians(value)
        for value in parse_joint_positions(text, expected_count=expected_count)
    )


def send_for_seconds(
    arm: ArxDCanArm,
    positions: Sequence[float],
    *,
    seconds: float,
    hz: float = DEFAULT_HZ,
) -> None:
    period = 1.0 / max(1.0, hz)
    deadline = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < deadline:
        arm.send_joint_positions(positions)
        time.sleep(period)


def interpolate_joint_positions(
    arm: ArxDCanArm,
    start: Sequence[float],
    end: Sequence[float],
    *,
    seconds: float,
    hz: float = DEFAULT_HZ,
) -> None:
    period = 1.0 / max(1.0, hz)
    steps = max(1, int(max(0.0, seconds) * max(1.0, hz)))
    start_values = tuple(float(value) for value in start)
    end_values = tuple(float(value) for value in end)
    if len(start_values) != len(end_values):
        raise ValueError("start and end must have the same length")
    for step in range(1, steps + 1):
        ratio = step / steps
        target = tuple(
            start_value + (end_value - start_value) * ratio
            for start_value, end_value in zip(start_values, end_values)
        )
        arm.send_joint_positions(target)
        time.sleep(period)
    arm.send_joint_positions(end_values)
