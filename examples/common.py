"""Shared helpers for ARX-D-CAN examples."""
from __future__ import annotations

import math
import time
from argparse import ArgumentParser, Namespace
from collections.abc import Sequence

from arx_d_can import ArxDCanArm


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
DEFAULT_HZ = 100.0
ZERO_ARM_POSITION = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def add_connection_arguments(parser: ArgumentParser) -> None:
    parser.add_argument("--port", default=DEFAULT_PORT, help="USB2CAN serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="USB2CAN serial baudrate")


def make_arm(args: Namespace | None = None, *, enable_gripper: bool = False) -> ArxDCanArm:
    return ArxDCanArm(
        port=DEFAULT_PORT if args is None else args.port,
        baud=DEFAULT_BAUD if args is None else args.baud,
        control_mode="posvel",
        enable_gripper=enable_gripper,
    )


def parse_joint_positions(text: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != 6:
        raise ValueError(f"expected 6 comma-separated joint positions, got {len(values)}")
    return values


def parse_joint_positions_degrees(text: str) -> tuple[float, ...]:
    """Parse six user-facing degree values and return SDK-facing radians."""
    return tuple(math.radians(value) for value in parse_joint_positions(text))


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
