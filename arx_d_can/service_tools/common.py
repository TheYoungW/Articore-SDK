"""维护工具共用的连接和关节参数辅助函数。"""
from __future__ import annotations

import math
import time
from argparse import ArgumentParser, Namespace
from collections.abc import Sequence
from pathlib import Path

from arx_d_can import ArxDCanArm
from arx_d_can.driver import SUPPORTED_TRANSPORTS


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
DEFAULT_HZ = 100.0
DEFAULT_ARM_MODEL = "yunyi_v1_0_right"


def add_connection_arguments(
    parser: ArgumentParser,
    *,
    allow_custom_config: bool = False,
    default_arm_model: str | None = DEFAULT_ARM_MODEL,
) -> None:
    """为高级维护工具添加机型和连接参数。"""
    profile = parser.add_mutually_exclusive_group() if allow_custom_config else parser
    profile.add_argument("--arm-model", default=default_arm_model)
    if allow_custom_config:
        profile.add_argument(
            "--config-path",
            "--hardware-config",
            dest="config_path",
            type=Path,
            default=None,
        )
    parser.add_argument("--port", "--channel", dest="port", default=None)
    parser.add_argument("--transport", choices=SUPPORTED_TRANSPORTS, default=None)
    parser.add_argument("--baud", type=int, default=None)


def make_arm(
    args: Namespace | None = None,
    *,
    enable_gripper: bool | None = None,
    control_mode: str = "posvel",
) -> ArxDCanArm:
    """按照维护工具参数创建通用机械臂控制器。"""
    if args is None:
        return ArxDCanArm(
            port=DEFAULT_PORT,
            baud=DEFAULT_BAUD,
            transport="dm-serial",
            control_mode=control_mode,
            enable_gripper=enable_gripper,
        )
    return ArxDCanArm(
        model=args.arm_model,
        config_path=getattr(args, "config_path", None),
        port=args.port,
        baud=args.baud,
        transport=getattr(args, "transport", None),
        control_mode=control_mode,
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
    """解析用户输入的角度值，并返回 SDK 使用的弧度值。"""
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
        arm._submit_joint_positions(positions)
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
        arm._submit_joint_positions(target)
        time.sleep(period)
    arm._submit_joint_positions(end_values)
