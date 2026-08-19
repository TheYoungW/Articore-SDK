"""ARX-D-CAN 示例共用的辅助函数。"""
from __future__ import annotations

import math
from argparse import ArgumentParser
from pathlib import Path

from arx_d_can.driver import SUPPORTED_TRANSPORTS


DEFAULT_BAUD = 1_000_000
DEFAULT_ARM_MODEL = "yunyi_v1_0_right"


def add_connection_arguments(
    parser: ArgumentParser,
    *,
    allow_custom_config: bool = False,
    default_arm_model: str | None = DEFAULT_ARM_MODEL,
) -> None:
    """添加连接参数；普通示例默认只向用户展示内置机型。"""
    profile = (
        parser.add_mutually_exclusive_group()
        if allow_custom_config
        else parser
    )
    profile.add_argument(
        "--arm-model",
        default=default_arm_model,
        help=f"机械臂机型；默认 {default_arm_model or '使用 SDK 默认配置'}",
    )
    if allow_custom_config:
        profile.add_argument(
            "--config-path",
            "--hardware-config",
            dest="config_path",
            type=Path,
            default=None,
            help="自定义机械臂硬件 YAML，不能与 --arm-model 同时使用",
        )
    parser.add_argument(
        "--port",
        "--channel",
        dest="port",
        default=None,
        help="通信通道，例如 0、1、/dev/ttyACM0 或 can0；默认使用机型配置",
    )
    parser.add_argument(
        "--transport",
        choices=SUPPORTED_TRANSPORTS,
        default=None,
        help="通信后端；默认使用机型配置",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=None,
        help=f"USB2CAN 串口波特率；默认使用机型配置（arx_d_can 为 {DEFAULT_BAUD}）",
    )


def parse_joint_positions(text: str, *, expected_count: int) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != expected_count:
        raise ValueError(
            f"expected {expected_count} comma-separated joint positions, got {len(values)}"
        )
    return values


def parse_joint_positions_degrees(
    text: str,
    *,
    expected_count: int,
) -> tuple[float, ...]:
    """解析用户输入的角度值，并返回 SDK 使用的弧度值。"""
    return tuple(
        math.radians(value)
        for value in parse_joint_positions(text, expected_count=expected_count)
    )


def positive_velocity_degrees(text: str) -> float:
    """解析正数速度，并从度/秒转换为弧度/秒。"""
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("速度必须是有限正数")
    return math.radians(value)


def speed_percent(text: str) -> float:
    """解析普通位置控制的 0～100 速度档位。"""
    value = float(text)
    if not math.isfinite(value) or not 0.0 <= value <= 100.0:
        raise ValueError("速度档位必须在 0～100 范围内")
    return value
