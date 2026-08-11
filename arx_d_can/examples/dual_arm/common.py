"""双臂示例的参数与角度解析。"""
from __future__ import annotations

import math
from argparse import ArgumentParser

from arx_d_can.driver import SUPPORTED_TRANSPORTS


def add_connection_arguments(parser: ArgumentParser) -> None:
    """添加两条独立 CAN 通道的连接参数。"""
    parser.add_argument("--left-channel", default=None)
    parser.add_argument("--right-channel", default=None)
    parser.add_argument("--transport", choices=SUPPORTED_TRANSPORTS, default=None)
    parser.add_argument("--baud", type=int, default=None)


def joint_degrees(text: str) -> tuple[float, ...]:
    """解析 Yunyi 单侧 7 个关节角度并转换为弧度。"""
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != 7:
        raise ValueError(f"必须提供 7 个关节角度，当前为 {len(values)} 个")
    return tuple(math.radians(value) for value in values)
