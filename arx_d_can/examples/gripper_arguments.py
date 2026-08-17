"""单臂和双臂夹爪示例共用的命令行参数。"""
from __future__ import annotations

import math
from argparse import ArgumentParser

from arx_d_can import GripperForceLevel


def gripper_opening(text: str) -> float:
    """解析 0～1000 开合度；0 闭合，1000 打开。"""
    value = float(text)
    if not math.isfinite(value) or not 0.0 <= value <= 1000.0:
        raise ValueError("夹爪开合度必须是 0～1000 的有限数值")
    return value


def gripper_speed(text: str) -> float:
    """解析产品归一化速度；1000 对应最大标定速度。"""
    value = float(text)
    if not math.isfinite(value) or not 0.0 < value <= 1000.0:
        raise ValueError("夹爪速度必须是 (0, 1000] 的有限数值")
    return value


def gripper_force_level(text: str) -> GripperForceLevel:
    """解析 1～10 档夹持力。"""
    try:
        return GripperForceLevel(int(text))
    except (TypeError, ValueError) as exc:
        raise ValueError("夹爪力矩档位必须是 1～10 的整数") from exc


def add_gripper_profile_arguments(parser: ArgumentParser) -> None:
    """添加速度和十档夹持力参数。"""
    parser.add_argument(
        "--speed",
        type=gripper_speed,
        default=1000.0,
        help="归一化速度 (0, 1000]；默认 1000（最大标定速度）",
    )
    parser.add_argument(
        "--force-level",
        type=gripper_force_level,
        default=GripperForceLevel.LEVEL_5,
        metavar="1..10",
        help="夹持力档位 1～10；默认 5，10 最强",
    )


__all__ = [
    "add_gripper_profile_arguments",
    "gripper_force_level",
    "gripper_opening",
    "gripper_speed",
]
