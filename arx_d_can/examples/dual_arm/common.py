"""双臂示例的参数与角度解析。"""
from __future__ import annotations

import math

from arx_d_can.examples.gripper_arguments import gripper_opening


def joint_values(text: str, *, name: str = "关节参数") -> tuple[float, ...]:
    """解析 Yunyi 单侧 7 个有限浮点参数。"""
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != 7:
        raise ValueError(f"{name}必须提供 7 个值，当前为 {len(values)} 个")
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{name}必须全部为有限值")
    return values


def joint_degrees(text: str) -> tuple[float, ...]:
    """解析 Yunyi 单侧 7 个关节角度并转换为弧度。"""
    return tuple(math.radians(value) for value in joint_values(text, name="关节角度"))


def speed_percent(text: str) -> float:
    """解析普通位置控制的 0～100 速度档位。"""
    value = float(text)
    if not math.isfinite(value) or not 0.0 <= value <= 100.0:
        raise ValueError("速度档位必须在 0～100 范围内")
    return value
