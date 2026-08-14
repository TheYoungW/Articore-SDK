"""双臂示例的参数与角度解析。"""
from __future__ import annotations

import math


MAX_MIT_VELOCITY_DEG_S = 200.0


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


def positive_velocity_degrees(text: str) -> float:
    """解析正数速度，并从度/秒转换为弧度/秒。"""
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("速度必须是有限正数")
    return math.radians(value)


def mit_velocity_degrees(text: str) -> float:
    """解析普通 MIT 统一速度，命令行单位为度/秒。"""
    value = float(text)
    if (
        not math.isfinite(value)
        or value <= 0.0
        or value > MAX_MIT_VELOCITY_DEG_S
    ):
        raise ValueError("MIT 速度必须在 (0, 200] 度/秒范围内")
    return math.radians(value)


def gripper_opening(text: str) -> float:
    """解析夹爪 0～1000 开合度；0 闭合，1000 打开。"""
    value = float(text)
    if not math.isfinite(value) or not 0.0 <= value <= 1000.0:
        raise ValueError("夹爪开合度必须是 0～1000 的有限数值")
    return value
