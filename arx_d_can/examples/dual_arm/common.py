"""双臂示例的参数与角度解析。"""
from __future__ import annotations

import math


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


def speed_level(text: str) -> float:
    """解析 0～400 产品速度档位。"""
    value = float(text)
    if not math.isfinite(value) or not 0.0 <= value <= 400.0:
        raise ValueError("速度档位必须是 0～400 的有限数值")
    return value


def positive_velocity_degrees(text: str) -> float:
    """解析高级轨迹示例的正数速度，并从度/秒转换为弧度/秒。"""
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("速度必须是有限正数")
    return math.radians(value)


def scaled_joint_velocities(arm, level: float) -> tuple[float, ...]:
    """按 Yunyi 独立产品速度曲线把 0～400 档换算为 rad/s。"""
    if not math.isfinite(level) or not 0.0 <= level <= 400.0:
        raise ValueError("速度档位必须是 0～400 的有限数值")
    profile = arm.config.product_velocity_at_400
    if len(profile) != len(arm.joint_names):
        raise RuntimeError("机型配置缺少完整的产品速度曲线")
    scale = level / 400.0
    return tuple(float(value) * scale for value in profile)


def gripper_opening(text: str) -> float:
    """解析夹爪 0～1000 开合度；0 闭合，1000 打开。"""
    value = float(text)
    if not math.isfinite(value) or not 0.0 <= value <= 1000.0:
        raise ValueError("夹爪开合度必须是 0～1000 的有限数值")
    return value
