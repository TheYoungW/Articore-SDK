"""重力补偿兼容层使用的只读样本类型。"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True, frozen=True)
class GravityCompensationSample:
    """一侧机械臂的反馈和 Runtime 最终发送的重力前馈。"""

    elapsed_s: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    commanded_torques: tuple[float, ...]
    limited_joints: tuple[str, ...] = ()
    clipped_joints: tuple[str, ...] = ()


__all__: list[str] = []
