"""高层 SDK 公开的状态与遥测数据对象。"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True, frozen=True)
class DualArmGripperState:
    """双臂产品夹爪的简化用户状态。"""

    opening: float
    gripper_level: int
    enabled: bool | None = None


@dataclass(slots=True, frozen=True)
class JointState:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    torques: tuple[float, ...]
    enabled: tuple[bool | None, ...] = (None,) * 7


@dataclass(slots=True, frozen=True)
class ArxDCanState:
    arm: JointState
    gripper: DualArmGripperState | None = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.arm.names

    @property
    def positions(self) -> tuple[float, ...]:
        return self.arm.positions


__all__ = [
    "ArxDCanState",
    "DualArmGripperState",
    "JointState",
]
