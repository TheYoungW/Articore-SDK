"""高层 SDK 公开的状态与遥测数据对象。"""
from __future__ import annotations

from dataclasses import dataclass

@dataclass(slots=True, frozen=True)
class MotorState:
    name: str
    motor_id: int
    feedback_id: int
    position: float
    velocity: float
    torque: float = 0.0


@dataclass(slots=True, frozen=True)
class GripperState:
    """同时包含用户坐标与电机坐标的夹爪反馈。"""

    name: str
    motor_id: int
    feedback_id: int
    opening: float
    motor_position: float
    motor_velocity: float
    torque: float = 0.0

    @property
    def position(self) -> float:
        """返回为兼容现有代码和高级控制而保留的原始电机位置。"""
        return self.motor_position

    @property
    def velocity(self) -> float:
        """返回为兼容现有代码和高级控制而保留的原始电机速度。"""
        return self.motor_velocity


@dataclass(slots=True, frozen=True)
class DualArmGripperState:
    """双臂产品夹爪的简化用户状态。"""

    opening: float
    gripper_level: int


@dataclass(slots=True, frozen=True)
class JointState:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    torques: tuple[float, ...]


@dataclass(slots=True, frozen=True)
class ArxDCanState:
    arm: JointState
    gripper: GripperState | DualArmGripperState | None = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.arm.names

    @property
    def positions(self) -> tuple[float, ...]:
        return self.arm.positions


__all__ = [
    "ArxDCanState",
    "DualArmGripperState",
    "GripperState",
    "JointState",
    "MotorState",
]
