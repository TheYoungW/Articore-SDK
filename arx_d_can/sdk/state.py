"""高层 SDK 公开的状态与遥测数据对象。"""
from __future__ import annotations

from dataclasses import dataclass

from ..errors import CommunicationError


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
class JointState:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    torques: tuple[float, ...]


@dataclass(slots=True, frozen=True)
class ArxDCanState:
    arm: JointState
    gripper: GripperState | None = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.arm.names

    @property
    def positions(self) -> tuple[float, ...]:
        return self.arm.positions


@dataclass(slots=True, frozen=True)
class CommunicationHealth:
    """高层 SDK 通信状态的快照。"""

    consecutive_feedback_failures: int
    has_fresh_feedback: bool
    using_fallback_state: bool
    last_error: CommunicationError | None
    last_fresh_feedback_age_s: float | None

    @property
    def healthy(self) -> bool:
        return (
            self.consecutive_feedback_failures == 0
            and self.has_fresh_feedback
            and not self.using_fallback_state
            and self.last_error is None
        )


@dataclass(slots=True, frozen=True)
class MitCommand:
    """驱动保留的一条完整逻辑关节 MIT 命令。"""

    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    feedforward_torques: tuple[float, ...]
    timestamp: float


@dataclass(slots=True, frozen=True)
class CoupledTorqueSaturation:
    """耦合关节最近一次物理电机力矩限幅结果。"""

    active: bool
    motor_names: tuple[str, ...]
    requested_torques: tuple[float, ...]
    limited_torques: tuple[float, ...]
    applied_torques: tuple[float, ...]
    saturation_scale: float
    timestamp: float


@dataclass(slots=True, frozen=True)
class CoupledTorqueTelemetry:
    """最新的耦合电机各阶段命令与实测电机状态。"""

    motor_names: tuple[str, ...]
    motor_positions: tuple[float, ...]
    motor_velocities: tuple[float, ...]
    transformed_torques: tuple[float, ...]
    motor_kd_gains: tuple[float, ...]
    damping_torques: tuple[float, ...]
    requested_torques: tuple[float, ...]
    limited_torques: tuple[float, ...]
    applied_torques: tuple[float, ...]
    estimated_total_torques: tuple[float, ...]
    saturation_scale: float
    timestamp: float


@dataclass(slots=True, frozen=True)
class CoupledControlStats:
    """耦合 MIT 内环的实测时序与反馈健康统计。"""

    target_hz: float
    achieved_hz: float
    cycle_count: int
    overrun_count: int
    feedback_stall_cycles: int
    stale_feedback_faults: int
    maximum_feedback_age_s: float
    torque_command_count: int
    torque_saturation_count: int

    @property
    def torque_saturation_rate(self) -> float:
        if self.torque_command_count <= 0:
            return 0.0
        return self.torque_saturation_count / self.torque_command_count


__all__ = [
    "ArxDCanState",
    "CommunicationHealth",
    "CoupledControlStats",
    "CoupledTorqueSaturation",
    "CoupledTorqueTelemetry",
    "GripperState",
    "JointState",
    "MitCommand",
    "MotorState",
]
