"""由 C++ 产品 Runtime 完整拥有的 Yunyi 双臂业务接口。"""
from __future__ import annotations

from dataclasses import dataclass
from numbers import Real
from typing import Sequence

from arx_d_can._motor_abi import (
    ArticoreRuntime,
    CartesianMotionStatus,
    GravityCompensationStatus,
    RuntimeControlMode,
    SafetyHealth,
    SafetyState,
    ProductPose,
)

from .state import ArxDCanState, DualArmGripperState, JointState


_LEFT_NAMES = tuple(f"l-joint{index}" for index in range(1, 8))
_RIGHT_NAMES = tuple(f"r-joint{index}" for index in range(1, 8))
_PUBLIC_NAMES = tuple(f"joint{index}" for index in range(1, 8))


@dataclass(slots=True, frozen=True)
class ArxDCanDualArmState:
    """同一原生产品快照中的左右臂、夹爪和时序信息。"""

    left: ArxDCanState
    right: ArxDCanState
    has_grippers: bool = True
    timestamp_ns: int = 0
    sequence: int = 0


def _mode(value: str) -> RuntimeControlMode:
    normalized = str(value).strip().lower()
    if normalized == "mit":
        return RuntimeControlMode.MIT
    if normalized in {"pv", "posvel"}:
        return RuntimeControlMode.PV
    raise ValueError("control_mode must be 'mit' or 'pv'")


def _side(value: str) -> int:
    normalized = str(value).strip().lower()
    if normalized == "left":
        return 0
    if normalized == "right":
        return 1
    raise ValueError("side must be 'left' or 'right'")


def _gripper_mode(value: str) -> int:
    normalized = str(value).strip().lower()
    if normalized == "protected":
        return 0
    if normalized == "direct":
        return 1
    raise ValueError("gripper mode must be 'protected' or 'direct'")


def _frame(left: Sequence[float], right: Sequence[float]) -> tuple[float, ...]:
    """只固定左右产品顺序；数量和数值合法性由 Runtime 校验。"""
    return tuple(float(value) for value in (*left, *right))


def _optional_frame(
    left: Sequence[float] | None,
    right: Sequence[float] | None,
) -> tuple[float, ...] | None:
    if left is None and right is None:
        return None
    return _frame(left or (0.0,) * 7, right or (0.0,) * 7)


def _gain_frame(value: float | Sequence[float] | None) -> tuple[float, ...] | None:
    if value is None:
        return None
    if isinstance(value, Real):
        return (float(value),) * 14
    values = tuple(float(item) for item in value)
    return values * 2 if len(values) == 7 else values


class ArxDCanDualArm:
    """只转发整机业务数据的 Yunyi V1.0 双臂客户端。"""

    def __init__(
        self, *, control_mode: str = "mit", with_grippers: bool = True
    ) -> None:
        self._runtime = ArticoreRuntime.create_yunyi(
            _mode(control_mode), with_grippers=with_grippers
        )

    @property
    def joint_names(self) -> tuple[str, ...]:
        return _PUBLIC_NAMES

    @property
    def control_mode(self) -> str:
        return "mit" if self._runtime.control_mode is RuntimeControlMode.MIT else "pv"

    @property
    def has_grippers(self) -> bool:
        return self._runtime.has_grippers

    @property
    def connected(self) -> bool:
        return (
            not self._runtime.closed
            and self._runtime.health.state is not SafetyState.DISCONNECTED
        )

    @property
    def enabled(self) -> bool:
        if self._runtime.closed:
            return False
        return self._runtime.health.state in {
            SafetyState.ENABLED,
            SafetyState.RUNNING,
            SafetyState.SAFE_HOLD,
            SafetyState.DEGRADED,
            SafetyState.SAFE_STOP,
            SafetyState.PARTIALLY_ENABLED,
        }

    def get_health(self) -> SafetyHealth:
        """读取 Runtime 统一健康状态和最近一次具体错误。"""
        return self._runtime.health

    def get_fps(self) -> float:
        """非阻塞返回最近 0.1 秒窗口内的双通道 CAN 总帧率。"""
        return self._runtime.get_fps()

    def set_max_speed(self, max_speed_percent: float) -> None:
        """设置普通 PV reference 速度百分比；0～100 线性对应 0～2 rad/s。"""
        if self._runtime.control_mode is not RuntimeControlMode.PV:
            raise RuntimeError("set_max_speed() requires PV mode")
        self._runtime.set_max_speed(max_speed_percent)

    def get_max_speed(self) -> float:
        """返回普通 PV 位置运动的当前最大速度百分比。"""
        if self._runtime.control_mode is not RuntimeControlMode.PV:
            raise RuntimeError("get_max_speed() requires PV mode")
        return self._runtime.get_max_speed()

    @property
    def gravity_compensation_status(self) -> GravityCompensationStatus:
        return self._runtime.gravity_compensation_status

    def connect(self) -> None:
        self._runtime.connect()

    def disconnect(self) -> None:
        self._runtime.disconnect()

    def enable(self, motors: Sequence[str] | None = None) -> bool:
        """原子使能整机或指定产品电机。"""
        return self._runtime.enable(motors=motors)

    def disable(self, motors: Sequence[str] | None = None) -> bool:
        """原子失能整机或指定产品电机。"""
        return self._runtime.disable(motors=motors)

    def configure_mode(self, mode: str) -> None:
        self._runtime.configure_mode(_mode(mode))

    def set_joint_mit(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        velocity: float = 100.0,
    ) -> None:
        if self._runtime.control_mode is not RuntimeControlMode.MIT:
            raise RuntimeError("set_joint_mit() requires MIT mode")
        self._runtime.set_joint_positions(_frame(left, right), velocity)

    def set_joint_pv(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
    ) -> None:
        if self._runtime.control_mode is not RuntimeControlMode.PV:
            raise RuntimeError("set_joint_pv() requires PV mode")
        self._runtime.set_joint_positions(_frame(left, right))

    def submit_raw_mit(
        self,
        *,
        left_positions: Sequence[float],
        right_positions: Sequence[float],
        left_velocities: Sequence[float] | None = None,
        right_velocities: Sequence[float] | None = None,
        kp: float | Sequence[float] | None = None,
        kd: float | Sequence[float] | None = None,
        left_feedforward_torques: Sequence[float] | None = None,
        right_feedforward_torques: Sequence[float] | None = None,
    ) -> None:
        self._runtime.submit_mit_frame(
            _frame(left_positions, right_positions),
            _optional_frame(left_velocities, right_velocities),
            _optional_frame(
                left_feedforward_torques, right_feedforward_torques
            ),
            _gain_frame(kp),
            _gain_frame(kd),
        )

    def read_state(self) -> ArxDCanDualArmState:
        value = self._runtime.state

        def arm(names: tuple[str, ...], source, gripper_source):
            gripper = None
            if gripper_source is not None and gripper_source.available:
                gripper = DualArmGripperState(
                    opening=gripper_source.opening,
                    gripper_level=gripper_source.gripper_level,
                    enabled=gripper_source.enabled,
                )
            return ArxDCanState(
                arm=JointState(
                    names=names,
                    positions=source.positions,
                    velocities=source.velocities,
                    torques=source.torques,
                    enabled=source.enabled,
                ),
                gripper=gripper,
            )

        left = arm(_LEFT_NAMES, value.left, value.left_gripper)
        right = arm(_RIGHT_NAMES, value.right, value.right_gripper)
        return ArxDCanDualArmState(
            left=left,
            right=right,
            has_grippers=value.has_grippers,
            timestamp_ns=value.timestamp_ns,
            sequence=value.sequence,
        )

    def read_cached_state(self) -> ArxDCanDualArmState:
        return self.read_state()

    def get_pose(self, side: str) -> list[float]:
        """返回指定手臂当前产品控制点位姿 [x, y, z, roll, pitch, yaw]。"""
        return self._runtime.get_pose(_side(side))

    def get_pose_sample(self, side: str) -> ProductPose:
        """返回位姿及其底层反馈时间戳和序列号。"""
        return self._runtime.get_pose_sample(_side(side))

    def move_pose(
        self, *, side: str, target_pose: Sequence[float], speed_percent: float = 50.0
    ) -> None:
        """提交普通 PV PTP 目标；无 motion ID、状态查询或取消接口。"""
        self._runtime.move_pose(_side(side), target_pose, speed_percent)

    def move_poses(
        self,
        *,
        left_target_pose: Sequence[float],
        right_target_pose: Sequence[float],
        speed_percent: float = 50.0,
    ) -> None:
        """原子提交双臂 PTP；两侧 IK 全部成功后同步启动。"""
        self._runtime.move_poses(
            left_target_pose, right_target_pose, speed_percent
        )

    def move_linear(
        self,
        *,
        side: str,
        start_pose: Sequence[float],
        end_pose: Sequence[float],
        speed_percent: float,
    ) -> int:
        """提交 Runtime 复合任务：PTP 到 start_pose，再执行直线路径。"""
        return self._runtime.move_linear(
            _side(side), start_pose, end_pose, speed_percent
        )

    def move_circular(
        self,
        *,
        side: str,
        start_pose: Sequence[float],
        via_pose: Sequence[float],
        end_pose: Sequence[float],
        speed_percent: float,
    ) -> int:
        """提交 Runtime 复合任务：PTP 到 start_pose，再执行圆弧路径。"""
        return self._runtime.move_circular(
            _side(side), start_pose, via_pose, end_pose, speed_percent
        )

    @property
    def cartesian_motion_status(self) -> CartesianMotionStatus:
        """返回最近提交的直线或圆弧运动状态。"""
        return self._runtime.cartesian_motion_status

    def get_cartesian_motion_status(
        self, motion_id: int
    ) -> CartesianMotionStatus:
        """按 Linear/Circular motion_id 查询队列、运行或完成状态。"""
        return self._runtime.get_cartesian_motion_status(motion_id)

    def cancel_cartesian_motion(self) -> None:
        """取消当前 Linear/Circular 和排队路径，并保持最后 PV 参考。"""
        self._runtime.cancel_cartesian_motion()

    def set_grippers(
        self,
        *,
        left: float,
        right: float,
        gripper_level: int,
        mode: str = "protected",
    ) -> None:
        self._runtime.set_product_grippers(
            left=left,
            right=right,
            gripper_level=gripper_level,
            mode=_gripper_mode(mode),
        )

    def start_gravity_compensation(self, *, transition_ms: int = 0) -> None:
        self._runtime.start_gravity_compensation(transition_ms=transition_ms)

    def stop_gravity_compensation(self) -> None:
        self._runtime.stop_gravity_compensation()

    def estop(self) -> None:
        """急停：立即停止所有控制、失能整机并锁存急停状态。

        该方法仅用于紧急情况，不是普通失能或恢复步骤。急停锁存后，
        Runtime 会拒绝继续运动；排除危险并确认安全后才能调用 recover()。
        """
        self._runtime.estop()

    def recover(self) -> None:
        """Clear faults, validate both arms, return to zero, then disable."""
        self._runtime.recover()

    def set_zero(self) -> bool:
        """Zero every installed product motor and return verified success."""
        return self._runtime.set_zero()

    def clear_motor_faults(self) -> None:
        """Clear recoverable faults without moving or changing calibration."""
        self._runtime.clear_faults()

    def __enter__(self) -> ArxDCanDualArm:
        return self

    def __exit__(self, *_args: object) -> None:
        self.disconnect()


__all__ = ["ArxDCanDualArm", "ArxDCanDualArmState"]
