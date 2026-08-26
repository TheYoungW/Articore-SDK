"""由 C++ 产品 Runtime 完整拥有的 Yunyi 双臂业务接口。"""
from __future__ import annotations

from dataclasses import dataclass
from numbers import Real
from typing import Sequence

from arx_d_can._motor_abi import (
    ArticoreRuntime,
    CartesianMotionStatus,
    GravityCompensationStatus,
    JointLimit,
    BimanualFollowStatus,
    RuntimeControlMode,
    SafetyHealth,
    SafetyState,
    TrajectoryStatus,
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


def _trajectory_frame(
    value: float | Sequence[float] | None,
) -> tuple[float, ...] | None:
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

    def get_joint_limits(self) -> dict[str, JointLimit]:
        """返回 Runtime 实际使用的14关节产品逻辑限位。"""
        limits = self._runtime.get_joint_limits()
        names = _LEFT_NAMES + _RIGHT_NAMES
        if len(limits) != len(names):
            raise RuntimeError(
                f"Runtime returned {len(limits)} joint limits; expected 14"
            )
        return dict(zip(names, limits, strict=True))

    @property
    def gravity_compensation_status(self) -> GravityCompensationStatus:
        return self._runtime.gravity_compensation_status

    @property
    def bimanual_follow_status(self) -> BimanualFollowStatus:
        return self._runtime.bimanual_follow_status

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
        self._runtime.set_joint_mit(_frame(left, right), velocity)

    def set_joint_pv(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        velocity: float = 50.0,
    ) -> None:
        if self._runtime.control_mode is not RuntimeControlMode.PV:
            raise RuntimeError("set_joint_pv() requires PV mode")
        self._runtime.set_joint_pv(_frame(left, right), velocity)

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

    def start_trajectory(
        self,
        *,
        timestamps: Sequence[float],
        left_positions: Sequence[Sequence[float]],
        right_positions: Sequence[Sequence[float]],
        interpolation: str = "quintic",
        left_velocities: Sequence[Sequence[float]] | None = None,
        right_velocities: Sequence[Sequence[float]] | None = None,
        left_accelerations: Sequence[Sequence[float]] | None = None,
        right_accelerations: Sequence[Sequence[float]] | None = None,
        kp: float | Sequence[float] | None = None,
        kd: float | Sequence[float] | None = None,
        feedforward_torque: float | Sequence[float] | None = None,
        pv_velocity_limits: float | Sequence[float] = 2.5,
    ) -> int:
        """一次性提交完整双臂关节轨迹，由 C++ Runtime 在 500 Hz 执行。"""
        if str(interpolation).strip().lower() != "quintic":
            raise ValueError("native joint trajectories only support 'quintic'")
        mode = self._runtime.control_mode
        if mode is RuntimeControlMode.MIT and (kp is None or kd is None):
            raise ValueError("MIT trajectories require explicit kp and kd")
        return self._runtime.start_trajectory(
            timestamps=timestamps,
            left_positions=left_positions,
            right_positions=right_positions,
            left_velocities=left_velocities,
            right_velocities=right_velocities,
            left_accelerations=left_accelerations,
            right_accelerations=right_accelerations,
            mit_kp=_trajectory_frame(kp),
            mit_kd=_trajectory_frame(kd),
            mit_feedforward_torques=_trajectory_frame(feedforward_torque),
            pv_velocity_limits=(
                _trajectory_frame(pv_velocity_limits)
                if mode is RuntimeControlMode.PV
                else None
            ),
        )

    @property
    def trajectory_status(self) -> TrajectoryStatus:
        """返回原生 14 关节轨迹的最新状态快照。"""
        return self._runtime.trajectory_status

    def cancel_trajectory(self) -> None:
        """幂等取消原生关节轨迹；不用于笛卡尔 Linear/Circular。"""
        self._runtime.cancel_trajectory()

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
        """返回指定手臂当前活动 TCP 位姿 [x, y, z, roll, pitch, yaw]。"""
        return self._runtime.get_pose(_side(side))

    def get_pose_sample(self, side: str) -> ProductPose:
        """返回位姿及其底层反馈时间戳和序列号。"""
        return self._runtime.get_pose_sample(_side(side))

    def set_tcp_offset(self, *, side: str, offset: Sequence[float]) -> None:
        """设置法兰 link7 到活动 TCP 的 [x,y,z,roll,pitch,yaw] 偏移。"""
        self._runtime.set_tcp_offset(_side(side), offset)

    def get_tcp_offset(self, *, side: str) -> list[float]:
        """读取法兰 link7 到当前活动 TCP 的偏移。"""
        return self._runtime.get_tcp_offset(_side(side))

    def reset_tcp_offset(self, *, side: str) -> None:
        """恢复产品默认 TCP：有夹爪为 tool0，无夹爪为 link7。"""
        self._runtime.reset_tcp_offset(_side(side))

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

    def start_bimanual_follow(self, *, leader: str = "left") -> None:
        """记录当前相对关节位置；普通 PV/MIT 控制主臂时从臂同步跟随。"""
        normalized = leader.strip().lower()
        if normalized not in {"left", "right"}:
            raise ValueError("leader must be 'left' or 'right'")
        self._runtime.start_bimanual_follow(0 if normalized == "left" else 1)

    def stop_bimanual_follow(self) -> None:
        """退出双臂跟随并让双臂保持退出瞬间的位置。"""
        self._runtime.stop_bimanual_follow()

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
