"""由 C++ 产品 Runtime 完整拥有的 Yunyi 双臂业务接口。"""
from __future__ import annotations

from dataclasses import dataclass
from numbers import Real
from typing import Sequence

from arx_d_can._dds import (
    DdsRuntimeClient,
    GravityCompensationStatus,
    JointLimit,
    BimanualFollowStatus,
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
    """同一 DDS 产品快照中的左右臂、夹爪和时序信息。"""

    left: ArxDCanState
    right: ArxDCanState
    has_grippers: bool = True
    motion_arrived: bool = True
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
    """校验每侧7轴并固定为 left J1..J7 + right J1..J7。"""
    left_values = tuple(float(value) for value in left)
    right_values = tuple(float(value) for value in right)
    if len(left_values) != 7 or len(right_values) != 7:
        raise ValueError("left and right joint arrays must each contain 7 values")
    return left_values + right_values


def _gain_frame(value: float | Sequence[float], name: str) -> tuple[float, ...]:
    if isinstance(value, Real):
        return (float(value),) * 14
    values = tuple(float(item) for item in value)
    if len(values) != 14:
        raise ValueError(f"{name} must be a scalar or contain exactly 14 values")
    return values


class ArxDCanDualArm:
    """通过 Cyclone DDS/IP 调用 RK3588 Runtime 的双臂客户端。"""

    def __init__(
        self,
        *,
        robot_id: str = "yunyi-001",
        domain_id: int = 0,
        client_id: str | None = None,
        security_identity: str = "",
        robot_ip: str | None = None,
        network_interfaces: Sequence[str] | None = None,
        control_mode: str = "mit",
        with_grippers: bool = True,
        request_timeout: float = 1.0,
        discovery_timeout: float = 5.0,
        _transport: DdsRuntimeClient | None = None,
    ) -> None:
        self._runtime = _transport or DdsRuntimeClient(
            robot_id=robot_id,
            domain_id=domain_id,
            client_id=client_id,
            security_identity=security_identity,
            robot_ip=robot_ip,
            network_interfaces=network_interfaces,
            control_mode=_mode(control_mode),
            with_grippers=with_grippers,
            request_timeout=request_timeout,
            discovery_timeout=discovery_timeout,
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
        return self._runtime.connected

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
        """非阻塞返回最近 1 秒窗口内的 DDS 状态接收频率。"""
        return self._runtime.get_fps()

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
        left_positions: Sequence[float],
        right_positions: Sequence[float],
        left_velocities: Sequence[float],
        right_velocities: Sequence[float],
        kp: float | Sequence[float],
        kd: float | Sequence[float],
        left_feedforward_torques: Sequence[float],
        right_feedforward_torques: Sequence[float],
    ) -> None:
        """提交用户完整声明的标准 MIT 帧；新帧原子覆盖旧帧。"""
        if self._runtime.control_mode is not RuntimeControlMode.MIT:
            raise RuntimeError("set_joint_mit() requires MIT mode")
        self._runtime.set_joint_mit(
            _frame(left_positions, right_positions),
            _frame(left_velocities, right_velocities),
            _gain_frame(kp, "kp"),
            _gain_frame(kd, "kd"),
            _frame(left_feedforward_torques, right_feedforward_torques),
        )

    def set_joint_mit_fast(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        velocity: float = 100.0,
    ) -> None:
        """提交快速 MIT 的最新完整关节角目标。

        ``velocity`` 为 0..100 的参考步进速度百分比；Runtime 固定
        dq/tau_ff/Kp/Kd，100 对应 5 rad/s，0 保留目标但保持当前参考。
        """
        if self._runtime.control_mode is not RuntimeControlMode.MIT:
            raise RuntimeError("set_joint_mit_fast() requires MIT mode")
        self._runtime.set_joint_mit_fast(_frame(left, right), velocity)

    def set_joint_pv(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        velocity: float = 50.0,
    ) -> None:
        """提交普通 PV 最终目标；速度包络与轨迹执行由 Runtime 负责。"""
        if self._runtime.control_mode is not RuntimeControlMode.PV:
            raise RuntimeError("set_joint_pv() requires PV mode")
        self._runtime.set_joint_pv(_frame(left, right), velocity)

    def set_speed_percent(self, percent: float) -> None:
        """设置 Runtime 共享速度百分比，范围 1..100。

        普通 PV 立即使用新值；后续提交的 Linear/Circular 在提交时
        捕获当前值，已排队或运行的轨迹不会重新计时。
        """
        self._runtime.set_speed_percent(percent)

    def get_speed_percent(self) -> float:
        """读取 Runtime 当前共享速度百分比。"""
        return self._runtime.get_speed_percent()

    def set_max_speed(self, rad_s: float) -> None:
        """设置普通 PV 在 100% 时的全局速度基础上限，0 表示恢复默认值。"""
        self._runtime.set_max_speed(rad_s)

    def get_max_speed(self) -> float:
        """读取普通 PV 全局速度基础上限；0 表示当前使用逐关节默认值。"""
        return self._runtime.get_max_speed()

    def set_max_acceleration(self, rad_s2: float) -> None:
        """设置普通 PV 在 100% 时的全局加速度基础上限，0 表示恢复默认值。"""
        self._runtime.set_max_acceleration(rad_s2)

    def get_max_acceleration(self) -> float:
        """读取普通 PV 全局加速度基础上限；0 表示当前使用逐关节默认值。"""
        return self._runtime.get_max_acceleration()

    def read_state(self) -> ArxDCanDualArmState:
        value = self._runtime.state

        def arm(names: tuple[str, ...], source, gripper_source):
            gripper = None
            if gripper_source is not None and gripper_source.available:
                gripper = DualArmGripperState(
                    opening=gripper_source.opening,
                    gripper_level=gripper_source.gripper_level,
                    enabled=gripper_source.enabled,
                    mos_temperature=gripper_source.mos_temperature,
                    rotor_temperature=gripper_source.rotor_temperature,
                )
            return ArxDCanState(
                arm=JointState(
                    names=names,
                    positions=source.positions,
                    velocities=source.velocities,
                    torques=source.torques,
                    enabled=source.enabled,
                    mos_temperatures=source.mos_temperatures,
                    rotor_temperatures=source.rotor_temperatures,
                ),
                gripper=gripper,
            )

        left = arm(_LEFT_NAMES, value.left, value.left_gripper)
        right = arm(_RIGHT_NAMES, value.right, value.right_gripper)
        return ArxDCanDualArmState(
            left=left,
            right=right,
            has_grippers=value.has_grippers,
            motion_arrived=value.motion_arrived,
            timestamp_ns=value.timestamp_ns,
            sequence=value.sequence,
        )

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

    def solve_ik(
        self,
        *,
        left_target_pose: Sequence[float],
        right_target_pose: Sequence[float],
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """只求解双臂目标位姿，返回左右各7个逻辑关节角，不执行运动。"""
        positions = self._runtime.solve_ik(
            left_target_pose,
            right_target_pose,
        )
        if len(positions) != 14:
            raise RuntimeError(
                f"Runtime returned {len(positions)} IK positions; expected 14"
            )
        return positions[:7], positions[7:]

    def move_pose(
        self, *, side: str, target_pose: Sequence[float],
    ) -> None:
        """非阻塞提交从 Runtime 当前规划位姿到目标位姿的平滑运动。"""
        self._runtime.move_pose(_side(side), target_pose)

    def move_linear(
        self,
        *,
        side: str,
        start_pose: Sequence[float] | None = None,
        end_pose: Sequence[float] | None = None,
        poses: Sequence[Sequence[float]] | None = None,
    ) -> None:
        """提交直线运动；省略 ``start_pose`` 时从当前规划位姿开始。"""
        if poses is not None:
            if start_pose is not None or end_pose is not None:
                raise ValueError("poses cannot be combined with start_pose/end_pose")
            raise ValueError(
                "DDS v1 does not carry explicit waypoint arrays; submit a Runtime "
                "linear target with end_pose"
            )
        if end_pose is None:
            raise ValueError("end_pose is required when poses is omitted")
        self._runtime.move_linear(_side(side), start_pose, end_pose)

    def move_circular(
        self,
        *,
        side: str,
        start_pose: Sequence[float],
        via_pose: Sequence[float],
        end_pose: Sequence[float],
    ) -> None:
        """非阻塞提交自动定时的有限圆弧运动。"""
        self._runtime.move_circular(
            _side(side), start_pose, via_pose, end_pose
        )

    def stop_motion(self) -> None:
        """停止当前有限笛卡尔运动并由 Runtime 安全保持。"""
        self._runtime.stop_motion()

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
