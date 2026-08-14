"""Articore 单臂/双臂原生安全运行时的 ctypes 边界。"""
from __future__ import annotations

import ctypes
from dataclasses import dataclass
from enum import Enum, IntEnum
import math
from typing import Sequence

from motor_drive_layer import articore_runtime_library_path
from motor_drive_layer.abi import get_abi


_UINT64_MAX = (1 << 64) - 1
_ARTICORE_ABI_MAJOR = 2
_ARTICORE_ABI_MINOR = 0
ARTICORE_CAP_COMMAND_LIFETIME = 1 << 11
ARTICORE_CAP_PROTECTIVE_FAULT_HOLD = 1 << 13
ARTICORE_CAP_DETERMINISTIC_DISABLE = 1 << 14
ARTICORE_CAP_LAYERED_JOINT_LIMITS = 1 << 18
ARTICORE_CAP_GRIPPER_COMMAND_PROFILES = 1 << 19
ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS = 1 << 20
ARTICORE_CAP_JOINT_MIT_POSITION = 1 << 21
ARTICORE_CAP_JOINT_PV_POSITION = 1 << 22
_REQUIRED_CAPABILITIES = (
    (1 << 0)  # 命令看门狗
    | (1 << 1)  # 安全保持
    | (1 << 2)  # 夹爪保护
    | (1 << 3)  # 单通道
    | (1 << 4)  # 双通道
    | (1 << 5)  # 结构化通信健康状态
    | (1 << 8)  # 实时关节命令邮箱
    | (1 << 10)  # 原子使能事务
    | ARTICORE_CAP_COMMAND_LIFETIME
    | ARTICORE_CAP_PROTECTIVE_FAULT_HOLD
    | ARTICORE_CAP_DETERMINISTIC_DISABLE
    | ARTICORE_CAP_LAYERED_JOINT_LIMITS
    | ARTICORE_CAP_GRIPPER_COMMAND_PROFILES
    | ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS
    | ARTICORE_CAP_JOINT_MIT_POSITION
    | ARTICORE_CAP_JOINT_PV_POSITION
)


class SafetyState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    READY = "READY"
    ENABLED = "ENABLED"
    RUNNING = "RUNNING"
    SAFE_HOLD = "SAFE_HOLD"
    FAULT = "FAULT"


class GripperControlState(str, Enum):
    DISABLED = "DISABLED"
    IDLE = "IDLE"
    MOVING = "MOVING"
    CONTACT = "CONTACT"
    HOLDING = "HOLDING"
    OVERLOAD_RETREAT = "OVERLOAD_RETREAT"
    FAULT = "FAULT"


class GripperForceLevel(IntEnum):
    """Runtime ABI 2.0 的十档夹持力；1 最轻，5 默认，10 最强。"""

    LEVEL_1 = 1
    LEVEL_2 = 2
    LEVEL_3 = 3
    LEVEL_4 = 4
    LEVEL_5 = 5
    LEVEL_6 = 6
    LEVEL_7 = 7
    LEVEL_8 = 8
    LEVEL_9 = 9
    LEVEL_10 = 10


_STATE_BY_CODE = {
    0: SafetyState.DISCONNECTED,
    1: SafetyState.READY,
    2: SafetyState.ENABLED,
    3: SafetyState.RUNNING,
    4: SafetyState.SAFE_HOLD,
    5: SafetyState.FAULT,
}

_GRIPPER_STATE_BY_CODE = {
    0: GripperControlState.DISABLED,
    1: GripperControlState.IDLE,
    2: GripperControlState.MOVING,
    3: GripperControlState.CONTACT,
    4: GripperControlState.HOLDING,
    5: GripperControlState.OVERLOAD_RETREAT,
    6: GripperControlState.FAULT,
}

@dataclass(slots=True, frozen=True)
class TransportHealth:
    connected: bool
    healthy: bool
    consecutive_send_failures: int
    consecutive_feedback_failures: int
    last_feedback_age_s: float | None
    last_error: str | None
    tx_frames: int = 0
    rx_frames: int = 0
    send_errors: int = 0
    receive_errors: int = 0
    last_tx_age_s: float | None = None
    last_rx_age_s: float | None = None


@dataclass(slots=True, frozen=True)
class SafetyHealth:
    state: SafetyState
    fault_reason: str | None
    last_successful_command_age_s: float | None
    last_fresh_feedback_age_s: float | None
    consecutive_send_failures: int
    consecutive_feedback_failures: int
    left_transport: TransportHealth
    right_transport: TransportHealth
    motor_faults: tuple[str, ...]
    unconfirmed_disable_motors: tuple[str, ...]
    safe_holding: bool
    disable_confirmed: bool
    left_gripper: GripperSafetyHealth | None = None
    right_gripper: GripperSafetyHealth | None = None


@dataclass(slots=True, frozen=True)
class GripperSafetyHealth:
    name: str
    side: int
    opening: float
    motor_position: float
    torque: float
    control_state: GripperControlState
    contact_detected: bool
    stalled: bool
    overload: bool
    hold_target: float | None
    feedback_age_s: float | None
    fault_reason: str | None


@dataclass(slots=True, frozen=True)
class NativeMotorDescriptor:
    motor: object
    side: int
    name: str
    is_gripper: bool = False
    safe_kp: float = 5.0
    safe_kd: float = 1.0
    overload_torque: float = 0.0
    retreat_distance: float = 0.0
    contact_torque: float = 0.0
    motion_window_s: float = 0.0
    stall_movement: float = 0.0
    min_position_error: float = 0.0
    contact_hold_s: float = 0.0
    overload_hold_s: float = 0.0
    hold_offset: float = 0.0
    retreat_retry_s: float = 0.0
    open_position: float = 0.0
    closed_position: float = 0.0
    normal_kp: float = 0.0
    normal_kd: float = 0.0
    close_speed: float = 0.0
    max_step_interval_s: float = 0.0
    closing_direction: float = 0.0
    lower_position: float = -math.inf
    upper_position: float = math.inf


@dataclass(slots=True, frozen=True)
class NativeJointControlConfig:
    """一个机械臂关节在 motor 原生坐标中的控制参数。"""

    motor: object
    lower_position: float
    upper_position: float
    velocity_limit: float
    torque_limit: float
    mit_kp: float
    mit_kd: float
    mit_feedforward_torque: float = 0.0


@dataclass(slots=True, frozen=True)
class NativeJointSafetyLimits:
    """一个机械臂关节在 motor 原生坐标中的分层位置安全参数。"""

    motor: object
    hard_lower_position: float
    hard_upper_position: float
    soft_lower_position: float
    soft_upper_position: float
    soft_limit_braking_zone: float
    braking_acceleration: float


@dataclass(slots=True, frozen=True)
class NativeGripperForceProfile:
    """一个实际安装夹爪的一档产品力矩与增益标定。"""

    motor: object
    force_level: GripperForceLevel
    contact_torque: float
    overload_torque: float
    moving_kp: float
    moving_kd: float
    hold_kp: float
    hold_kd: float


@dataclass(slots=True, frozen=True)
class EnableMotorResult:
    """原子使能事务中一台电机的最终确认结果。"""

    side: int
    name: str
    can_id: int
    status_code: int
    has_feedback: bool
    feedback_fresh: bool
    enabled: bool


@dataclass(slots=True, frozen=True)
class MissingEnableMotor:
    """原子使能事务中未取得有效反馈的电机标识。"""

    side: int
    can_id: int


@dataclass(slots=True, frozen=True)
class EnableReport:
    """最近一次原子使能事务的结构化报告。"""

    success: bool
    disable_confirmed: bool
    expected_count: int
    enabled_count: int
    missing_count: int
    failure_count: int
    missing_motors: tuple[MissingEnableMotor, ...]
    motors: tuple[EnableMotorResult, ...]
    error: str | None


class NativeEnableError(RuntimeError):
    """原生原子使能失败，并携带无需解析字符串的结构化报告。"""

    def __init__(self, report: EnableReport) -> None:
        self.report = report
        detail = report.error or "原子使能事务失败"
        super().__init__(detail)


@dataclass(slots=True, frozen=True)
class DisableMotorResult:
    """确定性失能事务中一台电机的最终确认结果。"""

    side: int
    name: str
    can_id: int
    status_code: int
    has_feedback: bool
    feedback_fresh: bool
    disabled: bool
    disable_sent: bool
    retry_sent: bool


@dataclass(slots=True, frozen=True)
class MissingDisableMotor:
    """确定性失能事务中未确认失能的电机标识。"""

    side: int
    can_id: int


@dataclass(slots=True, frozen=True)
class DisableReport:
    """最近一次确定性失能事务的结构化报告。"""

    success: bool
    barrier_confirmed: bool
    expected_count: int
    disabled_count: int
    missing_count: int
    failure_count: int
    retry_count: int
    missing_motors: tuple[MissingDisableMotor, ...]
    motors: tuple[DisableMotorResult, ...]
    error: str | None


class NativeDisableError(RuntimeError):
    """原生确定性失能失败；Runtime 及其依赖句柄仍归调用方持有。"""

    def __init__(self, report: DisableReport, *, operation: str) -> None:
        self.report = report
        self.operation = operation
        detail = report.error or "确定性失能事务失败"
        super().__init__(f"{operation} failed: {detail}")


class _RuntimeConfig(ctypes.Structure):
    _fields_ = [
        ("control_hz", ctypes.c_uint32),
        ("command_timeout_ms", ctypes.c_uint32),
        ("enable_grace_ms", ctypes.c_uint32),
        ("safe_hold_hz", ctypes.c_uint32),
        ("feedback_check_hz", ctypes.c_uint32),
        ("feedback_failure_threshold", ctypes.c_uint32),
        ("feedback_max_age_ms", ctypes.c_uint32),
        ("safe_hold_failure_threshold", ctypes.c_uint32),
        ("disable_feedback_timeout_ms", ctypes.c_uint32),
        ("safe_pv_velocity_limit", ctypes.c_float),
        ("gripper_control_hz", ctypes.c_uint32),
        ("gripper_fault_action", ctypes.c_int32),
    ]


class _MotorApi(ctypes.Structure):
    _fields_ = [
        ("group_send_pos_vel", ctypes.c_void_p),
        ("group_send_mit", ctypes.c_void_p),
        ("controller_disable_all", ctypes.c_void_p),
        ("controller_request_feedback_all_ex", ctypes.c_void_p),
        ("motor_get_state", ctypes.c_void_p),
        ("motor_get_feedback_stats", ctypes.c_void_p),
        ("last_error_message", ctypes.c_void_p),
        ("controller_get_transport_health", ctypes.c_void_p),
        ("motor_disable", ctypes.c_void_p),
    ]


class _MotorDescriptor(ctypes.Structure):
    _fields_ = [
        ("motor", ctypes.c_void_p),
        ("side", ctypes.c_uint8),
        ("is_gripper", ctypes.c_uint8),
        ("name", ctypes.c_char * 64),
        ("safe_kp", ctypes.c_float),
        ("safe_kd", ctypes.c_float),
        ("overload_torque", ctypes.c_float),
        ("retreat_distance", ctypes.c_float),
        ("contact_torque", ctypes.c_float),
        ("motion_window_ms", ctypes.c_uint32),
        ("stall_movement", ctypes.c_float),
        ("min_position_error", ctypes.c_float),
        ("contact_hold_ms", ctypes.c_uint32),
        ("overload_hold_ms", ctypes.c_uint32),
        ("hold_offset", ctypes.c_float),
        ("retreat_retry_ms", ctypes.c_uint32),
        ("open_position", ctypes.c_float),
        ("closed_position", ctypes.c_float),
        ("normal_kp", ctypes.c_float),
        ("normal_kd", ctypes.c_float),
        ("close_speed", ctypes.c_float),
        ("max_step_interval_ms", ctypes.c_uint32),
        ("closing_direction", ctypes.c_float),
        ("lower_position", ctypes.c_float),
        ("upper_position", ctypes.c_float),
    ]


class _PosVelCommand(ctypes.Structure):
    _fields_ = [
        ("motor", ctypes.c_void_p),
        ("target_position", ctypes.c_float),
        ("velocity_limit", ctypes.c_float),
    ]


class _MitCommand(ctypes.Structure):
    _fields_ = [
        ("motor", ctypes.c_void_p),
        ("target_position", ctypes.c_float),
        ("target_velocity", ctypes.c_float),
        ("stiffness", ctypes.c_float),
        ("damping", ctypes.c_float),
        ("feedforward_torque", ctypes.c_float),
    ]


class _JointMitTarget(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("motor", ctypes.c_void_p),
        ("target_position", ctypes.c_float),
    ]


class _JointPvTarget(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("motor", ctypes.c_void_p),
        ("target_position", ctypes.c_float),
    ]


class _GripperCommand(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("motor", ctypes.c_void_p),
        ("opening", ctypes.c_float),
        ("speed", ctypes.c_float),
        ("force_level", ctypes.c_int32),
    ]


class _GripperForceProfile(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("motor", ctypes.c_void_p),
        ("force_level", ctypes.c_int32),
        ("contact_torque", ctypes.c_float),
        ("overload_torque", ctypes.c_float),
        ("moving_kp", ctypes.c_float),
        ("moving_kd", ctypes.c_float),
        ("hold_kp", ctypes.c_float),
        ("hold_kd", ctypes.c_float),
    ]


class _JointControlConfig(ctypes.Structure):
    _fields_ = [
        ("motor", ctypes.c_void_p),
        ("lower_position", ctypes.c_float),
        ("upper_position", ctypes.c_float),
        ("velocity_limit", ctypes.c_float),
        ("torque_limit", ctypes.c_float),
        ("mit_kp", ctypes.c_float),
        ("mit_kd", ctypes.c_float),
        ("mit_feedforward_torque", ctypes.c_float),
    ]


class _JointSafetyLimits(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("motor", ctypes.c_void_p),
        ("hard_lower_position", ctypes.c_float),
        ("hard_upper_position", ctypes.c_float),
        ("soft_lower_position", ctypes.c_float),
        ("soft_upper_position", ctypes.c_float),
        ("soft_limit_braking_zone", ctypes.c_float),
        ("braking_acceleration", ctypes.c_float),
    ]


class _EnableMotorResult(ctypes.Structure):
    _fields_ = [
        ("side", ctypes.c_uint8),
        ("can_id", ctypes.c_uint8),
        ("status_code", ctypes.c_uint8),
        ("has_feedback", ctypes.c_uint8),
        ("feedback_fresh", ctypes.c_uint8),
        ("enabled", ctypes.c_uint8),
        ("name", ctypes.c_char * 64),
    ]


class _EnableReport(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("success", ctypes.c_int32),
        ("disable_confirmed", ctypes.c_int32),
        ("expected_count", ctypes.c_uint32),
        ("enabled_count", ctypes.c_uint32),
        ("missing_count", ctypes.c_uint32),
        ("failure_count", ctypes.c_uint32),
        ("missing_motor_sides", ctypes.c_uint8 * 32),
        ("missing_motor_ids", ctypes.c_uint32 * 32),
        ("motor_count", ctypes.c_uint32),
        ("motors", _EnableMotorResult * 32),
        ("error", ctypes.c_char * 512),
    ]


class _DisableMotorResult(ctypes.Structure):
    _fields_ = [
        ("side", ctypes.c_uint8),
        ("can_id", ctypes.c_uint8),
        ("status_code", ctypes.c_uint8),
        ("has_feedback", ctypes.c_uint8),
        ("feedback_fresh", ctypes.c_uint8),
        ("disabled", ctypes.c_uint8),
        ("disable_sent", ctypes.c_uint8),
        ("retry_sent", ctypes.c_uint8),
        ("name", ctypes.c_char * 64),
    ]


class _DisableReport(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("success", ctypes.c_int32),
        ("barrier_confirmed", ctypes.c_int32),
        ("expected_count", ctypes.c_uint32),
        ("disabled_count", ctypes.c_uint32),
        ("missing_count", ctypes.c_uint32),
        ("failure_count", ctypes.c_uint32),
        ("retry_count", ctypes.c_uint32),
        ("missing_motor_sides", ctypes.c_uint8 * 32),
        ("missing_motor_ids", ctypes.c_uint32 * 32),
        ("motor_count", ctypes.c_uint32),
        ("motors", _DisableMotorResult * 32),
        ("error", ctypes.c_char * 512),
    ]


class _TransportHealth(ctypes.Structure):
    _fields_ = [
        ("connected", ctypes.c_int32),
        ("healthy", ctypes.c_int32),
        ("consecutive_send_failures", ctypes.c_uint32),
        ("consecutive_feedback_failures", ctypes.c_uint32),
        ("last_feedback_age_ns", ctypes.c_uint64),
        ("tx_frames", ctypes.c_uint64),
        ("rx_frames", ctypes.c_uint64),
        ("send_errors", ctypes.c_uint64),
        ("receive_errors", ctypes.c_uint64),
        ("last_tx_age_ns", ctypes.c_uint64),
        ("last_rx_age_ns", ctypes.c_uint64),
        ("last_error", ctypes.c_char * 256),
    ]


class _GripperHealth(ctypes.Structure):
    _fields_ = [
        ("available", ctypes.c_int32),
        ("side", ctypes.c_uint8),
        ("control_state", ctypes.c_int32),
        ("opening", ctypes.c_float),
        ("motor_position", ctypes.c_float),
        ("torque", ctypes.c_float),
        ("contact_detected", ctypes.c_int32),
        ("stalled", ctypes.c_int32),
        ("overload", ctypes.c_int32),
        ("has_hold_target", ctypes.c_int32),
        ("hold_target", ctypes.c_float),
        ("feedback_age_ns", ctypes.c_uint64),
        ("name", ctypes.c_char * 64),
        ("fault_reason", ctypes.c_char * 256),
    ]


class _SafetyHealth(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_int32),
        ("safe_holding", ctypes.c_int32),
        ("disable_confirmed", ctypes.c_int32),
        ("last_successful_command_age_ns", ctypes.c_uint64),
        ("last_fresh_feedback_age_ns", ctypes.c_uint64),
        ("consecutive_send_failures", ctypes.c_uint32),
        ("consecutive_feedback_failures", ctypes.c_uint32),
        ("left_transport", _TransportHealth),
        ("right_transport", _TransportHealth),
        ("gripper_count", ctypes.c_uint32),
        ("grippers", _GripperHealth * 2),
        ("motor_fault_count", ctypes.c_uint32),
        ("motor_faults", (ctypes.c_char * 64) * 32),
        ("unconfirmed_disable_count", ctypes.c_uint32),
        ("unconfirmed_disable", (ctypes.c_char * 64) * 32),
        ("fault_reason", ctypes.c_char * 512),
    ]


def _pointer(value: object, *, name: str) -> int:
    pointer = getattr(value, "_ptr", None)
    if not pointer:
        raise RuntimeError(f"{name} does not expose an open native handle")
    return int(pointer)


def _function_pointer(function: object) -> int:
    value = ctypes.cast(function, ctypes.c_void_p).value
    if value is None:
        raise RuntimeError("motor-drive-layer exported a null function pointer")
    return value


def _text(value: bytes | ctypes.Array) -> str | None:
    raw = bytes(value).split(b"\0", 1)[0]
    return raw.decode(errors="replace") if raw else None


def _seconds(age_ns: int) -> float | None:
    return None if age_ns == _UINT64_MAX else age_ns / 1_000_000_000.0


class NativeSafetyRuntime:
    """管理单臂或双臂的 C++ 看门狗与安全状态机。"""

    def __init__(
        self,
        *,
        controller_group: object,
        left_controller: object,
        right_controller: object | None = None,
        motors: Sequence[NativeMotorDescriptor],
        joints: Sequence[NativeJointControlConfig],
        joint_safety_limits: Sequence[NativeJointSafetyLimits],
        gripper_force_profiles: Sequence[NativeGripperForceProfile] = (),
        control_hz: float = 500.0,
        command_timeout_s: float = 0.25,
        enable_grace_s: float = 2.0,
        safe_hold_hz: float = 100.0,
        feedback_check_hz: float = 100.0,
        feedback_failure_threshold: int = 3,
        feedback_max_age_s: float = 0.02,
        safe_hold_failure_threshold: int = 1,
        disable_feedback_timeout_ms: int = 50,
        safe_pv_velocity_limit: float = 0.2,
        gripper_control_hz: float = 500.0,
        gripper_fault_action: str = "hold",
    ) -> None:
        self._lib = ctypes.CDLL(articore_runtime_library_path())
        # 先读取旧 ABI 也具备的版本与能力符号。误装旧版运行库时应给出明确的版本
        # 错误，而不是因 ABI 2.0 符号不存在而抛出晦涩的 AttributeError。
        self._lib.articore_runtime_abi_version.restype = ctypes.c_uint32
        self._lib.articore_runtime_capabilities.restype = ctypes.c_uint64
        version = int(self._lib.articore_runtime_abi_version())
        major = version >> 16
        minor = version & 0xFFFF
        if major != _ARTICORE_ABI_MAJOR or minor < _ARTICORE_ABI_MINOR:
            raise RuntimeError(
                "incompatible Articore runtime ABI: "
                f"expected {_ARTICORE_ABI_MAJOR}.{_ARTICORE_ABI_MINOR} or newer, "
                f"got {major}.{minor}"
            )
        capabilities = int(self._lib.articore_runtime_capabilities())
        missing = _REQUIRED_CAPABILITIES & ~capabilities
        if missing:
            raise RuntimeError(
                "Articore runtime is missing required capabilities: "
                f"0x{missing:x}"
            )
        self._bind()
        self._motor_abi = get_abi()
        motor_lib = self._motor_abi.lib
        if not getattr(self._motor_abi, "has_transport_health", False):
            raise RuntimeError(
                "motor-drive-layer 0.9.2 must expose structured transport health"
            )
        if not getattr(self._motor_abi, "has_structured_feedback_report", False):
            raise RuntimeError(
                "motor-drive-layer 0.9.2 must expose structured feedback reports"
            )
        transport_health_pointer = _function_pointer(
            motor_lib.motor_controller_get_transport_health
        )
        self._api = _MotorApi(
            _function_pointer(motor_lib.motor_controller_group_send_pos_vel),
            _function_pointer(motor_lib.motor_controller_group_send_mit),
            _function_pointer(motor_lib.motor_controller_disable_all),
            _function_pointer(motor_lib.motor_controller_request_feedback_all_ex),
            _function_pointer(motor_lib.motor_handle_get_state),
            _function_pointer(motor_lib.motor_handle_get_feedback_stats),
            _function_pointer(motor_lib.motor_last_error_message),
            transport_health_pointer,
            _function_pointer(motor_lib.motor_handle_disable),
        )
        descriptors = tuple(motors)
        if not descriptors:
            raise ValueError("native safety runtime requires motors")
        self._motors = descriptors
        native_descriptors = (_MotorDescriptor * len(descriptors))()
        for index, descriptor in enumerate(descriptors):
            native = native_descriptors[index]
            native.motor = _pointer(descriptor.motor, name=descriptor.name)
            native.side = int(descriptor.side)
            native.is_gripper = int(descriptor.is_gripper)
            encoded_name = descriptor.name.encode()
            if len(encoded_name) >= 64:
                raise ValueError("motor name must contain fewer than 64 UTF-8 bytes")
            native.name = encoded_name
            native.safe_kp = descriptor.safe_kp
            native.safe_kd = descriptor.safe_kd
            native.overload_torque = descriptor.overload_torque
            native.retreat_distance = descriptor.retreat_distance
            native.contact_torque = descriptor.contact_torque
            native.motion_window_ms = max(0, round(descriptor.motion_window_s * 1000.0))
            native.stall_movement = descriptor.stall_movement
            native.min_position_error = descriptor.min_position_error
            native.contact_hold_ms = max(0, round(descriptor.contact_hold_s * 1000.0))
            native.overload_hold_ms = max(0, round(descriptor.overload_hold_s * 1000.0))
            native.hold_offset = descriptor.hold_offset
            native.retreat_retry_ms = max(0, round(descriptor.retreat_retry_s * 1000.0))
            native.open_position = descriptor.open_position
            native.closed_position = descriptor.closed_position
            native.normal_kp = descriptor.normal_kp
            native.normal_kd = descriptor.normal_kd
            native.close_speed = descriptor.close_speed
            native.max_step_interval_ms = max(
                0, round(descriptor.max_step_interval_s * 1000.0)
            )
            native.closing_direction = descriptor.closing_direction
            native.lower_position = descriptor.lower_position
            native.upper_position = descriptor.upper_position
        self._native_descriptors = native_descriptors
        normalized_fault_action = str(gripper_fault_action).strip().lower()
        if normalized_fault_action not in {"hold", "disable"}:
            raise ValueError("gripper_fault_action must be 'hold' or 'disable'")
        config = _RuntimeConfig(
            max(1, round(control_hz)),
            max(1, round(command_timeout_s * 1000.0)),
            max(1, round(enable_grace_s * 1000.0)),
            max(1, round(safe_hold_hz)),
            max(1, round(feedback_check_hz)),
            int(feedback_failure_threshold),
            max(1, round(feedback_max_age_s * 1000.0)),
            int(safe_hold_failure_threshold),
            int(disable_feedback_timeout_ms),
            float(safe_pv_velocity_limit),
            max(1, round(gripper_control_hz)),
            1 if normalized_fault_action == "hold" else 2,
        )
        self._ptr = self._lib.articore_runtime_create_ex(
            ctypes.byref(config),
            ctypes.byref(self._api),
            _pointer(controller_group, name="ControllerGroup"),
            _pointer(left_controller, name="left Controller"),
            (
                0
                if right_controller is None
                else _pointer(right_controller, name="right Controller")
            ),
            native_descriptors,
            len(descriptors),
            _function_pointer(motor_lib.motor_controller_enable_all),
            _function_pointer(motor_lib.motor_handle_enable),
        )
        if not self._ptr:
            raise RuntimeError(f"native safety runtime creation failed: {self._error()}")
        try:
            self._configure_joints(joints)
            self._configure_joint_safety_limits(joint_safety_limits)
            active_grippers = tuple(
                descriptor for descriptor in descriptors if descriptor.is_gripper
            )
            if active_grippers:
                self._configure_gripper_force_profiles(gripper_force_profiles)
            elif gripper_force_profiles:
                raise ValueError("gripper force profiles require an installed gripper")
        except Exception:
            self._lib.articore_runtime_free(self._ptr)
            self._ptr = None
            raise
        self._pv_array = None
        self._arm_mit_array = None
        self._joint_mit_target_array = None
        self._joint_pv_target_array = None
        self._gripper_command_array = None

    def _bind(self) -> None:
        lib = self._lib
        lib.articore_runtime_abi_version.restype = ctypes.c_uint32
        lib.articore_runtime_capabilities.restype = ctypes.c_uint64
        lib.articore_runtime_create_ex.argtypes = [
            ctypes.POINTER(_RuntimeConfig), ctypes.POINTER(_MotorApi), ctypes.c_void_p,
            ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(_MotorDescriptor), ctypes.c_uint32,
            ctypes.c_void_p, ctypes.c_void_p,
        ]
        lib.articore_runtime_create_ex.restype = ctypes.c_void_p
        lib.articore_runtime_free.argtypes = [ctypes.c_void_p]
        lib.articore_runtime_connect.argtypes = [ctypes.c_void_p]
        lib.articore_runtime_enable.argtypes = [ctypes.c_void_p, ctypes.c_int32]
        lib.articore_runtime_get_last_enable_report.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_EnableReport),
        ]
        lib.articore_runtime_get_last_disable_report.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_DisableReport),
        ]
        lib.articore_runtime_configure_joints.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_JointControlConfig),
            ctypes.c_uint32,
        ]
        lib.articore_runtime_configure_joint_safety_limits.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_JointSafetyLimits),
            ctypes.c_uint32,
        ]
        lib.articore_runtime_submit_pos_vel_ex.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_PosVelCommand),
            ctypes.c_uint32,
            ctypes.c_int32,
        ]
        lib.articore_runtime_submit_mit_ex.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_MitCommand),
            ctypes.c_uint32,
            ctypes.c_int32,
        ]
        lib.articore_runtime_set_joint_mit.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_JointMitTarget),
            ctypes.c_uint32,
            ctypes.c_float,
        ]
        lib.articore_runtime_set_joint_pv.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_JointPvTarget),
            ctypes.c_uint32,
            ctypes.c_float,
        ]
        lib.articore_runtime_configure_gripper_force_profiles.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_GripperForceProfile),
            ctypes.c_uint32,
        ]
        lib.articore_runtime_set_gripper_commands.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_GripperCommand),
            ctypes.c_uint32,
        ]
        lib.articore_runtime_report_feedback_failure.argtypes = [
            ctypes.c_void_p, ctypes.c_uint8, ctypes.c_char_p,
        ]
        lib.articore_runtime_disable.argtypes = [ctypes.c_void_p]
        lib.articore_runtime_estop.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        lib.articore_runtime_recover.argtypes = [ctypes.c_void_p]
        lib.articore_runtime_get_health.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(_SafetyHealth),
        ]
        lib.articore_runtime_close.argtypes = [ctypes.c_void_p]
        lib.articore_runtime_last_error.restype = ctypes.c_char_p
        for name in (
            "articore_runtime_connect", "articore_runtime_enable",
            "articore_runtime_get_last_enable_report",
            "articore_runtime_get_last_disable_report",
            "articore_runtime_configure_joints",
            "articore_runtime_configure_joint_safety_limits",
            "articore_runtime_submit_pos_vel_ex",
            "articore_runtime_submit_mit_ex",
            "articore_runtime_set_joint_mit",
            "articore_runtime_set_joint_pv",
            "articore_runtime_configure_gripper_force_profiles",
            "articore_runtime_set_gripper_commands",
            "articore_runtime_report_feedback_failure", "articore_runtime_disable",
            "articore_runtime_estop", "articore_runtime_recover",
            "articore_runtime_get_health", "articore_runtime_close",
        ):
            getattr(lib, name).restype = ctypes.c_int32

    def _error(self) -> str:
        message = self._lib.articore_runtime_last_error()
        return message.decode(errors="replace") if message else "unknown native runtime error"

    def _ok(self, result: int, operation: str) -> None:
        if result != 0:
            raise RuntimeError(f"{operation} failed: {self._error()}")

    def connect(self) -> None:
        self._ok(self._lib.articore_runtime_connect(self._ptr), "runtime connect")

    def _configure_joints(
        self,
        joints: Sequence[NativeJointControlConfig],
    ) -> None:
        values = tuple(joints)
        if not values:
            raise ValueError("native safety runtime requires arm joint configuration")
        native = (_JointControlConfig * len(values))()
        for index, joint in enumerate(values):
            native[index] = _JointControlConfig(
                _pointer(joint.motor, name="joint control motor"),
                float(joint.lower_position),
                float(joint.upper_position),
                float(joint.velocity_limit),
                float(joint.torque_limit),
                float(joint.mit_kp),
                float(joint.mit_kd),
                float(joint.mit_feedforward_torque),
            )
        self._ok(
            self._lib.articore_runtime_configure_joints(
                self._ptr,
                native,
                len(values),
            ),
            "runtime joint configuration",
        )
        self._native_joint_configs = native

    def _configure_joint_safety_limits(
        self,
        limits: Sequence[NativeJointSafetyLimits],
    ) -> None:
        """在 connect 前配置每个机械臂关节的硬/软限位与制动参数。"""
        values = tuple(limits)
        if not values:
            raise ValueError("joint safety limits must not be empty")
        native = (_JointSafetyLimits * len(values))()
        for index, value in enumerate(values):
            native[index] = _JointSafetyLimits(
                ctypes.sizeof(_JointSafetyLimits),
                _pointer(value.motor, name="joint safety motor"),
                float(value.hard_lower_position),
                float(value.hard_upper_position),
                float(value.soft_lower_position),
                float(value.soft_upper_position),
                float(value.soft_limit_braking_zone),
                float(value.braking_acceleration),
            )
        self._ok(
            self._lib.articore_runtime_configure_joint_safety_limits(
                self._ptr,
                native,
                len(values),
            ),
            "runtime layered joint safety configuration",
        )
        self._native_joint_safety_limits = native

    def _configure_gripper_force_profiles(
        self,
        profiles: Sequence[NativeGripperForceProfile],
    ) -> None:
        """在 connect 前为每个实际安装夹爪配置完整十档产品标定。"""
        values = tuple(profiles)
        if not values:
            raise ValueError("gripper force profiles must not be empty")
        native = (_GripperForceProfile * len(values))()
        for index, value in enumerate(values):
            native[index] = _GripperForceProfile(
                ctypes.sizeof(_GripperForceProfile),
                _pointer(value.motor, name="gripper force profile motor"),
                int(GripperForceLevel(value.force_level)),
                float(value.contact_torque),
                float(value.overload_torque),
                float(value.moving_kp),
                float(value.moving_kd),
                float(value.hold_kp),
                float(value.hold_kd),
            )
        self._ok(
            self._lib.articore_runtime_configure_gripper_force_profiles(
                self._ptr,
                native,
                len(values),
            ),
            "runtime gripper force profile configuration",
        )
        self._native_gripper_force_profiles = native

    def enable(self, mode: str) -> None:
        normalized = mode.strip().lower().replace("_", "")
        mode_id = 1 if normalized in {"pv", "posvel"} else 2 if normalized == "mit" else 0
        if mode_id == 0:
            raise ValueError("mode must be PV or MIT")
        result = self._lib.articore_runtime_enable(self._ptr, mode_id)
        if result != 0:
            raise NativeEnableError(self.last_enable_report)

    @property
    def last_enable_report(self) -> EnableReport:
        """返回最近一次原子使能事务的结构化结果。"""
        native = _EnableReport()
        native.struct_size = ctypes.sizeof(_EnableReport)
        self._ok(
            self._lib.articore_runtime_get_last_enable_report(
                self._ptr,
                ctypes.byref(native),
            ),
            "runtime enable report",
        )
        missing_count = min(int(native.missing_count), 32)
        motor_count = min(int(native.motor_count), 32)
        return EnableReport(
            success=bool(native.success),
            disable_confirmed=bool(native.disable_confirmed),
            expected_count=int(native.expected_count),
            enabled_count=int(native.enabled_count),
            missing_count=int(native.missing_count),
            failure_count=int(native.failure_count),
            missing_motors=tuple(
                MissingEnableMotor(
                    side=int(native.missing_motor_sides[index]),
                    can_id=int(native.missing_motor_ids[index]),
                )
                for index in range(missing_count)
            ),
            motors=tuple(
                EnableMotorResult(
                    side=int(value.side),
                    name=_text(value.name) or "",
                    can_id=int(value.can_id),
                    status_code=int(value.status_code),
                    has_feedback=bool(value.has_feedback),
                    feedback_fresh=bool(value.feedback_fresh),
                    enabled=bool(value.enabled),
                )
                for value in native.motors[:motor_count]
            ),
            error=_text(native.error),
        )

    def submit_pos_vel(
        self,
        commands: Sequence[object],
    ) -> None:
        count = len(commands)
        if self._pv_array is None or len(self._pv_array) != count:
            self._pv_array = (_PosVelCommand * count)()
        for index, command in enumerate(commands):
            pointer = _pointer(command.motor, name="PV motor")
            if self._pv_array[index].motor != pointer:
                self._pv_array[index].motor = pointer
            self._pv_array[index].target_position = float(command.pos)
            self._pv_array[index].velocity_limit = float(command.vlim)
        self._ok(
            self._lib.articore_runtime_submit_pos_vel_ex(
                self._ptr,
                self._pv_array,
                count,
                1,  # ARTICORE_COMMAND_STREAMING
            ),
            "runtime PV submit",
        )

    def _mit_commands(self, commands: Sequence[object]):
        count = len(commands)
        array = self._arm_mit_array
        if array is None or len(array) != count:
            array = (_MitCommand * count)()
            self._arm_mit_array = array
        for index, command in enumerate(commands):
            native = array[index]
            pointer = _pointer(command.motor, name="MIT motor")
            if native.motor != pointer:
                native.motor = pointer
            native.target_position = float(command.pos)
            native.target_velocity = float(command.vel)
            native.stiffness = float(command.kp)
            native.damping = float(command.kd)
            native.feedforward_torque = float(command.tau)
        return array, count

    def submit_mit(
        self,
        commands: Sequence[object],
    ) -> None:
        array, count = self._mit_commands(commands)
        self._ok(
            self._lib.articore_runtime_submit_mit_ex(
                self._ptr,
                array,
                count,
                1,  # ARTICORE_COMMAND_STREAMING
            ),
            "runtime MIT submit",
        )

    def _set_joint_positions(
        self,
        targets: Sequence[tuple[object, float]],
        velocity: float,
        *,
        mit: bool,
    ) -> None:
        """提交完整普通关节位置批次；500 Hz 限步由 Runtime 执行。"""
        values = tuple(targets)
        if not values:
            raise ValueError("ordinary joint position targets must not be empty")
        reference_velocity = float(velocity)
        if not math.isfinite(reference_velocity) or reference_velocity <= 0.0:
            raise ValueError("joint reference velocity must be finite and positive")
        structure = _JointMitTarget if mit else _JointPvTarget
        attribute = "_joint_mit_target_array" if mit else "_joint_pv_target_array"
        array = getattr(self, attribute)
        if array is None or len(array) != len(values):
            array = (structure * len(values))()
            setattr(self, attribute, array)
        for index, (motor, position) in enumerate(values):
            array[index] = structure(
                ctypes.sizeof(structure),
                _pointer(motor, name="ordinary joint position motor"),
                float(position),
            )
        function = (
            self._lib.articore_runtime_set_joint_mit
            if mit
            else self._lib.articore_runtime_set_joint_pv
        )
        mode = "MIT" if mit else "PV"
        self._ok(
            function(self._ptr, array, len(values), reference_velocity),
            f"runtime ordinary {mode} position submit",
        )

    def set_joint_mit(
        self,
        targets: Sequence[tuple[object, float]],
        velocity: float,
    ) -> None:
        self._set_joint_positions(targets, velocity, mit=True)

    def set_joint_pv(
        self,
        targets: Sequence[tuple[object, float]],
        velocity: float,
    ) -> None:
        self._set_joint_positions(targets, velocity, mit=False)

    def set_gripper_commands(
        self,
        commands: Sequence[tuple[object, float, float, GripperForceLevel]],
    ) -> None:
        """原子提交所有实际安装夹爪的开合度、归一化速度和力等级。"""
        values = tuple(commands)
        if not values:
            raise ValueError("gripper commands must not be empty")
        count = len(values)
        if (
            self._gripper_command_array is None
            or len(self._gripper_command_array) != count
        ):
            self._gripper_command_array = (_GripperCommand * count)()
        for index, (motor, opening, speed, force_level) in enumerate(values):
            self._gripper_command_array[index] = _GripperCommand(
                ctypes.sizeof(_GripperCommand),
                _pointer(motor, name="gripper command motor"),
                float(opening),
                float(speed),
                int(GripperForceLevel(force_level)),
            )
        self._ok(
            self._lib.articore_runtime_set_gripper_commands(
                self._ptr,
                self._gripper_command_array,
                count,
            ),
            "runtime gripper command submit",
        )

    def report_feedback_failure(self, side: int, reason: str) -> None:
        self._ok(
            self._lib.articore_runtime_report_feedback_failure(
                self._ptr, side, reason.encode()
            ),
            "runtime feedback failure report",
        )

    def disable(self) -> None:
        if self._lib.articore_runtime_disable(self._ptr) != 0:
            raise NativeDisableError(
                self.last_disable_report,
                operation="runtime disable",
            )

    @property
    def last_disable_report(self) -> DisableReport:
        """返回最近一次确定性失能事务的结构化结果。"""
        native = _DisableReport()
        native.struct_size = ctypes.sizeof(_DisableReport)
        self._ok(
            self._lib.articore_runtime_get_last_disable_report(
                self._ptr,
                ctypes.byref(native),
            ),
            "runtime disable report",
        )
        missing_count = min(int(native.missing_count), 32)
        motor_count = min(int(native.motor_count), 32)
        return DisableReport(
            success=bool(native.success),
            barrier_confirmed=bool(native.barrier_confirmed),
            expected_count=int(native.expected_count),
            disabled_count=int(native.disabled_count),
            missing_count=int(native.missing_count),
            failure_count=int(native.failure_count),
            retry_count=int(native.retry_count),
            missing_motors=tuple(
                MissingDisableMotor(
                    side=int(native.missing_motor_sides[index]),
                    can_id=int(native.missing_motor_ids[index]),
                )
                for index in range(missing_count)
            ),
            motors=tuple(
                DisableMotorResult(
                    side=int(value.side),
                    name=_text(value.name) or "",
                    can_id=int(value.can_id),
                    status_code=int(value.status_code),
                    has_feedback=bool(value.has_feedback),
                    feedback_fresh=bool(value.feedback_fresh),
                    disabled=bool(value.disabled),
                    disable_sent=bool(value.disable_sent),
                    retry_sent=bool(value.retry_sent),
                )
                for value in native.motors[:motor_count]
            ),
            error=_text(native.error),
        )

    def estop(self, reason: str = "emergency stop") -> None:
        self._ok(self._lib.articore_runtime_estop(self._ptr, reason.encode()), "runtime estop")

    def recover(self) -> None:
        self._ok(self._lib.articore_runtime_recover(self._ptr), "runtime recover")

    @property
    def health(self) -> SafetyHealth:
        native = _SafetyHealth()
        self._ok(
            self._lib.articore_runtime_get_health(self._ptr, ctypes.byref(native)),
            "runtime health",
        )

        def transport(value: _TransportHealth) -> TransportHealth:
            return TransportHealth(
                connected=bool(value.connected),
                healthy=bool(value.healthy),
                consecutive_send_failures=int(value.consecutive_send_failures),
                consecutive_feedback_failures=int(value.consecutive_feedback_failures),
                last_feedback_age_s=_seconds(int(value.last_feedback_age_ns)),
                last_error=_text(value.last_error),
                tx_frames=int(value.tx_frames),
                rx_frames=int(value.rx_frames),
                send_errors=int(value.send_errors),
                receive_errors=int(value.receive_errors),
                last_tx_age_s=_seconds(int(value.last_tx_age_ns)),
                last_rx_age_s=_seconds(int(value.last_rx_age_ns)),
            )

        grippers: dict[int, GripperSafetyHealth] = {}
        for index in range(min(int(native.gripper_count), 2)):
            value = native.grippers[index]
            if not value.available:
                continue
            grippers[int(value.side)] = GripperSafetyHealth(
                name=_text(value.name) or "gripper",
                side=int(value.side),
                opening=float(value.opening),
                motor_position=float(value.motor_position),
                torque=float(value.torque),
                control_state=_GRIPPER_STATE_BY_CODE[int(value.control_state)],
                contact_detected=bool(value.contact_detected),
                stalled=bool(value.stalled),
                overload=bool(value.overload),
                hold_target=(
                    float(value.hold_target) if value.has_hold_target else None
                ),
                feedback_age_s=_seconds(int(value.feedback_age_ns)),
                fault_reason=_text(value.fault_reason),
            )

        return SafetyHealth(
            state=_STATE_BY_CODE[int(native.state)],
            fault_reason=_text(native.fault_reason),
            last_successful_command_age_s=_seconds(
                int(native.last_successful_command_age_ns)
            ),
            last_fresh_feedback_age_s=_seconds(int(native.last_fresh_feedback_age_ns)),
            consecutive_send_failures=int(native.consecutive_send_failures),
            consecutive_feedback_failures=int(native.consecutive_feedback_failures),
            left_transport=transport(native.left_transport),
            right_transport=transport(native.right_transport),
            motor_faults=tuple(
                _text(native.motor_faults[index]) or ""
                for index in range(int(native.motor_fault_count))
            ),
            unconfirmed_disable_motors=tuple(
                _text(native.unconfirmed_disable[index]) or ""
                for index in range(int(native.unconfirmed_disable_count))
            ),
            safe_holding=bool(native.safe_holding),
            disable_confirmed=bool(native.disable_confirmed),
            left_gripper=grippers.get(0),
            right_gripper=grippers.get(1),
        )

    def close(self) -> None:
        if self._ptr:
            if self._lib.articore_runtime_close(self._ptr) != 0:
                # ABI 2.0 要求关闭失败时保留 Runtime。它仍拥有
                # ControllerGroup、Controller 和 Transport 的生命周期前置条件。
                raise NativeDisableError(
                    self.last_disable_report,
                    operation="runtime close",
                )
            self._lib.articore_runtime_free(self._ptr)
            self._ptr = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass


__all__ = [
    "ARTICORE_CAP_DETERMINISTIC_DISABLE",
    "ARTICORE_CAP_GRIPPER_COMMAND_PROFILES",
    "ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS",
    "ARTICORE_CAP_JOINT_MIT_POSITION",
    "ARTICORE_CAP_JOINT_PV_POSITION",
    "ARTICORE_CAP_LAYERED_JOINT_LIMITS",
    "DisableMotorResult",
    "DisableReport",
    "EnableMotorResult",
    "EnableReport",
    "GripperControlState",
    "GripperForceLevel",
    "GripperSafetyHealth",
    "MissingEnableMotor",
    "MissingDisableMotor",
    "NativeDisableError",
    "NativeEnableError",
    "NativeJointControlConfig",
    "NativeJointSafetyLimits",
    "NativeGripperForceProfile",
    "NativeMotorDescriptor",
    "NativeSafetyRuntime",
    "SafetyHealth",
    "SafetyState",
    "TransportHealth",
]
