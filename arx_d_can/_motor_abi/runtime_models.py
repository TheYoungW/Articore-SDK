from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum


class SafetyState(IntEnum):
    DISCONNECTED = 0
    READY = 1
    ENABLED = 2
    RUNNING = 3
    SAFE_HOLD = 4
    FAULT = 5
    DEGRADED = 6
    SAFE_STOP = 7
    PARTIALLY_ENABLED = 8


class RuntimeControlMode(IntEnum):
    PV = 1
    MIT = 2


class CartesianMotionState(str, Enum):
    IDLE = "idle"
    QUEUED = "queued"
    RUNNING = "running"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    FAULT = "fault"


class CartesianInterpolation(str, Enum):
    LINEAR = "linear"
    CIRCULAR = "circular"


class RuntimeOperation(IntEnum):
    NONE = 0
    CONNECT = 1
    ENABLE = 2
    DISABLE = 3
    CONFIGURE_MODE = 4
    CLEAR_FAULTS = 5
    SET_ZERO = 6
    CLOSE = 7
    DISCONNECT = 8
    COMMAND = 9
    RECOVER = 10
    START_TRAJECTORY = 11
    CANCEL_TRAJECTORY = 12
    MOVE_POSE = 13
    CANCEL_CARTESIAN_MOTION = 14
    MOVE_LINEAR = 15
    MOVE_CIRCULAR = 16


class OperationError(IntEnum):
    OK = 0
    INVALID_ARGUMENT = 1
    INVALID_STATE = 2
    TRANSPORT = 3
    FEEDBACK = 4
    NOT_DISABLED = 5
    NOT_STATIONARY = 6
    MOTOR_COMMAND = 7
    VERIFICATION = 8
    UNSUPPORTED = 9


class GravityCompensationPhase(IntEnum):
    INACTIVE = 0
    ENTERING = 1
    ACTIVE = 2
    EXITING = 3


class GripperControlState(IntEnum):
    DISABLED = 0
    IDLE = 1
    MOVING = 2
    CONTACT = 3
    HOLDING = 4
    OVERLOAD_RETREAT = 5
    FAULT = 6


class ConnectErrorCode(IntEnum):
    OK = 0
    CONFIGURATION = 1
    TRANSPORT = 2
    FEEDBACK_TIMEOUT = 3
    FEEDBACK_INCOMPLETE = 4
    FEEDBACK_INVALID = 5


@dataclass(frozen=True)
class ProductArmState:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    torques: tuple[float, ...]
    enabled: tuple[bool | None, ...] = (None,) * 7


@dataclass(frozen=True)
class ProductGripperState:
    available: bool
    opening: float
    gripper_level: int
    enabled: bool | None = None


@dataclass(frozen=True)
class ProductState:
    has_grippers: bool
    left: ProductArmState
    right: ProductArmState
    left_gripper: ProductGripperState | None
    right_gripper: ProductGripperState | None
    timestamp_ns: int
    sequence: int


@dataclass(frozen=True)
class ProductPose:
    side: int
    values: tuple[float, float, float, float, float, float]
    timestamp_ns: int
    sequence: int

    @property
    def x(self) -> float:
        return self.values[0]

    @property
    def y(self) -> float:
        return self.values[1]

    @property
    def z(self) -> float:
        return self.values[2]

    @property
    def roll(self) -> float:
        return self.values[3]

    @property
    def pitch(self) -> float:
        return self.values[4]

    @property
    def yaw(self) -> float:
        return self.values[5]


@dataclass(frozen=True)
class CartesianMotionStatus:
    state: CartesianMotionState
    motion_id: int
    superseded_motion_id: int
    side: str
    interpolation: CartesianInterpolation
    speed_percent: float
    elapsed_s: float
    duration_s: float
    progress: float
    target_pose: tuple[float, float, float, float, float, float]
    error: str | None


@dataclass(frozen=True)
class GravityCompensationStatus:
    phase: GravityCompensationPhase
    active: bool
    transition_progress: float
    control_cycles: int
    joints: tuple[str, ...]
    gravity_feedforward_torque: tuple[float, ...]


@dataclass(frozen=True)
class EnableMotorResult:
    side: int
    can_id: int
    status_code: int
    has_feedback: bool
    feedback_fresh: bool
    enabled: bool
    name: str


@dataclass(frozen=True)
class ConnectChannelResult:
    side: int
    active: bool
    request_code: int
    expected_count: int
    received_count: int
    missing_motor_ids: tuple[int, ...]
    error: str | None


@dataclass(frozen=True)
class ConnectMotorResult:
    side: int
    configured_can_id: int
    reported_can_id: int
    has_feedback: bool
    feedback_fresh: bool
    feedback_valid: bool
    update_count: int
    feedback_age_ns: int | None
    name: str
    error: str | None


@dataclass(frozen=True)
class ConnectReport:
    success: bool
    error_code: ConnectErrorCode
    expected_count: int
    received_count: int
    missing_count: int
    failure_count: int
    channels: tuple[ConnectChannelResult, ...]
    motors: tuple[ConnectMotorResult, ...]
    error: str | None


@dataclass(frozen=True)
class EnableReport:
    success: bool
    disable_confirmed: bool
    expected_count: int
    enabled_count: int
    missing_count: int
    failure_count: int
    missing_motors: tuple[tuple[int, int], ...]
    motors: tuple[EnableMotorResult, ...]
    error: str | None


@dataclass(frozen=True)
class DisableMotorResult:
    side: int
    can_id: int
    status_code: int
    has_feedback: bool
    feedback_fresh: bool
    disabled: bool
    disable_sent: bool
    retry_sent: bool
    name: str


@dataclass(frozen=True)
class DisableReport:
    success: bool
    barrier_confirmed: bool
    expected_count: int
    disabled_count: int
    missing_count: int
    failure_count: int
    retry_count: int
    missing_motors: tuple[tuple[int, int], ...]
    motors: tuple[DisableMotorResult, ...]
    error: str | None


@dataclass(frozen=True)
class MotorPowerResult:
    side: int
    can_id: int
    role: str
    requested_enabled: bool
    command_sent: bool
    rollback_sent: bool
    has_feedback: bool
    feedback_fresh: bool
    status_code: int
    confirmed: bool
    error: str | None


@dataclass(frozen=True)
class MotorPowerReport:
    success: bool
    requested_enabled: bool
    rollback_attempted: bool
    rollback_confirmed: bool
    requested_count: int
    command_sent_count: int
    confirmed_count: int
    failure_count: int
    motors: tuple[MotorPowerResult, ...]
    error: str | None


@dataclass(frozen=True)
class RuntimeTransportHealth:
    connected: bool
    healthy: bool
    consecutive_send_failures: int
    consecutive_feedback_failures: int
    last_feedback_age_ns: int | None
    tx_frames: int
    rx_frames: int
    send_errors: int
    receive_errors: int
    last_tx_age_ns: int | None
    last_rx_age_ns: int | None
    last_error: str | None


@dataclass(frozen=True)
class GripperHealth:
    available: bool
    side: int
    control_state: GripperControlState
    opening: float
    motor_position: float
    torque: float
    contact_detected: bool
    stalled: bool
    overload: bool
    hold_target: float | None
    feedback_age_ns: int | None
    name: str
    fault_reason: str | None


@dataclass(frozen=True)
class SafetyHealth:
    state: SafetyState
    safe_holding: bool
    disable_confirmed: bool
    last_successful_command_age_ns: int | None
    last_fresh_feedback_age_ns: int | None
    consecutive_send_failures: int
    consecutive_feedback_failures: int
    left_transport: RuntimeTransportHealth
    right_transport: RuntimeTransportHealth
    grippers: tuple[GripperHealth, ...]
    motor_faults: tuple[str, ...]
    unconfirmed_disable: tuple[str, ...]
    fault_reason: str | None
    last_operation: RuntimeOperation = RuntimeOperation.NONE
    last_operation_code: OperationError = OperationError.OK
    operation_failed_motors: tuple[str, ...] = ()
    last_operation_error: str | None = None
    degraded: bool = False
    safe_stopped: bool = False
    requires_resynchronization: bool = False
    command_scale: float = 1.0
    safety_reason: str | None = None
