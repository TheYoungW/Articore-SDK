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


class MotionState(str, Enum):
    IDLE = "idle"
    QUEUED = "queued"
    RUNNING = "running"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    FAULT = "fault"


class MotionType(str, Enum):
    JOINT_TRAJECTORY = "joint_trajectory"
    CARTESIAN_LINEAR = "cartesian_linear"
    CARTESIAN_CIRCULAR = "cartesian_circular"


class RuntimeOperation(IntEnum):
    NONE = 0
    CONNECT = 1
    ENABLE = 2
    DISABLE = 3
    CONFIGURE_MODE = 4
    CLEAR_FAULTS = 5
    SET_ZERO = 6
    DISCONNECT = 7
    COMMAND = 8
    RECOVER = 9
    START_TRAJECTORY = 10
    CANCEL_MOTION = 11
    MOVE_POSE = 12
    CANCEL_ALL_MOTIONS = 13
    MOVE_LINEAR = 14
    MOVE_CIRCULAR = 15
    START_BIMANUAL_FOLLOW = 16
    STOP_BIMANUAL_FOLLOW = 17
    SET_TCP_OFFSET = 18


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


class BimanualFollowPhase(IntEnum):
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


@dataclass(frozen=True)
class ProductArmState:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    torques: tuple[float, ...]
    enabled: tuple[bool | None, ...] = (None,) * 7
    mos_temperatures: tuple[float | None, ...] = (None,) * 7
    rotor_temperatures: tuple[float | None, ...] = (None,) * 7


@dataclass(frozen=True)
class ProductGripperState:
    available: bool
    opening: float
    gripper_level: int
    enabled: bool | None = None
    mos_temperature: float | None = None
    rotor_temperature: float | None = None


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
class JointLimit:
    min_angle_rad: float
    max_angle_rad: float
    max_velocity_rad_s: float


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
class MotionStatus:
    state: MotionState
    motion_id: int
    motion_type: MotionType
    active_segment: int
    waypoint_count: int
    elapsed_s: float
    duration_s: float
    progress: float
    error: str | None


@dataclass(frozen=True)
class GravityCompensationStatus:
    phase: GravityCompensationPhase
    active: bool
    transition_progress: float
    control_cycles: int
    joint_count: int
    gravity_feedforward_torque: tuple[float, ...]


@dataclass(frozen=True)
class BimanualFollowStatus:
    phase: BimanualFollowPhase
    active: bool
    leader: str
    follower: str
    transition_progress: float
    control_cycles: int
    leader_positions: tuple[float, ...]
    follower_target_positions: tuple[float, ...]
    max_tracking_error: float
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
