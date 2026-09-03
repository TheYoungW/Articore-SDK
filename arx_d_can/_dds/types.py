"""Cyclone DDS Python types matching motorbridge ``articore_protocol.idl``.

Keep this module wire-only: public SDK models and behavior belong outside the
generated-protocol boundary.
"""
from dataclasses import dataclass
from enum import auto

from cyclonedds.idl import IdlEnum, IdlStruct
from cyclonedds.idl.annotations import final, key
from cyclonedds.idl.types import (
    array,
    bounded_str,
    float32,
    uint16,
    uint32,
    uint64,
)


PROTOCOL_MAJOR = 1
PROTOCOL_MINOR = 2
ARM_DOF = 7
ROBOT_DOF = 14
POSE_DOF = 6

DISCOVERY_TOPIC = "articore.robot.discovery"
CONTROL_REQUEST_TOPIC = "articore.robot.control.request"
CONTROL_REPLY_TOPIC = "articore.robot.control.reply"
STREAM_COMMAND_TOPIC = "articore.robot.stream.command"
STATE_TOPIC = "articore.robot.state"
HEALTH_TOPIC = "articore.robot.health"
MOTION_EVENT_TOPIC = "articore.robot.motion.event"


class ProtocolError(IdlEnum, typename="articore_wire.ProtocolError"):
    OK = auto()
    INVALID_ARGUMENT = auto()
    WRONG_STATE = auto()
    BUSY = auto()
    NO_LEASE = auto()
    STALE_SEQUENCE = auto()
    TIMEOUT = auto()
    TRANSPORT_ERROR = auto()
    VERSION_MISMATCH = auto()
    INTERNAL_ERROR = auto()


class ControlOperation(IdlEnum, typename="articore_wire.ControlOperation"):
    ACQUIRE_LEASE = auto()
    HEARTBEAT = auto()
    RELEASE_LEASE = auto()
    CONNECT = auto()
    DISCONNECT = auto()
    CONFIGURE_MODE = auto()
    ENABLE = auto()
    DISABLE = auto()
    SET_ZERO = auto()
    CLEAR_FAULTS = auto()
    ESTOP = auto()
    RECOVER = auto()
    SET_SPEED_PERCENT = auto()
    SET_MAX_SPEED = auto()
    SET_MAX_ACCELERATION = auto()
    SOLVE_IK = auto()
    MOVE_POSE = auto()
    MOVE_LINEAR = auto()
    MOVE_CIRCULAR = auto()
    STOP_MOTION = auto()
    SET_GRIPPERS = auto()
    SET_TCP_OFFSET = auto()
    RESET_TCP_OFFSET = auto()
    START_GRAVITY_COMPENSATION = auto()
    STOP_GRAVITY_COMPENSATION = auto()
    START_BIMANUAL_FOLLOW = auto()
    STOP_BIMANUAL_FOLLOW = auto()
    QUERY_STATE = auto()
    QUERY_HEALTH = auto()
    GET_POSE = auto()
    GET_TCP_OFFSET = auto()
    GET_JOINT_LIMITS = auto()
    GET_SPEED_PERCENT = auto()
    GET_MAX_SPEED = auto()
    GET_MAX_ACCELERATION = auto()
    HAS_GRIPPERS = auto()
    GET_GRAVITY_STATUS = auto()
    GET_BIMANUAL_STATUS = auto()
    GET_HARDWARE_TOPOLOGY = auto()


class StreamKind(IdlEnum, typename="articore_wire.StreamKind"):
    PV = auto()
    MIT = auto()
    MIT_FAST = auto()


class MotionEventKind(IdlEnum, typename="articore_wire.MotionEventKind"):
    MOTION_ACCEPTED = auto()
    MOTION_COMPLETED = auto()
    MOTION_CANCELLED = auto()
    MOTION_FAILED = auto()


@final
@dataclass
class Discovery(IdlStruct, typename="articore_wire.Discovery"):
    protocol_major: uint16
    protocol_minor: uint16
    robot_id: bounded_str[64]
    client_id: bounded_str[64]
    security_identity: bounded_str[128]
    sequence_id: uint64
    service_version: bounded_str[64]
    domain_id: uint32
    ready: bool

    key("robot_id")


@final
@dataclass
class ControlRequest(IdlStruct, typename="articore_wire.ControlRequest"):
    protocol_major: uint16
    protocol_minor: uint16
    robot_id: bounded_str[64]
    client_id: bounded_str[64]
    security_identity: bounded_str[128]
    sequence_id: uint64
    request_id: uint64
    lease_id: uint64
    operation: ControlOperation
    side: uint32
    mode: uint32
    scalar: array[float32, 8]
    positions: array[float32, 14]
    pose_a: array[float32, 6]
    pose_b: array[float32, 6]
    pose_c: array[float32, 6]

    key("robot_id")


@final
@dataclass
class ControlReply(IdlStruct, typename="articore_wire.ControlReply"):
    protocol_major: uint16
    protocol_minor: uint16
    robot_id: bounded_str[64]
    client_id: bounded_str[64]
    security_identity: bounded_str[128]
    sequence_id: uint64
    request_id: uint64
    lease_id: uint64
    error: ProtocolError
    message: bounded_str[512]
    values: array[float32, 64]

    key("robot_id")


@final
@dataclass
class StreamCommand(IdlStruct, typename="articore_wire.StreamCommand"):
    protocol_major: uint16
    protocol_minor: uint16
    robot_id: bounded_str[64]
    client_id: bounded_str[64]
    security_identity: bounded_str[128]
    sequence_id: uint64
    lease_id: uint64
    kind: StreamKind
    positions: array[float32, 14]
    velocities: array[float32, 14]
    kp: array[float32, 14]
    kd: array[float32, 14]
    feedforward_torques: array[float32, 14]
    speed_percent: float32

    key("robot_id")


@final
@dataclass
class RobotState(IdlStruct, typename="articore_wire.RobotState"):
    protocol_major: uint16
    protocol_minor: uint16
    robot_id: bounded_str[64]
    client_id: bounded_str[64]
    security_identity: bounded_str[128]
    sequence_id: uint64
    source_timestamp_ns: uint64
    positions: array[float32, 14]
    velocities: array[float32, 14]
    torques: array[float32, 14]
    mos_temperatures: array[float32, 14]
    rotor_temperatures: array[float32, 14]
    enabled_mask: uint32
    enabled_valid_mask: uint32
    temperature_valid_mask: uint32
    gripper_openings: array[float32, 2]
    gripper_available: array[bool, 2]
    gripper_feedback_valid: array[bool, 2]
    motion_arrived: bool

    key("robot_id")


@final
@dataclass
class Health(IdlStruct, typename="articore_wire.Health"):
    protocol_major: uint16
    protocol_minor: uint16
    robot_id: bounded_str[64]
    client_id: bounded_str[64]
    security_identity: bounded_str[128]
    sequence_id: uint64
    state: uint32
    safe_holding: bool
    disable_confirmed: bool
    degraded: bool
    safe_stopped: bool
    requires_resynchronization: bool
    consecutive_send_failures: uint32
    consecutive_feedback_failures: uint32
    fault_reason: bounded_str[512]
    safety_reason: bounded_str[512]

    key("robot_id")


@final
@dataclass
class MotionEvent(IdlStruct, typename="articore_wire.MotionEvent"):
    protocol_major: uint16
    protocol_minor: uint16
    robot_id: bounded_str[64]
    client_id: bounded_str[64]
    security_identity: bounded_str[128]
    sequence_id: uint64
    request_id: uint64
    kind: MotionEventKind
    error: ProtocolError
    message: bounded_str[512]

    key("robot_id")


__all__ = [
    "PROTOCOL_MAJOR",
    "PROTOCOL_MINOR",
    "ControlOperation",
    "ControlReply",
    "ControlRequest",
    "Discovery",
    "Health",
    "MotionEvent",
    "MotionEventKind",
    "ProtocolError",
    "RobotState",
    "StreamCommand",
    "StreamKind",
]
