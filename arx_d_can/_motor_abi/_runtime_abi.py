from __future__ import annotations

import ctypes
import ctypes.util
import os
import sys
from ctypes import (
    POINTER,
    Structure,
    c_char,
    c_char_p,
    c_float,
    c_int32,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
)
from importlib.metadata import PackageNotFoundError, distribution
from pathlib import Path

from .errors import AbiLoadError


RUNTIME_ABI_VERSION = 0x000B0004


def _runtime_library_name() -> str:
    if sys.platform.startswith("win"):
        return "articore_runtime.dll"
    if sys.platform == "darwin":
        return "libarticore_runtime.dylib"
    return "libarticore_runtime.so"


def _runtime_library_candidates() -> list[Path]:
    name = _runtime_library_name()
    candidates: list[Path] = []
    override = os.getenv("ARTICORE_RUNTIME_LIB")
    if override:
        candidates.append(Path(override).expanduser())

    try:
        package = distribution("motor-drive-layer")
    except PackageNotFoundError:
        package = None
    if package is not None:
        suffix = ("motor_drive_layer_native", "lib", name)
        for entry in package.files or ():
            if tuple(entry.parts[-len(suffix) :]) == suffix:
                candidates.append(Path(package.locate_file(entry)).resolve())

    source = os.getenv("MOTOR_DRIVE_LAYER_SOURCE")
    if source:
        root = Path(source).expanduser().resolve()
        candidates.extend(
            (
                root / "build" / "articore_runtime" / name,
                root / "build" / "articore_runtime" / "Release" / name,
            )
        )
    return candidates


def runtime_library_path() -> str:
    tried: list[str] = []
    for candidate in _runtime_library_candidates():
        tried.append(str(candidate))
        if candidate.is_file():
            return str(candidate)
    found = ctypes.util.find_library("articore_runtime")
    if found:
        return found
    detail = "\n".join(f"- {item}" for item in tried) or "- no candidates"
    raise AbiLoadError(
        "cannot locate libarticore_runtime; install the required "
        f"motor-drive-layer wheel or set ARTICORE_RUNTIME_LIB. Tried:\n{detail}"
    )


class CMotorPowerResult(Structure):
    _fields_ = [
        ("side", c_uint8), ("can_id", c_uint8),
        ("requested_enabled", c_uint8), ("command_sent", c_uint8),
        ("rollback_sent", c_uint8), ("has_feedback", c_uint8),
        ("feedback_fresh", c_uint8), ("status_code", c_uint8),
        ("confirmed", c_uint8), ("role", c_char * 64),
        ("error", c_char * 256),
    ]


class CMotorPowerReport(Structure):
    _fields_ = [
        ("struct_size", c_uint32), ("success", c_int32),
        ("requested_enabled", c_int32), ("rollback_attempted", c_int32),
        ("rollback_confirmed", c_int32), ("requested_count", c_uint32),
        ("command_sent_count", c_uint32), ("confirmed_count", c_uint32),
        ("failure_count", c_uint32), ("motor_count", c_uint32),
        ("motors", CMotorPowerResult * 32), ("error", c_char * 512),
    ]


class CRuntimeTransportHealth(Structure):
    _fields_ = [
        ("connected", c_int32), ("healthy", c_int32),
        ("consecutive_send_failures", c_uint32), ("consecutive_feedback_failures", c_uint32),
        ("last_feedback_age_ns", c_uint64), ("tx_frames", c_uint64), ("rx_frames", c_uint64),
        ("send_errors", c_uint64), ("receive_errors", c_uint64),
        ("last_tx_age_ns", c_uint64), ("last_rx_age_ns", c_uint64),
        ("last_error", c_char * 256),
    ]


class CGripperHealth(Structure):
    _fields_ = [
        ("available", c_int32), ("side", c_uint8), ("control_state", c_int32),
        ("opening", c_float), ("motor_position", c_float), ("torque", c_float),
        ("contact_detected", c_int32), ("stalled", c_int32), ("overload", c_int32),
        ("has_hold_target", c_int32), ("hold_target", c_float),
        ("feedback_age_ns", c_uint64), ("name", c_char * 64), ("fault_reason", c_char * 256),
    ]


class CMotorFeedbackHealth(Structure):
    _fields_ = [
        ("side", c_uint32),
        ("can_id", c_uint32),
        ("can_id_valid", c_uint32),
        ("is_gripper", c_uint32),
        ("has_feedback", c_uint32),
        ("fresh", c_uint32),
        ("has_state", c_uint32),
        ("values_finite", c_uint32),
        ("status_code", c_uint32),
        ("issues", c_uint32),
        ("position", c_float),
        ("velocity", c_float),
        ("torque", c_float),
        ("feedback_age_ns", c_uint64),
        ("update_count", c_uint64),
        ("role", c_char * 64),
    ]


class CSafetyHealth(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("state", c_int32), ("safe_holding", c_int32), ("disable_confirmed", c_int32),
        ("last_successful_command_age_ns", c_uint64), ("last_fresh_feedback_age_ns", c_uint64),
        ("consecutive_send_failures", c_uint32), ("consecutive_feedback_failures", c_uint32),
        ("left_transport", CRuntimeTransportHealth), ("right_transport", CRuntimeTransportHealth),
        ("motor_feedback_count", c_uint32),
        ("feedback_issue_count", c_uint32),
        ("feedback_issue_scope", c_int32),
        ("motor_feedback", CMotorFeedbackHealth * 32),
        ("gripper_count", c_uint32), ("grippers", CGripperHealth * 2),
        ("motor_fault_count", c_uint32), ("motor_faults", (c_char * 64) * 32),
        ("unconfirmed_disable_count", c_uint32), ("unconfirmed_disable", (c_char * 64) * 32),
        ("fault_reason", c_char * 512),
        ("last_operation", c_int32), ("last_operation_code", c_int32),
        ("operation_failed_motor_count", c_uint32),
        ("operation_failed_motors", (c_char * 64) * 32),
        ("last_operation_error", c_char * 512),
        ("degraded", c_int32), ("safe_stopped", c_int32),
        ("requires_resynchronization", c_int32), ("command_scale", c_float),
        ("safety_reason", c_char * 512),
    ]


class CProductArmState(Structure):
    _fields_ = [
        ("positions", c_float * 7),
        ("velocities", c_float * 7),
        ("torques", c_float * 7),
        ("mos_temperatures", c_float * 7),
        ("rotor_temperatures", c_float * 7),
        ("enabled_mask", c_uint32),
        ("enabled_valid_mask", c_uint32),
        ("temperature_valid_mask", c_uint32),
    ]


class CProductState(Structure):
    _fields_ = [
        ("struct_size", c_uint32), ("has_grippers", c_int32),
        ("left", CProductArmState), ("right", CProductArmState),
        ("left_gripper_available", c_int32), ("right_gripper_available", c_int32),
        ("left_gripper_opening", c_float), ("right_gripper_opening", c_float),
        ("left_gripper_level", c_int32), ("right_gripper_level", c_int32),
        ("left_gripper_enabled", c_int32),
        ("right_gripper_enabled", c_int32),
        ("left_gripper_enabled_valid", c_int32),
        ("right_gripper_enabled_valid", c_int32),
        ("left_gripper_mos_temperature", c_float),
        ("left_gripper_rotor_temperature", c_float),
        ("right_gripper_mos_temperature", c_float),
        ("right_gripper_rotor_temperature", c_float),
        ("left_gripper_temperature_valid", c_int32),
        ("right_gripper_temperature_valid", c_int32),
        ("timestamp_ns", c_uint64), ("sequence", c_uint64),
    ]


class CProductJointAngleVelLimits(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("joint_count", c_uint32),
        ("lower_angles", c_float * 14),
        ("upper_angles", c_float * 14),
        ("velocity_limits", c_float * 14),
    ]


class CProductPose(Structure):
    _fields_ = [
        ("struct_size", c_uint32), ("side", c_uint32),
        ("values", c_float * 6), ("timestamp_ns", c_uint64),
        ("sequence", c_uint64),
    ]


class CTcpOffset(Structure):
    _fields_ = [
        ("struct_size", c_uint32), ("side", c_uint32),
        ("values", c_float * 6),
    ]


class CTrajectoryWaypoint(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("time_s", ctypes.c_double),
        ("left_positions", c_float * 7),
        ("right_positions", c_float * 7),
        ("left_velocities", c_float * 7),
        ("right_velocities", c_float * 7),
        ("left_accelerations", c_float * 7),
        ("right_accelerations", c_float * 7),
        ("velocity_valid_mask", c_uint32),
        ("acceleration_valid_mask", c_uint32),
    ]


class CTrajectoryConfig(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("interpolation", c_int32),
        ("control_mode", c_int32),
        ("mit_kp", c_float * 14),
        ("mit_kd", c_float * 14),
        ("mit_feedforward_torque", c_float * 14),
        ("pv_velocity_limits", c_float * 14),
    ]


class CMotionStatus(Structure):
    _fields_ = [
        ("struct_size", c_uint32),
        ("motion_id", c_uint64),
        ("motion_type", c_int32),
        ("state", c_int32),
        ("active_segment", c_uint32),
        ("waypoint_count", c_uint32),
        ("elapsed_s", ctypes.c_double),
        ("duration_s", ctypes.c_double),
        ("progress", c_float),
        ("error", c_char * 512),
    ]


class CGravityCompensationConfig(Structure):
    _fields_ = [("struct_size", c_uint32), ("transition_ms", c_uint32)]


class CGravityCompensationStatus(Structure):
    _fields_ = [
        ("struct_size", c_uint32), ("phase", c_int32), ("active", c_int32),
        ("transition_progress", c_float), ("control_cycles", c_uint64),
        ("joint_count", c_uint32),
        ("gravity_feedforward_torque", c_float * 14),
    ]


class CBimanualFollowStatus(Structure):
    _fields_ = [
        ("struct_size", c_uint32), ("phase", c_int32), ("active", c_int32),
        ("leader_side", c_uint32), ("follower_side", c_uint32),
        ("transition_progress", c_float), ("control_cycles", c_uint64),
        ("leader_positions", c_float * 7),
        ("follower_target_positions", c_float * 7),
        ("max_tracking_error", c_float), ("error", c_char * 512),
    ]


class RuntimeAbi:
    def __init__(self) -> None:
        self.lib = ctypes.CDLL(runtime_library_path())
        self.lib.articore_runtime_abi_version.argtypes = []
        self.lib.articore_runtime_abi_version.restype = c_uint32
        version = int(self.lib.articore_runtime_abi_version())
        if version != RUNTIME_ABI_VERSION:
            raise AbiLoadError(
                "Articore-SDK requires Runtime ABI exactly 11.4; "
                f"loaded {version >> 16}.{version & 0xFFFF}"
            )
        self.abi_version = version
        self._bind()

    def _bind(self) -> None:
        lib = self.lib
        lib.articore_runtime_last_error.restype = c_char_p
        lib.articore_runtime_create_yunyi.argtypes = [
            c_int32, c_int32, POINTER(c_void_p),
        ]
        lib.articore_runtime_create_yunyi.restype = c_int32
        lib.articore_runtime_free.argtypes = [c_void_p]

        for name in ("connect", "disconnect", "disable", "recover", "clear_faults", "set_zero"):
            function = getattr(lib, f"articore_runtime_{name}")
            function.argtypes = [c_void_p]
            function.restype = c_int32

        lib.articore_runtime_enable.argtypes = [c_void_p]
        lib.articore_runtime_enable.restype = c_int32
        role_array = POINTER(c_char_p)
        lib.articore_runtime_enable_motors.argtypes = [
            c_void_p, role_array, c_uint32, POINTER(CMotorPowerReport),
        ]
        lib.articore_runtime_enable_motors.restype = c_int32
        lib.articore_runtime_disable_motors.argtypes = [
            c_void_p, role_array, c_uint32, POINTER(CMotorPowerReport),
        ]
        lib.articore_runtime_disable_motors.restype = c_int32
        lib.articore_runtime_estop.argtypes = [c_void_p]
        lib.articore_runtime_estop.restype = c_int32
        lib.articore_runtime_configure_mode.argtypes = [c_void_p, c_int32]
        lib.articore_runtime_configure_mode.restype = c_int32
        lib.articore_runtime_get_control_mode.argtypes = [c_void_p, POINTER(c_int32)]
        lib.articore_runtime_get_control_mode.restype = c_int32

        float_pointer = POINTER(c_float)
        lib.articore_runtime_set_joint_pv.argtypes = [
            c_void_p, float_pointer, c_uint32, c_float,
        ]
        lib.articore_runtime_set_joint_pv.restype = c_int32
        lib.articore_runtime_set_joint_mit_direct.argtypes = [
            c_void_p, float_pointer, c_uint32,
        ]
        lib.articore_runtime_set_joint_mit_direct.restype = c_int32
        lib.articore_runtime_set_joint_mit_fast_follow.argtypes = [
            c_void_p, float_pointer, c_uint32,
        ]
        lib.articore_runtime_set_joint_mit_fast_follow.restype = c_int32
        lib.articore_runtime_submit_mit_frame.argtypes = [
            c_void_p, float_pointer, float_pointer, float_pointer,
            float_pointer, float_pointer, c_uint32,
        ]
        lib.articore_runtime_submit_mit_frame.restype = c_int32
        lib.articore_runtime_move_joint_trajectory.argtypes = [
            c_void_p,
            POINTER(CTrajectoryWaypoint),
            c_uint32,
            POINTER(CTrajectoryConfig),
            POINTER(c_uint64),
        ]
        lib.articore_runtime_move_joint_trajectory.restype = c_int32
        lib.articore_runtime_set_grippers.argtypes = [
            c_void_p, c_float, c_float, c_int32, c_int32,
        ]
        lib.articore_runtime_set_grippers.restype = c_int32
        lib.articore_runtime_has_grippers.argtypes = [c_void_p, POINTER(c_int32)]
        lib.articore_runtime_has_grippers.restype = c_int32

        lib.articore_runtime_get_state.argtypes = [c_void_p, POINTER(CProductState)]
        lib.articore_runtime_get_state.restype = c_int32
        lib.articore_runtime_get_joint_angle_vel_limits.argtypes = [
            c_void_p, POINTER(CProductJointAngleVelLimits),
        ]
        lib.articore_runtime_get_joint_angle_vel_limits.restype = c_int32
        lib.articore_runtime_get_pose.argtypes = [c_void_p, c_uint32, POINTER(CProductPose)]
        lib.articore_runtime_get_pose.restype = c_int32
        lib.articore_runtime_solve_ik.argtypes = [
            c_void_p,
            float_pointer,
            float_pointer,
            float_pointer,
            c_uint32,
        ]
        lib.articore_runtime_solve_ik.restype = c_int32
        lib.articore_runtime_set_tcp_offset.argtypes = [
            c_void_p, POINTER(CTcpOffset),
        ]
        lib.articore_runtime_set_tcp_offset.restype = c_int32
        lib.articore_runtime_get_tcp_offset.argtypes = [
            c_void_p, c_uint32, POINTER(CTcpOffset),
        ]
        lib.articore_runtime_get_tcp_offset.restype = c_int32
        lib.articore_runtime_reset_tcp_offset.argtypes = [c_void_p, c_uint32]
        lib.articore_runtime_reset_tcp_offset.restype = c_int32
        lib.articore_runtime_set_pose.argtypes = [
            c_void_p, float_pointer, float_pointer, c_float,
        ]
        lib.articore_runtime_set_pose.restype = c_int32
        lib.articore_runtime_move_linear_trajectory.argtypes = [
            c_void_p, c_uint32, float_pointer, float_pointer,
            ctypes.c_double, POINTER(c_uint64),
        ]
        lib.articore_runtime_move_linear_trajectory.restype = c_int32
        lib.articore_runtime_move_linear_path_trajectory.argtypes = [
            c_void_p, c_uint32, float_pointer, c_uint32,
            ctypes.c_double, POINTER(c_uint64),
        ]
        lib.articore_runtime_move_linear_path_trajectory.restype = c_int32
        lib.articore_runtime_move_circular_trajectory.argtypes = [
            c_void_p, c_uint32, float_pointer, float_pointer, float_pointer,
            ctypes.c_double, POINTER(c_uint64),
        ]
        lib.articore_runtime_move_circular_trajectory.restype = c_int32
        lib.articore_runtime_get_motion_status.argtypes = [
            c_void_p, c_uint64, POINTER(CMotionStatus),
        ]
        lib.articore_runtime_get_motion_status.restype = c_int32
        lib.articore_runtime_cancel_motion.argtypes = [c_void_p, c_uint64]
        lib.articore_runtime_cancel_motion.restype = c_int32
        lib.articore_runtime_cancel_all_motions.argtypes = [c_void_p]
        lib.articore_runtime_cancel_all_motions.restype = c_int32
        lib.articore_runtime_get_health.argtypes = [c_void_p, POINTER(CSafetyHealth)]
        lib.articore_runtime_get_health.restype = c_int32

        lib.articore_runtime_start_gravity_compensation.argtypes = [
            c_void_p, POINTER(CGravityCompensationConfig),
        ]
        lib.articore_runtime_start_gravity_compensation.restype = c_int32
        lib.articore_runtime_stop_gravity_compensation.argtypes = [c_void_p]
        lib.articore_runtime_stop_gravity_compensation.restype = c_int32
        lib.articore_runtime_get_gravity_compensation_status.argtypes = [
            c_void_p, POINTER(CGravityCompensationStatus),
        ]
        lib.articore_runtime_get_gravity_compensation_status.restype = c_int32
        lib.articore_runtime_start_bimanual_follow.argtypes = [c_void_p, c_uint32]
        lib.articore_runtime_start_bimanual_follow.restype = c_int32
        lib.articore_runtime_stop_bimanual_follow.argtypes = [c_void_p]
        lib.articore_runtime_stop_bimanual_follow.restype = c_int32
        lib.articore_runtime_get_bimanual_follow_status.argtypes = [
            c_void_p, POINTER(CBimanualFollowStatus),
        ]
        lib.articore_runtime_get_bimanual_follow_status.restype = c_int32


_runtime_abi: RuntimeAbi | None = None


def get_runtime_abi() -> RuntimeAbi:
    global _runtime_abi
    if _runtime_abi is None:
        _runtime_abi = RuntimeAbi()
    return _runtime_abi
