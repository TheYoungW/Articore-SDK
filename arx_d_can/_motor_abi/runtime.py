from __future__ import annotations

import ctypes
import math
from collections.abc import Sequence
from threading import Event, RLock, Thread
from types import TracebackType

from ._runtime_abi import (
    CGravityCompensationConfig,
    CGravityCompensationStatus,
    CBimanualFollowStatus,
    CMotorPowerReport,
    CMotionStatus,
    CProductJointAngleVelLimits,
    CProductPose,
    CProductState,
    CRuntimeTransportHealth,
    CSafetyHealth,
    CTrajectoryConfig,
    CTrajectoryWaypoint,
    CTcpOffset,
    get_runtime_abi,
)
from .errors import RuntimeCallError, RuntimeTransactionError
from .runtime_models import (
    GripperControlState,
    GripperHealth,
    GravityCompensationPhase,
    GravityCompensationStatus,
    BimanualFollowPhase,
    BimanualFollowStatus,
    MotorPowerReport,
    MotorPowerResult,
    MotorFeedbackHealth,
    MotorFeedbackIssue,
    FeedbackIssueScope,
    MotionState,
    MotionStatus,
    MotionType,
    JointLimit,
    RuntimeControlMode,
    RuntimeOperation,
    OperationError,
    ProductArmState,
    ProductGripperState,
    ProductPose,
    ProductState,
    RuntimeTransportHealth,
    SafetyHealth,
    SafetyState,
)

_UINT64_MAX = (1 << 64) - 1


def _text(value: object) -> str:
    return bytes(value).split(b"\0", 1)[0].decode(errors="replace")


def _optional_text(value: object) -> str | None:
    result = _text(value)
    return result or None


def _optional_age(value: int) -> int | None:
    return None if int(value) == _UINT64_MAX else int(value)


def _motion_id(value: int) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError("motion_id must be a positive integer")
    return value


def _transport_health(value: CRuntimeTransportHealth) -> RuntimeTransportHealth:
    return RuntimeTransportHealth(
        connected=bool(value.connected),
        healthy=bool(value.healthy),
        consecutive_send_failures=int(value.consecutive_send_failures),
        consecutive_feedback_failures=int(value.consecutive_feedback_failures),
        last_feedback_age_ns=_optional_age(value.last_feedback_age_ns),
        tx_frames=int(value.tx_frames),
        rx_frames=int(value.rx_frames),
        send_errors=int(value.send_errors),
        receive_errors=int(value.receive_errors),
        last_tx_age_ns=_optional_age(value.last_tx_age_ns),
        last_rx_age_ns=_optional_age(value.last_rx_age_ns),
        last_error=_optional_text(value.last_error),
    )


def _pose(values: Sequence[float]) -> ctypes.Array[ctypes.c_float]:
    pose = tuple(float(value) for value in values)
    if len(pose) != 6:
        raise ValueError(
            "pose must contain exactly 6 values: x, y, z, roll, pitch, yaw"
        )
    if not all(math.isfinite(value) for value in pose):
        raise ValueError("pose values must all be finite")
    return (ctypes.c_float * 6)(*pose)


class ArticoreRuntime:
    """Yunyi 双臂产品 Runtime 的轻量 ctypes 绑定。"""

    def __init__(self) -> None:
        raise TypeError("use ArticoreRuntime.create_yunyi()")

    @classmethod
    def create_yunyi(
        cls,
        control_mode: RuntimeControlMode,
        with_grippers: bool = True,
    ) -> ArticoreRuntime:
        """创建完全由 C++ 拥有资源和配置的 Yunyi 双臂 Runtime。"""
        mode = RuntimeControlMode(control_mode)
        runtime_abi = get_runtime_abi()
        output = ctypes.c_void_p()
        result = int(runtime_abi.lib.articore_runtime_create_yunyi(
            int(mode), int(bool(with_grippers)), ctypes.byref(output),
        ))
        if result != 0 or not output.value:
            detail = runtime_abi.lib.articore_runtime_last_error()
            text = detail.decode(errors="replace") if detail else "unknown error"
            raise RuntimeCallError(f"create_yunyi failed: {text}")
        self = cls.__new__(cls)
        self._lock = RLock()
        self._fps = 0.0
        self._fps_stop = Event()
        self._fps_thread = None
        self._ptr = output.value
        self._runtime_abi = runtime_abi
        self._control_mode = mode
        return self

    @property
    def closed(self) -> bool:
        return not bool(self._ptr)

    def _last_error(self) -> str:
        value = self._runtime_abi.lib.articore_runtime_last_error()
        return value.decode(errors="replace") if value else "unknown Runtime error"

    def _require_open(self) -> int:
        if not self._ptr:
            raise RuntimeCallError("ArticoreRuntime is closed")
        return self._ptr

    def _call(self, function: object, operation: str, *args: object) -> None:
        with self._lock:
            rc = int(function(self._require_open(), *args))
            if rc != 0:
                raise RuntimeCallError(f"{operation} failed: {self._last_error()}")

    def get_fps(self) -> float:
        """立即返回最近一个 0.1 秒窗口计算出的接收帧率。"""
        return float(self._fps)

    def _received_frame_count(self) -> int:
        health = self.health
        return int(health.left_transport.rx_frames) + int(
            health.right_transport.rx_frames
        )

    def _start_fps_monitor(self) -> None:
        thread = self._fps_thread
        if thread is not None and thread.is_alive():
            return
        self._fps = 0.0
        self._fps_stop = Event()
        stop = self._fps_stop
        try:
            previous: int | None = self._received_frame_count()
        except Exception:
            previous = None

        def monitor() -> None:
            nonlocal previous
            while not stop.wait(0.1):
                try:
                    current = self._received_frame_count()
                except Exception:
                    previous = None
                    continue
                if stop.is_set():
                    break
                if previous is not None:
                    self._fps = float(max(0, current - previous) * 10)
                previous = current

        self._fps_thread = Thread(
            target=monitor,
            name="articore-runtime-fps",
            daemon=True,
        )
        self._fps_thread.start()

    def _stop_fps_monitor(self) -> None:
        thread = self._fps_thread
        if thread is None:
            self._fps = 0.0
            return
        self._fps_stop.set()
        thread.join(timeout=0.5)
        self._fps_thread = None
        self._fps = 0.0

    @property
    def control_mode(self) -> RuntimeControlMode:
        value = ctypes.c_int32()
        self._call(
            self._runtime_abi.lib.articore_runtime_get_control_mode,
            "get_control_mode", ctypes.byref(value),
        )
        return RuntimeControlMode(value.value)

    def start_gravity_compensation(self, transition_ms: int = 0) -> None:
        if not isinstance(transition_ms, int) or not 0 <= transition_ms <= 60_000:
            raise ValueError("transition_ms must be an integer in 0..60000")
        native = CGravityCompensationConfig()
        native.struct_size = ctypes.sizeof(native)
        native.transition_ms = transition_ms
        self._call(
            self._runtime_abi.lib.articore_runtime_start_gravity_compensation,
            "start_gravity_compensation", ctypes.byref(native),
        )

    def stop_gravity_compensation(self) -> None:
        self._call(
            self._runtime_abi.lib.articore_runtime_stop_gravity_compensation,
            "stop_gravity_compensation",
        )

    @property
    def gravity_compensation_status(self) -> GravityCompensationStatus:
        native = CGravityCompensationStatus()
        native.struct_size = ctypes.sizeof(native)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_gravity_compensation_status,
            "get_gravity_compensation_status", ctypes.byref(native),
        )
        count = min(int(native.joint_count), 14)
        return GravityCompensationStatus(
            phase=GravityCompensationPhase(native.phase),
            active=bool(native.active),
            transition_progress=float(native.transition_progress),
            control_cycles=int(native.control_cycles),
            joint_count=count,
            gravity_feedforward_torque=tuple(
                float(native.gravity_feedforward_torque[index])
                for index in range(count)
            ),
        )

    def start_bimanual_follow(self, leader_side: int) -> None:
        if leader_side not in (0, 1):
            raise ValueError("leader_side must be 0 (left) or 1 (right)")
        self._call(
            self._runtime_abi.lib.articore_runtime_start_bimanual_follow,
            "start_bimanual_follow", leader_side,
        )

    def stop_bimanual_follow(self) -> None:
        self._call(
            self._runtime_abi.lib.articore_runtime_stop_bimanual_follow,
            "stop_bimanual_follow",
        )

    @property
    def bimanual_follow_status(self) -> BimanualFollowStatus:
        native = CBimanualFollowStatus()
        native.struct_size = ctypes.sizeof(native)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_bimanual_follow_status,
            "get_bimanual_follow_status", ctypes.byref(native),
        )
        sides = ("left", "right")
        error = bytes(native.error).split(b"\0", 1)[0].decode(
            "utf-8", errors="replace",
        )
        return BimanualFollowStatus(
            phase=BimanualFollowPhase(native.phase),
            active=bool(native.active),
            leader=sides[native.leader_side],
            follower=sides[native.follower_side],
            transition_progress=float(native.transition_progress),
            control_cycles=int(native.control_cycles),
            leader_positions=tuple(float(value) for value in native.leader_positions),
            follower_target_positions=tuple(
                float(value) for value in native.follower_target_positions
            ),
            max_tracking_error=float(native.max_tracking_error),
            error=error or None,
        )

    def connect(self) -> None:
        self._call(self._runtime_abi.lib.articore_runtime_connect, "connect")
        self._start_fps_monitor()

    def _motor_power_report(self, native: CMotorPowerReport) -> MotorPowerReport:
        count = min(int(native.motor_count), 32)
        return MotorPowerReport(
            success=bool(native.success),
            requested_enabled=bool(native.requested_enabled),
            rollback_attempted=bool(native.rollback_attempted),
            rollback_confirmed=bool(native.rollback_confirmed),
            requested_count=int(native.requested_count),
            command_sent_count=int(native.command_sent_count),
            confirmed_count=int(native.confirmed_count),
            failure_count=int(native.failure_count),
            motors=tuple(
                MotorPowerResult(
                    side=int(item.side), can_id=int(item.can_id),
                    role=_text(item.role),
                    requested_enabled=bool(item.requested_enabled),
                    command_sent=bool(item.command_sent),
                    rollback_sent=bool(item.rollback_sent),
                    has_feedback=bool(item.has_feedback),
                    feedback_fresh=bool(item.feedback_fresh),
                    status_code=int(item.status_code),
                    confirmed=bool(item.confirmed),
                    error=_optional_text(item.error),
                )
                for item in native.motors[:count]
            ),
            error=_optional_text(native.error),
        )

    def _set_motor_power(self, motors: Sequence[str], *, enabled: bool) -> bool:
        encoded = tuple(role.encode("utf-8") for role in motors)
        roles = (ctypes.c_char_p * len(encoded))(*encoded)
        native = CMotorPowerReport()
        native.struct_size = ctypes.sizeof(native)
        function = (
            self._runtime_abi.lib.articore_runtime_enable_motors
            if enabled
            else self._runtime_abi.lib.articore_runtime_disable_motors
        )
        operation = "enable_motors" if enabled else "disable_motors"
        with self._lock:
            rc = int(function(
                self._require_open(), roles, len(encoded), ctypes.byref(native)
            ))
            report = self._motor_power_report(native)
            if rc != 0:
                raise RuntimeTransactionError(
                    f"{operation} failed: {self._last_error()}", report,
                )
        return True

    def enable(self, motors: Sequence[str] | None = None) -> bool:
        if motors is not None:
            return self._set_motor_power(motors, enabled=True)
        with self._lock:
            rc = int(self._runtime_abi.lib.articore_runtime_enable(
                self._require_open()
            ))
            if rc != 0:
                raise RuntimeCallError(f"enable failed: {self._last_error()}")
        return True

    def disable(self, motors: Sequence[str] | None = None) -> bool:
        if motors is not None:
            return self._set_motor_power(motors, enabled=False)
        with self._lock:
            rc = int(self._runtime_abi.lib.articore_runtime_disable(self._require_open()))
            if rc != 0:
                raise RuntimeCallError(f"disable failed: {self._last_error()}")
        return True

    def estop(self) -> None:
        """请求原生 Runtime 执行整机急停并锁存急停状态。"""
        self._call(self._runtime_abi.lib.articore_runtime_estop, "estop")

    def recover(self) -> None:
        """Recover the whole product to calibrated zero, then disable it."""
        self._call(self._runtime_abi.lib.articore_runtime_recover, "recover")

    def configure_mode(self, mode: RuntimeControlMode) -> None:
        selected = RuntimeControlMode(mode)
        self._call(
            self._runtime_abi.lib.articore_runtime_configure_mode,
            "configure_mode", int(selected),
        )
        self._control_mode = selected

    def disconnect(self) -> None:
        self._stop_fps_monitor()
        with self._lock:
            if not self._ptr:
                return
            pointer = self._ptr
            rc = int(
                self._runtime_abi.lib.articore_runtime_disconnect(pointer)
            )
            failure = self._last_error() if rc != 0 else None
            # The C ABI retains an opaque tombstone for idempotency. Python
            # owns that final allocation and always frees it internally; the
            # business API never exposes a separate close/free step.
            self._runtime_abi.lib.articore_runtime_free(pointer)
            self._ptr = None
            if failure is not None:
                raise RuntimeCallError(f"disconnect failed: {failure}")

    def set_max_acceleration(self, max_acceleration_rad_s2: float) -> None:
        """Set the ordinary-PV maximum acceleration in rad/s²."""
        self._call(
            self._runtime_abi.lib.articore_runtime_set_max_acceleration,
            "set_max_acceleration", float(max_acceleration_rad_s2),
        )

    def get_max_acceleration(self) -> float:
        """Return the ordinary-PV maximum acceleration in rad/s²."""
        value = ctypes.c_float()
        self._call(
            self._runtime_abi.lib.articore_runtime_get_max_acceleration,
            "get_max_acceleration", ctypes.byref(value),
        )
        return float(value.value)

    def get_joint_limits(self) -> tuple[JointLimit, ...]:
        """Return the fixed 14-joint logical product limit table."""
        native = CProductJointAngleVelLimits()
        native.struct_size = ctypes.sizeof(native)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_joint_angle_vel_limits,
            "get_joint_angle_vel_limits", ctypes.byref(native),
        )
        if int(native.joint_count) != 14:
            raise RuntimeCallError(
                "get_joint_angle_vel_limits returned "
                f"joint_count={int(native.joint_count)}, expected 14"
            )
        return tuple(
            JointLimit(
                min_angle_rad=float(native.lower_angles[index]),
                max_angle_rad=float(native.upper_angles[index]),
                max_velocity_rad_s=float(native.velocity_limits[index]),
            )
            for index in range(14)
        )

    def set_joint_pv(
        self,
        positions: Sequence[float],
        speed_percent: float = 50.0,
    ) -> None:
        values = tuple(float(value) for value in positions)
        native = (ctypes.c_float * len(values))(*values)
        self._call(
            self._runtime_abi.lib.articore_runtime_set_joint_pv,
            "set_joint_pv", native, len(values), float(speed_percent),
        )

    def set_joint_mit(
        self,
        positions: Sequence[float],
        speed_percent: float = 50.0,
    ) -> None:
        values = tuple(float(value) for value in positions)
        native = (ctypes.c_float * len(values))(*values)
        self._call(
            self._runtime_abi.lib.articore_runtime_set_joint_mit,
            "set_joint_mit", native, len(values), float(speed_percent),
        )

    def submit_mit_frame(
        self,
        positions: Sequence[float],
        velocities: Sequence[float] | None = None,
        feedforward_torques: Sequence[float] | None = None,
        kp: Sequence[float] | None = None,
        kd: Sequence[float] | None = None,
    ) -> None:
        q = tuple(float(value) for value in positions)
        def optional(source: Sequence[float] | None):
            if source is None:
                return None
            values = tuple(float(value) for value in source)
            return (ctypes.c_float * len(values))(*values)
        native_q = (ctypes.c_float * len(q))(*q)
        self._call(
            self._runtime_abi.lib.articore_runtime_submit_mit_frame,
            "submit_mit_frame", native_q, optional(velocities),
            optional(feedforward_torques), optional(kp), optional(kd), len(q),
        )

    def move_joint_trajectory(
        self,
        *,
        timestamps: Sequence[float],
        left_positions: Sequence[Sequence[float]],
        right_positions: Sequence[Sequence[float]],
        mit_kp: Sequence[float] | None = None,
        mit_kd: Sequence[float] | None = None,
        mit_feedforward_torques: Sequence[float] | None = None,
    ) -> int:
        """Copy positions and timestamps into the native planner and FIFO."""
        times = tuple(float(value) for value in timestamps)
        left_q = tuple(tuple(float(value) for value in row) for row in left_positions)
        right_q = tuple(tuple(float(value) for value in row) for row in right_positions)
        count = len(times)
        if count < 2 or len(left_q) != count or len(right_q) != count:
            raise ValueError(
                "trajectory timestamps and both position arrays must contain "
                "the same 2..30000 waypoint count"
            )
        if count > 30_000:
            raise ValueError("trajectory supports at most 30000 waypoints")

        if any(len(row) != 7 for row in (*left_q, *right_q)):
            raise ValueError("each trajectory arm position must contain 7 values")

        native_waypoints = (CTrajectoryWaypoint * count)()
        for index, waypoint in enumerate(native_waypoints):
            waypoint.struct_size = ctypes.sizeof(CTrajectoryWaypoint)
            waypoint.time_s = times[index]
            waypoint.left_positions[:] = left_q[index]
            waypoint.right_positions[:] = right_q[index]

        def vector(name: str, source: Sequence[float] | None) -> tuple[float, ...]:
            if source is None:
                return (0.0,) * 14
            values = tuple(float(value) for value in source)
            if len(values) != 14:
                raise ValueError(f"{name} must contain 14 values")
            return values

        native_config = CTrajectoryConfig()
        native_config.struct_size = ctypes.sizeof(CTrajectoryConfig)
        native_config.interpolation = 1
        native_config.control_mode = int(self._control_mode)
        native_config.mit_kp[:] = vector("mit_kp", mit_kp)
        native_config.mit_kd[:] = vector("mit_kd", mit_kd)
        native_config.mit_feedforward_torque[:] = vector(
            "mit_feedforward_torques", mit_feedforward_torques
        )
        motion_id = ctypes.c_uint64()
        self._call(
            self._runtime_abi.lib.articore_runtime_move_joint_trajectory,
            "move_joint_trajectory",
            native_waypoints,
            count,
            ctypes.byref(native_config),
            ctypes.byref(motion_id),
        )
        return int(motion_id.value)

    def set_product_grippers(
        self, *, left: float, right: float, gripper_level: int, mode: int
    ) -> None:
        self._call(
            self._runtime_abi.lib.articore_runtime_set_grippers,
            "set_grippers", float(left), float(right),
            int(gripper_level), int(mode),
        )

    @property
    def has_grippers(self) -> bool:
        value = ctypes.c_int32()
        self._call(
            self._runtime_abi.lib.articore_runtime_has_grippers,
            "has_grippers", ctypes.byref(value),
        )
        return bool(value.value)

    @property
    def state(self) -> ProductState:
        native = CProductState()
        native.struct_size = ctypes.sizeof(native)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_state,
            "get_state", ctypes.byref(native),
        )
        def arm(value) -> ProductArmState:
            enabled = tuple(
                bool(value.enabled_mask & (1 << index))
                if value.enabled_valid_mask & (1 << index) else None
                for index in range(7)
            )
            mos_temperatures = tuple(
                float(value.mos_temperatures[index])
                if value.temperature_valid_mask & (1 << index) else None
                for index in range(7)
            )
            rotor_temperatures = tuple(
                float(value.rotor_temperatures[index])
                if value.temperature_valid_mask & (1 << index) else None
                for index in range(7)
            )
            return ProductArmState(
                tuple(float(item) for item in value.positions),
                tuple(float(item) for item in value.velocities),
                tuple(float(item) for item in value.torques),
                enabled,
                mos_temperatures,
                rotor_temperatures,
            )
        def gripper(
            available, opening, level, enabled, enabled_valid,
            mos_temperature, rotor_temperature, temperature_valid,
        ) -> ProductGripperState | None:
            if not available:
                return None
            return ProductGripperState(
                True, float(opening), int(level),
                bool(enabled) if enabled_valid else None,
                float(mos_temperature) if temperature_valid else None,
                float(rotor_temperature) if temperature_valid else None,
            )
        return ProductState(
            bool(native.has_grippers),
            arm(native.left), arm(native.right),
            gripper(
                native.left_gripper_available,
                native.left_gripper_opening,
                native.left_gripper_level,
                native.left_gripper_enabled,
                native.left_gripper_enabled_valid,
                native.left_gripper_mos_temperature,
                native.left_gripper_rotor_temperature,
                native.left_gripper_temperature_valid,
            ),
            gripper(
                native.right_gripper_available,
                native.right_gripper_opening,
                native.right_gripper_level,
                native.right_gripper_enabled,
                native.right_gripper_enabled_valid,
                native.right_gripper_mos_temperature,
                native.right_gripper_rotor_temperature,
                native.right_gripper_temperature_valid,
            ),
            int(native.timestamp_ns), int(native.sequence),
        )

    def get_pose_sample(self, side: int) -> ProductPose:
        if side not in (0, 1):
            raise ValueError("side must be 0 (left) or 1 (right)")
        native = CProductPose()
        native.struct_size = ctypes.sizeof(native)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_pose,
            "get_pose", int(side), ctypes.byref(native),
        )
        values = tuple(float(value) for value in native.values)
        return ProductPose(
            side=int(native.side),
            values=values,  # type: ignore[arg-type]
            timestamp_ns=int(native.timestamp_ns),
            sequence=int(native.sequence),
        )

    def get_pose(self, side: int) -> list[float]:
        """Return the cached active-TCP pose as [x, y, z, roll, pitch, yaw]."""
        return list(self.get_pose_sample(side).values)

    def set_tcp_offset(self, side: int, offset: Sequence[float]) -> None:
        """Set the native flange-to-TCP transform for one arm."""
        if side not in (0, 1):
            raise ValueError("side must be 0 (left) or 1 (right)")
        values = _pose(offset)
        native = CTcpOffset()
        native.struct_size = ctypes.sizeof(native)
        native.side = int(side)
        native.values[:] = values[:]
        self._call(
            self._runtime_abi.lib.articore_runtime_set_tcp_offset,
            "set_tcp_offset", ctypes.byref(native),
        )

    def get_tcp_offset(self, side: int) -> list[float]:
        """Return the native flange-to-active-TCP transform."""
        if side not in (0, 1):
            raise ValueError("side must be 0 (left) or 1 (right)")
        native = CTcpOffset()
        native.struct_size = ctypes.sizeof(native)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_tcp_offset,
            "get_tcp_offset", int(side), ctypes.byref(native),
        )
        return [float(value) for value in native.values]

    def reset_tcp_offset(self, side: int) -> None:
        """Restore tool0 with grippers or link7 without grippers."""
        if side not in (0, 1):
            raise ValueError("side must be 0 (left) or 1 (right)")
        self._call(
            self._runtime_abi.lib.articore_runtime_reset_tcp_offset,
            "reset_tcp_offset", int(side),
        )

    def solve_ik(
        self,
        left_target_pose: Sequence[float],
        right_target_pose: Sequence[float],
    ) -> tuple[float, ...]:
        """Solve both TCP targets without installing or sending a motion command."""
        left_target = _pose(left_target_pose)
        right_target = _pose(right_target_pose)
        output = (ctypes.c_float * 14)()
        self._call(
            self._runtime_abi.lib.articore_runtime_solve_ik,
            "solve_ik",
            left_target,
            right_target,
            output,
            14,
        )
        return tuple(float(value) for value in output)

    def set_pose(
        self,
        left_target_pose: Sequence[float],
        right_target_pose: Sequence[float],
        speed_percent: float = 50.0,
    ) -> None:
        """Solve both endpoint poses and install one target in the current mode."""
        left_target = _pose(left_target_pose)
        right_target = _pose(right_target_pose)
        self._call(
            self._runtime_abi.lib.articore_runtime_set_pose,
            "set_pose",
            left_target,
            right_target,
            float(speed_percent),
        )

    def move_linear_trajectory(
        self,
        side: int,
        start_pose: Sequence[float],
        end_pose: Sequence[float],
        duration_s: float,
    ) -> int:
        """Submit Linear motion for native 100 Hz planning and 500 Hz execution."""
        start = _pose(start_pose)
        end = _pose(end_pose)
        motion_id = ctypes.c_uint64()
        self._call(
            self._runtime_abi.lib.articore_runtime_move_linear_trajectory,
            "move_linear_trajectory", int(side), start, end,
            float(duration_s), ctypes.byref(motion_id),
        )
        return int(motion_id.value)

    def move_linear_path_trajectory(
        self,
        side: int,
        poses: Sequence[Sequence[float]],
        segment_duration_s: float,
    ) -> int:
        """Submit one atomically planned Linear path with automatic 10 mm blends."""
        values = tuple(_pose(pose) for pose in poses)
        if not 2 <= len(values) <= 64:
            raise ValueError("linear path requires 2..64 poses")
        flattened_values = [
            float(value)
            for pose in values
            for value in pose
        ]
        flattened = (ctypes.c_float * len(flattened_values))(
            *flattened_values
        )
        motion_id = ctypes.c_uint64()
        self._call(
            self._runtime_abi.lib.articore_runtime_move_linear_path_trajectory,
            "move_linear_trajectory", int(side), flattened, len(values),
            float(segment_duration_s), ctypes.byref(motion_id),
        )
        return int(motion_id.value)

    def move_circular_trajectory(
        self,
        side: int,
        start_pose: Sequence[float],
        via_pose: Sequence[float],
        end_pose: Sequence[float],
        duration_s: float,
    ) -> int:
        """Submit one native joint-approach plus start/via/end Circular task."""
        start = _pose(start_pose)
        via = _pose(via_pose)
        end = _pose(end_pose)
        motion_id = ctypes.c_uint64()
        self._call(
            self._runtime_abi.lib.articore_runtime_move_circular_trajectory,
            "move_circular_trajectory", int(side), start, via, end,
            float(duration_s), ctypes.byref(motion_id),
        )
        return int(motion_id.value)

    def get_motion_status(self, motion_id: int) -> MotionStatus:
        native = CMotionStatus()
        native.struct_size = ctypes.sizeof(native)
        validated_id = _motion_id(motion_id)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_motion_status,
            "get_motion_status", validated_id, ctypes.byref(native),
        )
        states = {
            0: MotionState.IDLE,
            1: MotionState.RUNNING,
            2: MotionState.COMPLETED,
            3: MotionState.CANCELLED,
            4: MotionState.FAULT,
            5: MotionState.QUEUED,
        }
        motion_types = {
            1: MotionType.JOINT_TRAJECTORY,
            2: MotionType.CARTESIAN_LINEAR,
            3: MotionType.CARTESIAN_CIRCULAR,
        }
        state_value = int(native.state)
        try:
            state = states[state_value]
        except KeyError as exc:
            raise RuntimeCallError(
                f"get_motion_status returned unknown state {state_value}"
            ) from exc
        try:
            motion_type = motion_types[int(native.motion_type)]
        except KeyError as exc:
            raise RuntimeCallError(
                "get_motion_status returned unknown motion type "
                f"{int(native.motion_type)}"
            ) from exc
        return MotionStatus(
            state=state,
            motion_id=int(native.motion_id),
            motion_type=motion_type,
            active_segment=int(native.active_segment),
            waypoint_count=int(native.waypoint_count),
            elapsed_s=float(native.elapsed_s),
            duration_s=float(native.duration_s),
            progress=float(native.progress),
            error=_optional_text(native.error),
        )

    def cancel_motion(self, motion_id: int) -> None:
        self._call(
            self._runtime_abi.lib.articore_runtime_cancel_motion,
            "cancel_motion", _motion_id(motion_id),
        )

    def cancel_all_motions(self) -> None:
        self._call(
            self._runtime_abi.lib.articore_runtime_cancel_all_motions,
            "cancel_all_motions",
        )

    def clear_faults(self) -> None:
        """Clear recoverable motor faults without commanding motion."""
        self._call(
            self._runtime_abi.lib.articore_runtime_clear_faults,
            "clear_faults",
        )

    def set_zero(self) -> bool:
        """Calibrate the current installed-motor positions as zero."""
        self._call(
            self._runtime_abi.lib.articore_runtime_set_zero,
            "set_zero",
        )
        return True

    @property
    def health(self) -> SafetyHealth:
        native = CSafetyHealth()
        native.struct_size = ctypes.sizeof(native)
        self._call(self._runtime_abi.lib.articore_runtime_get_health,
                   "get_health", ctypes.byref(native))
        value = native
        motor_feedback_count = min(int(value.motor_feedback_count), 32)
        gripper_count = min(int(value.gripper_count), 2)
        return SafetyHealth(
            state=SafetyState(value.state), safe_holding=bool(value.safe_holding),
            disable_confirmed=bool(value.disable_confirmed),
            last_successful_command_age_ns=_optional_age(value.last_successful_command_age_ns),
            last_fresh_feedback_age_ns=_optional_age(value.last_fresh_feedback_age_ns),
            consecutive_send_failures=int(value.consecutive_send_failures),
            consecutive_feedback_failures=int(value.consecutive_feedback_failures),
            left_transport=_transport_health(value.left_transport),
            right_transport=_transport_health(value.right_transport),
            grippers=tuple(GripperHealth(
                available=bool(item.available), side=int(item.side),
                control_state=GripperControlState(item.control_state),
                opening=float(item.opening), motor_position=float(item.motor_position),
                torque=float(item.torque), contact_detected=bool(item.contact_detected),
                stalled=bool(item.stalled), overload=bool(item.overload),
                hold_target=float(item.hold_target) if item.has_hold_target else None,
                feedback_age_ns=_optional_age(item.feedback_age_ns),
                name=_text(item.name), fault_reason=_optional_text(item.fault_reason),
            ) for item in value.grippers[:gripper_count]),
            motor_faults=tuple(_text(value.motor_faults[i])
                               for i in range(min(int(value.motor_fault_count), 32))),
            unconfirmed_disable=tuple(_text(value.unconfirmed_disable[i])
                                      for i in range(min(int(value.unconfirmed_disable_count), 32))),
            fault_reason=_optional_text(value.fault_reason),
            last_operation=RuntimeOperation(value.last_operation),
            last_operation_code=OperationError(value.last_operation_code),
            operation_failed_motors=tuple(
                _text(value.operation_failed_motors[i])
                for i in range(min(int(value.operation_failed_motor_count), 32))
            ),
            last_operation_error=_optional_text(value.last_operation_error),
            degraded=bool(value.degraded),
            safe_stopped=bool(value.safe_stopped),
            requires_resynchronization=bool(value.requires_resynchronization),
            command_scale=float(value.command_scale),
            safety_reason=_optional_text(value.safety_reason),
            motor_feedback=tuple(
                MotorFeedbackHealth(
                    side=int(item.side),
                    can_id=int(item.can_id) if item.can_id_valid else None,
                    is_gripper=bool(item.is_gripper),
                    has_feedback=bool(item.has_feedback),
                    fresh=bool(item.fresh),
                    has_state=bool(item.has_state),
                    values_finite=bool(item.values_finite),
                    status_code=int(item.status_code),
                    issues=MotorFeedbackIssue(item.issues),
                    position=float(item.position),
                    velocity=float(item.velocity),
                    torque=float(item.torque),
                    feedback_age_ns=_optional_age(item.feedback_age_ns),
                    update_count=int(item.update_count),
                    role=_text(item.role),
                )
                for item in value.motor_feedback[:motor_feedback_count]
            ),
            feedback_issue_count=int(value.feedback_issue_count),
            feedback_issue_scope=FeedbackIssueScope(value.feedback_issue_scope),
        )

    def __enter__(self) -> ArticoreRuntime:
        self._require_open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.disconnect()

    def __del__(self) -> None:
        try:
            self.disconnect()
        except Exception:
            pass
