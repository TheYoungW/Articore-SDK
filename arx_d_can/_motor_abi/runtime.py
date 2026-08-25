from __future__ import annotations

import ctypes
from collections.abc import Sequence
from threading import Event, RLock, Thread
from types import TracebackType

from ._runtime_abi import (
    CCartesianMotionStatus,
    CConnectReport,
    CDisableReport,
    CEnableReport,
    CGravityCompensationConfig,
    CGravityCompensationStatus,
    CMotorPowerReport,
    CProductPose,
    CProductStateV2,
    CRuntimeTransportHealth,
    CSafetyHealthV2,
    get_runtime_abi,
)
from .errors import RuntimeCallError, RuntimeTransactionError
from .runtime_models import (
    CartesianInterpolation,
    CartesianMotionState,
    CartesianMotionStatus,
    ConnectChannelResult,
    ConnectErrorCode,
    ConnectMotorResult,
    ConnectReport,
    DisableMotorResult,
    DisableReport,
    EnableMotorResult,
    EnableReport,
    GripperControlState,
    GripperHealth,
    GravityCompensationPhase,
    GravityCompensationStatus,
    MotorPowerReport,
    MotorPowerResult,
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
        count = min(int(native.joint_count), len(native.joints))
        joints = tuple(
            f"{side}/{prefix}-joint{index}"
            for side, prefix in (("left", "l"), ("right", "r"))
            for index in range(1, 8)
        )[:count]
        return GravityCompensationStatus(
            phase=GravityCompensationPhase(native.phase),
            active=bool(native.active),
            transition_progress=float(native.transition_progress),
            control_cycles=int(native.control_cycles),
            joints=joints,
            gravity_feedforward_torque=tuple(
                float(native.gravity_feedforward_torque[index])
                for index in range(count)
            ),
        )

    def connect(self) -> ConnectReport:
        with self._lock:
            rc = int(self._runtime_abi.lib.articore_runtime_connect(
                self._require_open()
            ))
            failure = self._last_error() if rc != 0 else None
            report = self.last_connect_report()
            if rc != 0:
                raise RuntimeTransactionError(
                    f"connect failed: {failure}", report
                )
        self._start_fps_monitor()
        return report

    def last_connect_report(self) -> ConnectReport:
        native = CConnectReport()
        native.struct_size = ctypes.sizeof(CConnectReport)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_last_connect_report,
            "get_last_connect_report", ctypes.byref(native),
        )
        channels = tuple(
            ConnectChannelResult(
                side=int(item.side), active=bool(item.active),
                request_code=int(item.request_code),
                expected_count=int(item.expected_count),
                received_count=int(item.received_count),
                missing_motor_ids=tuple(
                    int(item.missing_motor_ids[index])
                    for index in range(min(int(item.missing_count), 32))
                ),
                error=_optional_text(item.error),
            )
            for item in native.channels
            if item.active
        )
        motors = tuple(
            ConnectMotorResult(
                side=int(item.side),
                configured_can_id=int(item.configured_can_id),
                reported_can_id=int(item.reported_can_id),
                has_feedback=bool(item.has_feedback),
                feedback_fresh=bool(item.feedback_fresh),
                feedback_valid=bool(item.feedback_valid),
                update_count=int(item.update_count),
                feedback_age_ns=_optional_age(item.feedback_age_ns),
                name=_text(item.name), error=_optional_text(item.error),
            )
            for item in native.motors[:min(int(native.motor_count), 32)]
        )
        return ConnectReport(
            success=bool(native.success),
            error_code=ConnectErrorCode(native.error_code),
            expected_count=int(native.expected_count),
            received_count=int(native.received_count),
            missing_count=int(native.missing_count),
            failure_count=int(native.failure_count),
            channels=channels, motors=motors,
            error=_optional_text(native.error),
        )

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
                self._require_open(), int(self._control_mode)
            ))
            if rc != 0:
                raise RuntimeTransactionError(
                    f"enable failed: {self._last_error()}",
                    self._last_enable_report(),
                )
        return True

    def _last_enable_report(self) -> EnableReport:
        native = CEnableReport()
        native.struct_size = ctypes.sizeof(CEnableReport)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_last_enable_report,
            "get_last_enable_report", ctypes.byref(native),
        )
        count = min(int(native.motor_count), 32)
        missing_count = min(int(native.missing_count), 32)
        return EnableReport(
            success=bool(native.success), disable_confirmed=bool(native.disable_confirmed),
            expected_count=int(native.expected_count), enabled_count=int(native.enabled_count),
            missing_count=int(native.missing_count), failure_count=int(native.failure_count),
            missing_motors=tuple(
                (int(native.missing_motor_sides[i]), int(native.missing_motor_ids[i]))
                for i in range(missing_count)
            ),
            motors=tuple(EnableMotorResult(
                side=int(item.side), can_id=int(item.can_id), status_code=int(item.status_code),
                has_feedback=bool(item.has_feedback), feedback_fresh=bool(item.feedback_fresh),
                enabled=bool(item.enabled), name=_text(item.name),
            ) for item in native.motors[:count]),
            error=_optional_text(native.error),
        )

    def disable(self, motors: Sequence[str] | None = None) -> bool:
        if motors is not None:
            return self._set_motor_power(motors, enabled=False)
        with self._lock:
            rc = int(self._runtime_abi.lib.articore_runtime_disable(self._require_open()))
            if rc != 0:
                raise RuntimeTransactionError(
                    f"disable failed: {self._last_error()}",
                    self._last_disable_report(),
                )
        return True

    def _last_disable_report(self) -> DisableReport:
        native = CDisableReport()
        native.struct_size = ctypes.sizeof(CDisableReport)
        self._call(self._runtime_abi.lib.articore_runtime_get_last_disable_report,
                   "get_last_disable_report", ctypes.byref(native))
        count = min(int(native.motor_count), 32)
        missing_count = min(int(native.missing_count), 32)
        return DisableReport(
            success=bool(native.success), barrier_confirmed=bool(native.barrier_confirmed),
            expected_count=int(native.expected_count), disabled_count=int(native.disabled_count),
            missing_count=int(native.missing_count), failure_count=int(native.failure_count),
            retry_count=int(native.retry_count),
            missing_motors=tuple(
                (int(native.missing_motor_sides[i]), int(native.missing_motor_ids[i]))
                for i in range(missing_count)
            ),
            motors=tuple(DisableMotorResult(
                side=int(item.side), can_id=int(item.can_id), status_code=int(item.status_code),
                has_feedback=bool(item.has_feedback), feedback_fresh=bool(item.feedback_fresh),
                disabled=bool(item.disabled), disable_sent=bool(item.disable_sent),
                retry_sent=bool(item.retry_sent), name=_text(item.name),
            ) for item in native.motors[:count]),
            error=_optional_text(native.error),
        )

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

    def set_max_speed(self, max_speed_percent: float) -> None:
        """Set persistent PV reference speed; 0..100 maps to 0..3 rad/s."""
        self._call(
            self._runtime_abi.lib.articore_runtime_set_max_speed,
            "set_max_speed", float(max_speed_percent),
        )

    def get_max_speed(self) -> float:
        """Return the persistent 0..100 ordinary-PV reference percentage."""
        value = ctypes.c_float()
        self._call(
            self._runtime_abi.lib.articore_runtime_get_max_speed,
            "get_max_speed", ctypes.byref(value),
        )
        return float(value.value)

    def set_joint_positions(
        self,
        positions: Sequence[float],
        speed_percent: float | None = None,
    ) -> None:
        values = tuple(float(value) for value in positions)
        native = (ctypes.c_float * len(values))(*values)
        if self.control_mode is RuntimeControlMode.PV:
            if speed_percent is not None:
                raise ValueError(
                    "PV position commands use the persistent max speed"
                )
            self._call(
                self._runtime_abi.lib.articore_runtime_set_joint_positions_v2,
                "set_joint_positions_v2", native, len(values),
            )
            return
        self._call(
            self._runtime_abi.lib.articore_runtime_set_joint_positions,
            "set_joint_positions", native, len(values),
            100.0 if speed_percent is None else float(speed_percent),
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

    def set_product_grippers(
        self, *, left: float, right: float, gripper_level: int, mode: int
    ) -> None:
        self._call(
            self._runtime_abi.lib.articore_runtime_set_grippers_v2,
            "set_product_grippers_v2", float(left), float(right),
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
        native = CProductStateV2()
        native.struct_size = ctypes.sizeof(native)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_state_v2,
            "get_state_v2", ctypes.byref(native),
        )
        def arm(value) -> ProductArmState:
            enabled = tuple(
                bool(value.enabled_mask & (1 << index))
                if value.enabled_valid_mask & (1 << index) else None
                for index in range(7)
            )
            return ProductArmState(
                tuple(float(item) for item in value.positions),
                tuple(float(item) for item in value.velocities),
                tuple(float(item) for item in value.torques),
                enabled,
            )
        def gripper(
            available, opening, level, enabled, enabled_valid,
        ) -> ProductGripperState | None:
            if not available:
                return None
            return ProductGripperState(
                True, float(opening), int(level),
                bool(enabled) if enabled_valid else None,
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
            ),
            gripper(
                native.right_gripper_available,
                native.right_gripper_opening,
                native.right_gripper_level,
                native.right_gripper_enabled,
                native.right_gripper_enabled_valid,
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
        """Return the cached product-control pose as [x, y, z, roll, pitch, yaw]."""
        return list(self.get_pose_sample(side).values)

    def move_pose(
        self, side: int, target_pose: Sequence[float], speed_percent: float
    ) -> int:
        """Submit one native PTP IK target for sampled PV execution."""
        target = _pose(target_pose)
        motion_id = ctypes.c_uint64()
        self._call(
            self._runtime_abi.lib.articore_runtime_move_pose,
            "move_pose", int(side), target, float(speed_percent),
            ctypes.byref(motion_id),
        )
        return int(motion_id.value)

    def move_linear(
        self,
        side: int,
        start_pose: Sequence[float],
        end_pose: Sequence[float],
        speed_percent: float,
    ) -> int:
        """Submit an explicit start-to-end Cartesian line."""
        start = _pose(start_pose)
        end = _pose(end_pose)
        motion_id = ctypes.c_uint64()
        self._call(
            self._runtime_abi.lib.articore_runtime_move_linear_v2,
            "move_linear_v2", int(side), start, end,
            float(speed_percent), ctypes.byref(motion_id),
        )
        return int(motion_id.value)

    def move_circular(
        self,
        side: int,
        start_pose: Sequence[float],
        via_pose: Sequence[float],
        end_pose: Sequence[float],
        speed_percent: float,
    ) -> int:
        """Submit an explicit start/via/end circular and SLERP path."""
        start = _pose(start_pose)
        via = _pose(via_pose)
        end = _pose(end_pose)
        motion_id = ctypes.c_uint64()
        self._call(
            self._runtime_abi.lib.articore_runtime_move_circular,
            "move_circular", int(side), start, via, end,
            float(speed_percent), ctypes.byref(motion_id),
        )
        return int(motion_id.value)

    @property
    def cartesian_motion_status(self) -> CartesianMotionStatus:
        native = CCartesianMotionStatus()
        native.struct_size = ctypes.sizeof(native)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_cartesian_motion_status,
            "get_cartesian_motion_status", ctypes.byref(native),
        )
        states = {
            0: CartesianMotionState.IDLE,
            1: CartesianMotionState.RUNNING,
            2: CartesianMotionState.COMPLETED,
            3: CartesianMotionState.CANCELLED,
            4: CartesianMotionState.FAULT,
        }
        interpolations = {
            1: CartesianInterpolation.POINT_TO_POINT,
            2: CartesianInterpolation.LINEAR,
            3: CartesianInterpolation.CIRCULAR,
        }
        state_value = int(native.state)
        try:
            state = states[state_value]
        except KeyError as exc:
            raise RuntimeCallError(
                f"get_cartesian_motion_status returned unknown state {state_value}"
            ) from exc
        try:
            interpolation = interpolations[int(native.interpolation)]
        except KeyError as exc:
            raise RuntimeCallError(
                "get_cartesian_motion_status returned unknown interpolation "
                f"{int(native.interpolation)}"
            ) from exc
        values = tuple(float(value) for value in native.target_pose)
        return CartesianMotionStatus(
            state=state,
            motion_id=int(native.motion_id),
            superseded_motion_id=int(native.superseded_motion_id),
            side="left" if int(native.side) == 0 else "right",
            interpolation=interpolation,
            speed_percent=float(native.speed_percent),
            elapsed_s=float(native.elapsed_s),
            duration_s=float(native.duration_s),
            progress=float(native.progress),
            target_pose=values,  # type: ignore[arg-type]
            error=_optional_text(native.error),
        )

    def cancel_cartesian_motion(self) -> None:
        self._call(
            self._runtime_abi.lib.articore_runtime_cancel_cartesian_motion,
            "cancel_cartesian_motion",
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
        native = CSafetyHealthV2()
        native.struct_size = ctypes.sizeof(native)
        self._call(self._runtime_abi.lib.articore_runtime_get_health_v2,
                   "get_health", ctypes.byref(native))
        value = native.health
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
            last_operation=RuntimeOperation(native.last_operation),
            last_operation_code=OperationError(native.last_operation_code),
            operation_failed_motors=tuple(
                _text(native.operation_failed_motors[i])
                for i in range(min(int(native.operation_failed_motor_count), 32))
            ),
            last_operation_error=_optional_text(native.last_operation_error),
            degraded=bool(native.degraded),
            safe_stopped=bool(native.safe_stopped),
            requires_resynchronization=bool(native.requires_resynchronization),
            command_scale=float(native.command_scale),
            safety_reason=_optional_text(native.safety_reason),
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
