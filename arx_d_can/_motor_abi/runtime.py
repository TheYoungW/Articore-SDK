from __future__ import annotations

import ctypes
from collections.abc import Sequence
from threading import Event, RLock, Thread
from types import TracebackType

from ._runtime_abi import (
    CConnectReport,
    CDisableReport,
    CEnableReport,
    CGripperCommand,
    CGripperProductBinding,
    CGravityCompensationConfig,
    CGravityCompensationStatus,
    CGravityProductBinding,
    CJointControlConfig,
    CJointSafetyLimits,
    CJointTarget,
    CMitTorqueLimitStats,
    CMotorIdentity,
    CMitCommand,
    CPosVelCommand,
    CProductPose,
    CProductState,
    CRuntimeConfig,
    CRuntimeMotorDescriptor,
    CRuntimeTransportCapabilities,
    CRuntimeTransportHealth,
    CSafetyHealth,
    CSafetyHealthV2,
    get_runtime_abi,
)
from .core import Controller, ControllerGroup, Motor
from .errors import RuntimeCallError, RuntimeTransactionError
from .models import PresenceState
from .runtime_models import (
    ActiveCapability,
    CommandLifetime,
    ConnectChannelResult,
    ConnectErrorCode,
    ConnectMotorResult,
    ConnectReport,
    DisableMotorResult,
    DisableReport,
    EnableMotorResult,
    EnableReport,
    GripperCommand,
    GripperControlState,
    GripperHealth,
    GripperProductBinding,
    GravityCompensationPhase,
    GravityCompensationStatus,
    GravityProductBinding,
    JointControlConfig,
    JointPositionTarget,
    JointSafetyLimits,
    MitTorqueLimitJointStats,
    MitTorqueLimitStats,
    MotorPowerState,
    RuntimeConfig,
    RuntimeControlMode,
    RuntimeOperation,
    OperationError,
    ProductArmState,
    ProductGripperState,
    ProductPose,
    ProductState,
    RuntimeMitCommand,
    RuntimeMotor,
    RuntimePvCommand,
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


def _fixed_text(value: str, field: str) -> bytes:
    if not isinstance(value, str):
        raise TypeError(f"{field} must be a string")
    encoded = value.encode()
    if not encoded or b"\0" in encoded or len(encoded) >= 64:
        raise ValueError(f"{field} must contain 1..63 UTF-8 bytes and no NUL")
    return encoded


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


class ArticoreRuntime:
    """Official owner-safe Python binding for the native Articore Runtime.

    All control, watchdog, safety, gripper, and fixed-rate behavior remains in
    ``libarticore_runtime``. This class only validates Python object ownership,
    converts typed values, and translates stable native reports.
    """

    def __init__(
        self,
        config: RuntimeConfig,
        controller_group: ControllerGroup,
        left_controller: Controller | None,
        right_controller: Controller | None,
        motors: Sequence[RuntimeMotor],
    ) -> None:
        values = tuple(motors)
        if not values:
            raise ValueError("motors must not be empty")
        if len({id(item.motor) for item in values}) != len(values):
            raise ValueError("runtime motors must not contain duplicates")
        if left_controller is None and right_controller is None:
            raise ValueError("at least one controller is required")
        controllers = tuple(
            controller
            for controller in (left_controller, right_controller)
            if controller is not None
        )
        if len({id(controller) for controller in controllers}) != len(controllers):
            raise ValueError("left and right controllers must be distinct")
        if any(controller not in controller_group._controllers for controller in controllers):
            raise ValueError("runtime controllers must belong to controller_group")
        for item in values:
            if item.side not in (0, 1):
                raise ValueError(f"{item.name}: side must be 0 or 1")
            expected = left_controller if item.side == 0 else right_controller
            if expected is None or item.motor._controller is not expected:
                raise ValueError(f"{item.name}: motor does not belong to its side controller")
            controller_group._validate_motor(item.motor)

        self._lock = RLock()
        self._fps = 0.0
        self._fps_stop = Event()
        self._fps_thread: Thread | None = None
        self._ptr: int | None = None
        self._runtime_abi = get_runtime_abi()
        self._group = controller_group
        self._controllers = controllers
        self._motors = values
        self._motor_set = {id(item.motor): item.motor for item in values}
        self._leases: list[object] = []
        self._product_owned = False
        self._control_mode: RuntimeControlMode | None = None

        native_config = CRuntimeConfig(
            int(config.control_hz), int(config.command_timeout_ms),
            int(config.enable_grace_ms), int(config.safe_hold_hz),
            int(config.feedback_check_hz), int(config.feedback_failure_threshold),
            int(config.feedback_max_age_ms), int(config.safe_hold_failure_threshold),
            int(config.disable_feedback_timeout_ms), float(config.safe_pv_velocity_limit),
            int(config.gripper_control_hz), int(config.gripper_fault_action),
        )
        native_motors = (CRuntimeMotorDescriptor * len(values))()
        for index, item in enumerate(values):
            descriptor = native_motors[index]
            descriptor.motor = item.motor._require_open()
            descriptor.side = item.side
            descriptor.is_gripper = int(item.is_gripper)
            descriptor.name = _fixed_text(item.name, "motor name")
            descriptor.safe_kp = float(item.safe_kp)
            descriptor.safe_kd = float(item.safe_kd)
            # All gripper product fields intentionally remain zero. The SDK
            # binds a named native product profile before connect().

        resources = (controller_group, *controllers, *(item.motor for item in values))
        try:
            for resource in resources:
                resource._acquire_runtime_lease()
                self._leases.append(resource)
            self._motor_api = self._runtime_abi.motor_api()
            self._maintenance_api = self._runtime_abi.maintenance_api()
            motor_lib = self._runtime_abi.motor.lib
            create_arguments = (
                ctypes.byref(native_config), ctypes.byref(self._motor_api),
                controller_group._require_open(),
                left_controller._require_open() if left_controller else None,
                right_controller._require_open() if right_controller else None,
                native_motors, len(values),
                ctypes.cast(motor_lib.motor_controller_enable_all, ctypes.c_void_p),
                ctypes.cast(motor_lib.motor_handle_enable, ctypes.c_void_p),
            )
            if self._runtime_abi.has_transport_aware_create:
                native_transports = (
                    CRuntimeTransportCapabilities * len(controllers)
                )()
                for output, controller in zip(native_transports, controllers):
                    capabilities = controller.transport_capabilities()
                    encoded_transport = capabilities.transport.encode()
                    if not encoded_transport or len(encoded_transport) >= 32:
                        raise ValueError(
                            "transport capability name must contain 1..31 bytes"
                        )
                    output.struct_size = ctypes.sizeof(
                        CRuntimeTransportCapabilities
                    )
                    output.side = 0 if controller is left_controller else 1
                    output.can_fd = int(capabilities.can_fd)
                    output.can_fd_brs = int(capabilities.can_fd_brs)
                    output.transport = encoded_transport
                if self._runtime_abi.has_owner_safe_maintenance:
                    pointer = self._runtime_abi.lib.articore_runtime_create_ex3(
                        ctypes.byref(native_config), ctypes.byref(self._motor_api),
                        ctypes.byref(self._maintenance_api),
                        controller_group._require_open(),
                        left_controller._require_open() if left_controller else None,
                        right_controller._require_open() if right_controller else None,
                        native_motors, len(values),
                        ctypes.cast(motor_lib.motor_controller_enable_all, ctypes.c_void_p),
                        ctypes.cast(motor_lib.motor_handle_enable, ctypes.c_void_p),
                        native_transports, len(native_transports),
                    )
                else:
                    pointer = self._runtime_abi.lib.articore_runtime_create_ex2(
                        *create_arguments, native_transports, len(native_transports)
                    )
            else:
                pointer = self._runtime_abi.lib.articore_runtime_create_ex(
                    *create_arguments
                )
            if not pointer:
                raise RuntimeCallError(
                    f"articore_runtime_create_ex failed: {self._last_error()}"
                )
            self._ptr = int(pointer)
            motor_ids = tuple(getattr(item.motor, "_motor_id", None) for item in values)
            if any(value is not None for value in motor_ids):
                if not all(value is not None for value in motor_ids):
                    raise ValueError(
                        "Runtime motor CAN identities are only partially available"
                    )
                identities = (CMotorIdentity * len(values))()
                for identity, item, can_id in zip(identities, values, motor_ids):
                    identity.struct_size = ctypes.sizeof(CMotorIdentity)
                    identity.motor = item.motor._require_open()
                    identity.can_id = int(can_id)
                self._call(
                    self._runtime_abi.lib.articore_runtime_configure_motor_identities,
                    "configure_motor_identities", identities, len(values),
                )
        except Exception:
            if self._ptr:
                self._runtime_abi.lib.articore_runtime_free(self._ptr)
                self._ptr = None
            self._release_leases()
            raise

    @classmethod
    def create_product(
        cls,
        product_id: str,
        control_mode: RuntimeControlMode,
        with_grippers: bool = True,
    ) -> ArticoreRuntime:
        """创建完全由 C++ 拥有资源和产品配置的整机 Runtime。"""
        mode = RuntimeControlMode(control_mode)
        runtime_abi = get_runtime_abi()
        pointer = runtime_abi.lib.articore_runtime_create_product(
            _fixed_text(product_id, "product_id"), int(mode),
            int(bool(with_grippers)),
        )
        if not pointer:
            detail = runtime_abi.lib.articore_runtime_last_error()
            text = detail.decode(errors="replace") if detail else "unknown error"
            raise RuntimeCallError(f"create_product failed: {text}")
        self = cls.__new__(cls)
        self._lock = RLock()
        self._fps = 0.0
        self._fps_stop = Event()
        self._fps_thread = None
        self._ptr = pointer
        self._runtime_abi = runtime_abi
        self._group = None
        self._controllers = ()
        self._motors = ()
        self._motor_set = {}
        self._leases = []
        self._product_owned = True
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

    def _motor_pointer(self, motor: Motor) -> int:
        owned = self._motor_set.get(id(motor))
        if owned is not motor:
            raise ValueError("motor does not belong to this ArticoreRuntime")
        return motor._require_open()

    def _release_leases(self) -> None:
        while self._leases:
            self._leases.pop()._release_runtime_lease()

    @property
    def control_hz(self) -> int:
        value = ctypes.c_uint32()
        self._call(
            self._runtime_abi.lib.articore_runtime_get_control_hz,
            "get_control_hz", ctypes.byref(value),
        )
        return int(value.value)

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

    @property
    def mit_torque_limit_stats(self) -> MitTorqueLimitStats:
        native = CMitTorqueLimitStats()
        native.struct_size = ctypes.sizeof(native)
        self._call(
            self._runtime_abi.lib.articore_runtime_get_mit_torque_limit_stats,
            "get_mit_torque_limit_stats", ctypes.byref(native),
        )
        motors_by_pointer = {
            self._motor_pointer(item.motor): item.motor for item in self._motors
        }
        product_names = tuple(
            f"{side}/{prefix}-joint{index}"
            for side, prefix in (("left", "l"), ("right", "r"))
            for index in range(1, 8)
        )
        count = min(int(native.joint_count), len(native.joints))
        joints = []
        for index in range(count):
            pointer = int(native.joints[index] or 0)
            motor = motors_by_pointer.get(pointer)
            if motor is None and self._product_owned and index < len(product_names):
                motor = product_names[index]
            if motor is None:
                raise RuntimeCallError(
                    "MIT torque limit stats returned an unknown motor handle"
                )
            joints.append(MitTorqueLimitJointStats(
                motor=motor,
                requested_resultant_torque=float(
                    native.requested_resultant_torque[index]
                ),
                applied_scale=float(native.applied_scale[index]),
                applied_resultant_torque=float(
                    native.applied_resultant_torque[index]
                ),
                limited=bool(native.torque_limited_joint_mask & (1 << index)),
            ))
        return MitTorqueLimitStats(
            torque_limit_activation_count=int(
                native.torque_limit_activation_count
            ),
            torque_limited_joint_mask=int(native.torque_limited_joint_mask),
            joints=tuple(joints),
        )

    def configure_joints(self, configs: Sequence[JointControlConfig]) -> None:
        values = tuple(configs)
        native = (CJointControlConfig * len(values))(
            *(CJointControlConfig(
                self._motor_pointer(item.motor), item.lower_position,
                item.upper_position, item.velocity_limit, item.torque_limit,
                item.mit_kp, item.mit_kd, item.mit_feedforward_torque,
            ) for item in values)
        )
        self._call(self._runtime_abi.lib.articore_runtime_configure_joints,
                   "configure_joints", native if values else None, len(values))

    def configure_joint_safety_limits(
        self, limits: Sequence[JointSafetyLimits]
    ) -> None:
        values = tuple(limits)
        native = (CJointSafetyLimits * len(values))()
        for output, item in zip(native, values):
            output.struct_size = ctypes.sizeof(CJointSafetyLimits)
            output.motor = self._motor_pointer(item.motor)
            output.hard_lower_position = item.hard_lower_position
            output.hard_upper_position = item.hard_upper_position
            output.soft_lower_position = item.soft_lower_position
            output.soft_upper_position = item.soft_upper_position
            output.soft_limit_braking_zone = item.soft_limit_braking_zone
            output.braking_acceleration = item.braking_acceleration
        self._call(
            self._runtime_abi.lib.articore_runtime_configure_joint_safety_limits,
            "configure_joint_safety_limits", native if values else None, len(values),
        )

    def configure_gripper_products(
        self, bindings: Sequence[GripperProductBinding]
    ) -> None:
        values = tuple(bindings)
        native = (CGripperProductBinding * len(values))()
        for output, item in zip(native, values):
            output.struct_size = ctypes.sizeof(CGripperProductBinding)
            output.motor = self._motor_pointer(item.motor)
            output.profile_id = _fixed_text(item.profile_id, "profile_id")
        self._call(
            self._runtime_abi.lib.articore_runtime_configure_gripper_products,
            "configure_gripper_products", native if values else None, len(values),
        )

    def configure_gravity_products(
        self, bindings: Sequence[GravityProductBinding]
    ) -> None:
        values = tuple(bindings)
        native = (CGravityProductBinding * len(values))()
        for output, item in zip(native, values):
            output.struct_size = ctypes.sizeof(CGravityProductBinding)
            output.runtime_side = item.runtime_side
            output.robot_side = item.robot_side
            output.product_id = _fixed_text(item.product_id, "product_id")
        self._call(
            self._runtime_abi.lib.articore_runtime_configure_gravity_products,
            "configure_gravity_products", native if values else None, len(values),
        )

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
        if self._product_owned:
            joints = tuple(
                f"{side}/{prefix}-joint{index}"
                for side, prefix in (("left", "l"), ("right", "r"))
                for index in range(1, 8)
            )[:count]
        else:
            motors_by_pointer = {
                self._motor_pointer(item.motor): item.motor for item in self._motors
            }
            joints = tuple(
                motors_by_pointer[int(native.joints[index] or 0)]
                for index in range(count)
            )
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

    def declare_motor_presence(self, role: str, state: PresenceState) -> None:
        encoded = _fixed_text(role, "motor role")
        self._call(
            self._runtime_abi.lib.articore_runtime_declare_motor_presence,
            "declare_motor_presence", encoded, int(state),
        )

    def motor_presence(self, role: str) -> PresenceState:
        value = ctypes.c_int32()
        self._call(
            self._runtime_abi.lib.articore_runtime_motor_presence,
            "motor_presence", _fixed_text(role, "motor role"), ctypes.byref(value),
        )
        return PresenceState(value.value)

    @property
    def active_capabilities(self) -> ActiveCapability:
        with self._lock:
            value = int(
                self._runtime_abi.lib.articore_runtime_active_capabilities(
                    self._require_open()
                )
            )
            return ActiveCapability(value)

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

    def enable(
        self,
        mode: RuntimeControlMode | None = None,
        *,
        motor: str | None = None,
    ) -> bool:
        if motor is not None:
            return self.set_motor_enabled(motor, True)
        try:
            selected = self._control_mode if mode is None else RuntimeControlMode(mode)
        except ValueError as error:
            raise RuntimeTransactionError(
                "enable failed: control mode must be PV or MIT",
                self._last_enable_report(),
            ) from error
        if selected is None:
            raise ValueError("control mode is required for a generic Runtime")
        with self._lock:
            rc = int(self._runtime_abi.lib.articore_runtime_enable(
                self._require_open(), int(selected)
            ))
            if rc != 0:
                raise RuntimeTransactionError(
                    f"enable failed: {self._last_error()}",
                    self._last_enable_report(),
                )
        return True

    def set_motor_enabled(self, motor: str | None, enabled: bool) -> bool:
        """Set one motor, or all motors when ``motor`` is ``None``.

        Stable product selectors are ``left/joint1`` through
        ``left/joint7``, ``right/joint1`` through ``right/joint7``, and the
        optional ``left/gripper`` / ``right/gripper`` roles.
        """
        if not isinstance(enabled, bool):
            raise TypeError("enabled must be bool")
        encoded = None if motor is None else _fixed_text(motor, "motor")
        confirmed = ctypes.c_int32()
        self._call(
            self._runtime_abi.lib.articore_runtime_set_motor_power,
            "set_motor_power", encoded, int(enabled), ctypes.byref(confirmed),
        )
        expected = (
            MotorPowerState.ENABLED if enabled else MotorPowerState.DISABLED
        )
        return MotorPowerState(confirmed.value) is expected

    def motor_power_state(self, motor: str | None = None) -> MotorPowerState:
        encoded = None if motor is None else _fixed_text(motor, "motor")
        state = ctypes.c_int32()
        self._call(
            self._runtime_abi.lib.articore_runtime_get_motor_power,
            "get_motor_power", encoded, ctypes.byref(state),
        )
        return MotorPowerState(state.value)

    def is_enabled(self, motor: str | None = None) -> bool:
        return self.motor_power_state(motor) is MotorPowerState.ENABLED

    def is_disabled(self, motor: str | None = None) -> bool:
        return self.motor_power_state(motor) is MotorPowerState.DISABLED

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

    def set_joint_mit(
        self, targets: Sequence[JointPositionTarget], speed: float
    ) -> None:
        self._set_joint_targets("mit", targets, speed)

    def set_joint_pv(
        self, targets: Sequence[JointPositionTarget], speed: float
    ) -> None:
        self._set_joint_targets("pv", targets, speed)

    def _set_joint_targets(
        self, mode: str, targets: Sequence[JointPositionTarget], speed: float
    ) -> None:
        values = tuple(targets)
        native = (CJointTarget * len(values))()
        for output, target in zip(native, values):
            output.struct_size = ctypes.sizeof(CJointTarget)
            output.motor = self._motor_pointer(target.motor)
            output.target_position = float(target.position)
        self._call(
            getattr(self._runtime_abi.lib, f"articore_runtime_set_joint_{mode}"),
            f"set_joint_{mode}", native if values else None, len(values), float(speed),
        )

    def submit_mit(
        self, commands: Sequence[RuntimeMitCommand],
        lifetime: CommandLifetime = CommandLifetime.STREAMING,
    ) -> None:
        values = tuple(commands)
        native = (CMitCommand * len(values))(
            *(CMitCommand(self._motor_pointer(item.motor), item.position,
                          item.velocity, item.kp, item.kd,
                          item.feedforward_torque) for item in values)
        )
        self._call(self._runtime_abi.lib.articore_runtime_submit_mit_ex,
                   "submit_mit", native if values else None, len(values), int(lifetime))

    def submit_pv(
        self, commands: Sequence[RuntimePvCommand],
        lifetime: CommandLifetime = CommandLifetime.STREAMING,
    ) -> None:
        values = tuple(commands)
        native = (CPosVelCommand * len(values))(
            *(CPosVelCommand(self._motor_pointer(item.motor), item.position,
                             item.velocity_limit) for item in values)
        )
        self._call(self._runtime_abi.lib.articore_runtime_submit_pos_vel_ex,
                   "submit_pv", native if values else None, len(values), int(lifetime))

    def set_grippers(self, commands: Sequence[GripperCommand]) -> None:
        values = tuple(commands)
        native = (CGripperCommand * len(values))()
        for output, item in zip(native, values):
            output.struct_size = ctypes.sizeof(CGripperCommand)
            output.motor = self._motor_pointer(item.motor)
            output.opening = item.opening
            output.speed = item.speed
            output.force_level = item.force_level
        self._call(self._runtime_abi.lib.articore_runtime_set_gripper_commands,
                   "set_grippers", native if values else None, len(values))

    def report_feedback_failure(self, side: int, reason: str) -> None:
        if side not in (0, 1):
            raise ValueError("side must be 0 or 1")
        self._call(self._runtime_abi.lib.articore_runtime_report_feedback_failure,
                   "report_feedback_failure", side, reason.encode())

    def disable(self, *, motor: str | None = None) -> bool:
        if motor is not None:
            return self.set_motor_enabled(motor, False)
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

    def estop(self, reason: str) -> None:
        self._call(self._runtime_abi.lib.articore_runtime_estop,
                   "estop", reason.encode())

    def recover(self) -> None:
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
        try:
            self._call(
                self._runtime_abi.lib.articore_runtime_disconnect,
                "disconnect",
            )
        except Exception:
            self._start_fps_monitor()
            raise

    def set_joint_positions(
        self, positions: Sequence[float], speed: float = 100.0
    ) -> None:
        values = tuple(float(value) for value in positions)
        native = (ctypes.c_float * len(values))(*values)
        self._call(
            self._runtime_abi.lib.articore_runtime_set_joint_positions,
            "set_joint_positions", native, len(values),
            float(speed),
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

    def submit_pv_frame(
        self, positions: Sequence[float], velocity_limits: Sequence[float]
    ) -> None:
        q = tuple(float(value) for value in positions)
        v = tuple(float(value) for value in velocity_limits)
        native_q = (ctypes.c_float * len(q))(*q)
        native_v = (ctypes.c_float * len(v))(*v)
        self._call(
            self._runtime_abi.lib.articore_runtime_submit_pv_frame,
            "submit_pv_frame", native_q, native_v, len(q),
        )

    def set_product_grippers(
        self, *, left: float, right: float, gripper_level: int = 3
    ) -> None:
        self._call(
            self._runtime_abi.lib.articore_runtime_set_grippers,
            "set_product_grippers", float(left), float(right), int(gripper_level),
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
            return ProductArmState(
                tuple(float(item) for item in value.positions),
                tuple(float(item) for item in value.velocities),
                tuple(float(item) for item in value.torques),
            )
        def gripper(available, opening, level) -> ProductGripperState | None:
            if not available:
                return None
            return ProductGripperState(
                True, float(opening), int(level),
            )
        return ProductState(
            bool(native.has_grippers),
            arm(native.left), arm(native.right),
            gripper(
                native.left_gripper_available,
                native.left_gripper_opening,
                native.left_gripper_level,
            ),
            gripper(
                native.right_gripper_available,
                native.right_gripper_opening,
                native.right_gripper_level,
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
        """Return cached flange pose as [x, y, z, roll, pitch, yaw]."""
        return list(self.get_pose_sample(side).values)

    def clear_faults(self) -> None:
        self._call(
            self._runtime_abi.lib.articore_runtime_clear_faults,
            "clear_faults",
        )

    def set_zero(self) -> bool:
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

    def close(self) -> None:
        self._stop_fps_monitor()
        try:
            with self._lock:
                if not self._ptr:
                    return
                rc = int(self._runtime_abi.lib.articore_runtime_close(self._ptr))
                failure = self._last_error() if rc != 0 else None
                if rc != 0:
                    try:
                        report = self._last_disable_report()
                    except Exception:
                        report = DisableReport(False, False, 0, 0, 0, 1, 0, (), (), self._last_error())
                    # A failed native close means physical disable was not
                    # confirmed. Keep the native Runtime and every acquired lease
                    # alive so callers can inspect the report and retry close;
                    # releasing the group/controllers here would violate the
                    # deterministic-close ownership barrier.
                    raise RuntimeTransactionError(
                        f"close failed: {failure}", report
                    )
                self._runtime_abi.lib.articore_runtime_free(self._ptr)
                self._ptr = None
                self._release_leases()
        except Exception:
            if self._ptr:
                self._start_fps_monitor()
            raise

    def __enter__(self) -> ArticoreRuntime:
        self._require_open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
