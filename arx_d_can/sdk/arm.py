"""ARX-D-CAN 机械臂高层控制器。"""
from __future__ import annotations

import math
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Sequence

import numpy as np

from ..actuator import ArxDCan
from ..driver import (
    ControllerGroup,
    build_scan_command,
    damiao_model_limits,
    parse_scan_ids,
)
from ..errors import UnexpectedMotorStateError
from .config import (
    ArxDCanConfig,
    _actuator_config_from_sdk,
    _config_from_loaded,
    _connection_channel,
    _mit_gain_vector,
)
from .joint_commands import _JointCommandMixin
from .native_safety import (
    CommandLifetime,
    DisableReport,
    EnableReport,
    GripperSafetyHealth,
    NativeJointControlConfig,
    NativeMotorDescriptor,
    NativeSafetyRuntime,
    SafetyHealth,
    SafetyState,
    TrajectoryStatus,
    TransportHealth,
)
from .safety import _SafetyMixin
from .state import (
    ArxDCanState,
    GripperState,
    JointState,
)


_DEFAULT_SPEED_LEVEL = 200.0
_MAX_SPEED_LEVEL = 400.0


def _load_profile(config_path: str | Path | None, *, model: str | None) -> dict:
    """从包级入口解析配置加载器，保持现有 monkeypatch 钩子有效。"""
    sdk_package = sys.modules[__package__]
    return sdk_package.load_cfg(config_path, model=model)


@dataclass(slots=True, frozen=True)
class _PreparedJointPositionBatch:
    """一侧机械臂已完成验证和坐标换算的批量命令。"""

    mode: str
    commands: tuple[object, ...]


class ArxDCanArm(_SafetyMixin, _JointCommandMixin):
    """通过 DM Device、dm-serial 或 Linux SocketCAN 控制机械臂。"""

    def __init__(
        self,
        *,
        port: str | None = None,
        channel: str | None = None,
        baud: int | None = None,
        transport: str | None = None,
        model: str | None = None,
        config_path: str | Path | None = None,
        config: ArxDCanConfig | None = None,
        control_mode: str = "posvel",
        enable_gripper: bool | None = None,
    ) -> None:
        """创建机械臂控制器，但不打开总线，也不使能硬件。

        硬件布局可由内置 ``model``、外部 ``config_path`` 或已解析的 ``config``
        提供，连接参数会覆盖所选机型配置。调用 :meth:`connect` 前总线保持关闭。
        默认启用机型配置中声明的夹爪；更换自定义末端时请传入
        ``enable_gripper=False``。
        """
        if config is not None and (model is not None or config_path is not None):
            raise ValueError("config cannot be combined with model or config_path")
        connection_channel = _connection_channel(port, channel)
        if config is None:
            loaded_config = _load_profile(config_path, model=model)
            self.config = _config_from_loaded(
                loaded_config,
                port=connection_channel,
                baud=baud,
                transport=transport,
                arm_control_mode=control_mode,
            )
            loaded_config = dict(loaded_config)
            loaded_config["channel"] = self.config.port
            loaded_config["transport"] = self.config.transport
            loaded_config["baud"] = self.config.baud
        else:
            self.config = replace(
                config,
                port=(
                    config.port
                    if connection_channel is None
                    else connection_channel
                ),
                baud=config.baud if baud is None else baud,
                transport=config.transport if transport is None else transport,
            )
            loaded_config = _actuator_config_from_sdk(self.config)
        self._validate_safety_config()
        has_configured_gripper = self.config.gripper is not None
        if enable_gripper is True and not has_configured_gripper:
            raise ValueError("the selected model does not configure a gripper")
        self.enable_gripper = (
            has_configured_gripper
            if enable_gripper is None
            else bool(enable_gripper)
        )
        active_joint_names = list(self.config.joint_names)
        if self.enable_gripper and self.config.gripper is not None:
            active_joint_names.append(self.config.gripper.name)
        self.robot = ArxDCan(
            config_data=loaded_config,
            joint_names=active_joint_names,
        )
        self._connected = False
        self._enabled = False
        self._configured = False
        self._faulted = False
        self._fault_reason: str | None = None
        self._safe_holding = False
        self._state_lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._dual_runtime_managed = False
        self._single_controller_group: ControllerGroup | None = None
        self._single_safety_runtime: NativeSafetyRuntime | None = None
        self._last_state: ArxDCanState | None = None
        self._mode = self.config.arm_control_mode.strip().lower().replace("_", "")

    @property
    def joint_names(self) -> tuple[str, ...]:
        """按命令和反馈向量的顺序返回机械臂关节名称。"""
        return self.config.joint_names

    @property
    def connected(self) -> bool:
        """返回配置的通信通道当前是否已打开。"""
        return self._connected

    @property
    def enabled(self) -> bool:
        """返回 SDK 是否认为当前活动电机已使能。"""
        runtime = self._single_safety_runtime
        if runtime is not None:
            self._sync_native_safety_flags(runtime.health)
        return self._enabled

    @property
    def has_gripper(self) -> bool:
        """返回当前控制器是否包含配置的夹爪。"""
        return self.enable_gripper and self.config.gripper is not None

    @property
    def faulted(self) -> bool:
        """返回故障是否已锁存，以及普通命令是否已被阻止。"""
        runtime = self._single_safety_runtime
        if runtime is not None:
            self._sync_native_safety_flags(runtime.health)
        return self._faulted

    @property
    def fault_reason(self) -> str | None:
        """返回锁存的故障说明；状态正常时返回 ``None``。"""
        runtime = self._single_safety_runtime
        if runtime is not None:
            self._sync_native_safety_flags(runtime.health)
        return self._fault_reason

    @property
    def safe_holding(self) -> bool:
        """返回看门狗是否正在重复发送最后一条安全命令。"""
        runtime = self._single_safety_runtime
        if runtime is not None:
            self._sync_native_safety_flags(runtime.health)
        return self._safe_holding

    @property
    def safety_health(self) -> SafetyHealth:
        """返回 motor 原生安全状态机快照。"""
        runtime = self._single_safety_runtime
        if runtime is not None:
            health = runtime.health
            self._sync_native_safety_flags(health)
            return health
        connected = self.connected
        state = (
            SafetyState.RUNNING
            if self.enabled
            else SafetyState.READY
            if connected
            else SafetyState.DISCONNECTED
        )
        transport = TransportHealth(
            connected=connected,
            healthy=connected,
            consecutive_send_failures=0,
            consecutive_feedback_failures=0,
            last_feedback_age_s=None,
            last_error=None,
        )
        inactive = TransportHealth(
            connected=False,
            healthy=False,
            consecutive_send_failures=0,
            consecutive_feedback_failures=0,
            last_feedback_age_s=None,
            last_error=None,
        )
        return SafetyHealth(
            state=state,
            fault_reason=self.fault_reason,
            last_successful_command_age_s=None,
            last_fresh_feedback_age_s=None,
            consecutive_send_failures=0,
            consecutive_feedback_failures=0,
            left_transport=transport,
            right_transport=inactive,
            motor_faults=(),
            unconfirmed_disable_motors=(),
            safe_holding=self.safe_holding,
            disable_confirmed=not self.enabled,
        )

    @property
    def last_enable_report(self) -> EnableReport | None:
        """返回最近一次原子使能报告；未创建原生 Runtime 时返回 ``None``。"""
        runtime = self._single_safety_runtime
        return None if runtime is None else runtime.last_enable_report

    @property
    def last_disable_report(self) -> DisableReport | None:
        """返回最近一次确定性失能报告；未创建原生 Runtime 时返回 ``None``。"""
        runtime = self._single_safety_runtime
        return None if runtime is None else runtime.last_disable_report

    @property
    def gripper_safety_health(self) -> GripperSafetyHealth | None:
        """返回产品夹爪的原生防堵转状态；无原生夹爪时返回 ``None``。"""
        return self.safety_health.left_gripper

    @property
    def communication_health(self) -> TransportHealth:
        """直接返回 motor 提供的结构化通信健康状态。"""
        return self.safety_health.left_transport

    def _native_motor_descriptors(
        self,
        *,
        side: int = 0,
        label: str | None = None,
    ) -> tuple[NativeMotorDescriptor, ...]:
        """生成供 Articore C++ runtime 使用的通用电机描述。"""
        prefix = "" if label is None else f"{label}/"
        descriptors = []
        for joint in self.config.arm_joints:
            position_range, _, _ = damiao_model_limits(joint.model)
            logical_lower = (
                -position_range if joint.lower_limit is None else joint.lower_limit
            )
            logical_upper = (
                position_range if joint.upper_limit is None else joint.upper_limit
            )
            motor_limits = (
                joint.direction * logical_lower,
                joint.direction * logical_upper,
            )
            descriptors.append(
                NativeMotorDescriptor(
                    motor=self.robot._motor_map[joint.name],
                    side=side,
                    name=f"{prefix}{joint.name}",
                    safe_kp=self.config.safe_hold_mit_kp,
                    safe_kd=self.config.safe_hold_mit_kd,
                    lower_position=min(motor_limits),
                    upper_position=max(motor_limits),
                )
            )
        if self.enable_gripper and self.config.gripper is not None:
            gripper = self.config.gripper
            protection = self.config.gripper_protection
            closed = self.config.gripper_closed_value
            opened = self.config.gripper_open_value
            descriptors.append(
                NativeMotorDescriptor(
                    motor=self.robot._motor_map[gripper.name],
                    side=side,
                    name=f"{prefix}{gripper.name}",
                    is_gripper=True,
                    safe_kp=protection.hold_kp,
                    safe_kd=protection.hold_kd,
                    overload_torque=protection.overload_torque,
                    retreat_distance=protection.retreat_distance,
                    contact_torque=protection.contact_torque,
                    motion_window_s=protection.motion_window_s,
                    stall_movement=protection.stall_movement,
                    min_position_error=protection.min_position_error,
                    contact_hold_s=protection.contact_hold_s,
                    overload_hold_s=protection.overload_hold_s,
                    hold_offset=protection.hold_offset,
                    retreat_retry_s=protection.overload_retreat_interval_s,
                    open_position=opened,
                    closed_position=closed,
                    normal_kp=gripper.mit_kp,
                    normal_kd=gripper.mit_kd,
                    close_speed=protection.close_speed,
                    max_step_interval_s=protection.max_step_interval_s,
                    closing_direction=1.0 if closed > opened else -1.0,
                    lower_position=min(closed, opened),
                    upper_position=max(closed, opened),
                )
            )
        return tuple(descriptors)

    def _native_joint_control_configs(self) -> tuple[NativeJointControlConfig, ...]:
        """生成 runtime 使用的原生坐标限位和 MIT 默认参数。"""
        configs = []
        for joint in self.config.arm_joints:
            position_range, native_velocity, native_torque = damiao_model_limits(
                joint.model
            )
            logical_lower = (
                -position_range if joint.lower_limit is None else joint.lower_limit
            )
            logical_upper = (
                position_range if joint.upper_limit is None else joint.upper_limit
            )
            motor_limits = (
                joint.direction * logical_lower,
                joint.direction * logical_upper,
            )
            configured_velocity_range = (
                native_velocity
                if joint.velocity_range is None
                else joint.velocity_range
            )
            velocity_scale = native_velocity / configured_velocity_range
            raw_velocity_limit = joint.pv_vlim * max(1.0, velocity_scale)
            configured_torque_range = (
                native_torque if joint.torque_range is None else joint.torque_range
            )
            torque_scale = native_torque / configured_torque_range
            logical_torque_limit = (
                configured_torque_range
                if joint.effort_limit is None
                else joint.effort_limit
            )
            raw_torque_limit = min(
                native_torque,
                logical_torque_limit * torque_scale,
            )
            configs.append(
                NativeJointControlConfig(
                    motor=self.robot._motor_map[joint.name],
                    lower_position=min(motor_limits),
                    upper_position=max(motor_limits),
                    velocity_limit=raw_velocity_limit,
                    torque_limit=raw_torque_limit,
                    mit_kp=joint.mit_kp,
                    mit_kd=joint.mit_kd,
                )
            )
        return tuple(configs)

    def _default_joint_velocity_limits(self) -> np.ndarray:
        """返回 SDK 默认的逐关节速度上限。"""
        configured = np.asarray(
            self.config.product_velocity_at_400
            or tuple(joint.pv_vlim for joint in self.config.arm_joints),
            dtype=np.float64,
        )
        return configured * (_DEFAULT_SPEED_LEVEL / _MAX_SPEED_LEVEL)

    def _create_single_safety_runtime(self) -> None:
        """在底层对象提供原生句柄时创建单通道常驻安全运行时。"""
        if self._dual_runtime_managed:
            return
        if not hasattr(self.robot.arm, "_controller_for_batch"):
            return
        controller = self._controller_for_parallel_batch()
        if not getattr(controller, "_ptr", None):
            return
        descriptors = self._native_motor_descriptors()
        if not all(getattr(item.motor, "_ptr", None) for item in descriptors):
            return
        group = ControllerGroup([controller])
        if not getattr(group, "_ptr", None):
            group.close()
            return
        try:
            runtime = NativeSafetyRuntime(
                controller_group=group,
                left_controller=controller,
                right_controller=None,
                motors=descriptors,
                joints=self._native_joint_control_configs(),
                control_hz=self.config.control_hz,
                command_timeout_s=self.config.command_timeout_s,
                enable_grace_s=self.config.enable_grace_s,
                safe_hold_hz=self.config.safe_hold_hz,
                feedback_check_hz=self.config.feedback_check_hz,
                feedback_failure_threshold=self.config.feedback_fault_threshold,
                feedback_max_age_s=self.config.max_cached_feedback_age_s,
                safe_hold_failure_threshold=self.config.safe_hold_failure_threshold,
                safe_pv_velocity_limit=self.config.safe_hold_pv_velocity_limit,
                # ABI 字段为兼容保留；1.8 正常运行时夹爪跟随机械臂控制频率。
                gripper_control_hz=self.config.control_hz,
                gripper_fault_action=self.config.gripper_fault_action,
                trajectory_execution=self.config.trajectory_execution,
            )
            runtime.connect()
        except Exception:
            group.close()
            raise
        self._single_controller_group = group
        self._single_safety_runtime = runtime

    def _sync_native_safety_flags(self, health: SafetyHealth) -> None:
        active = health.state in {
            SafetyState.ENABLED,
            SafetyState.RUNNING,
            SafetyState.SAFE_HOLD,
        } or (health.state is SafetyState.FAULT and not health.disable_confirmed)
        with self._state_lock:
            self._enabled = active
            self._safe_holding = health.state is SafetyState.SAFE_HOLD
            self._faulted = health.state in {
                SafetyState.SAFE_HOLD,
                SafetyState.FAULT,
            }
            self._fault_reason = health.fault_reason

    def connect(self) -> None:
        """打开配置的总线并重置 SDK 临时状态。

        此操作可重复调用，不会配置控制模式或使能电机。只读场景连接后可直接读取
        状态；需要运动时调用 :meth:`enable`，SDK 会自动完成首次模式配置。
        """
        if self._connected:
            return
        self.robot.connect()
        with self._state_lock:
            self._connected = True
            self._configured = False
            self._enabled = False
            self._faulted = False
            self._fault_reason = None
            self._safe_holding = False
            self._last_state = None
        try:
            self._create_single_safety_runtime()
            if (
                not self._dual_runtime_managed
                and self.config.model != "custom"
                and self._single_safety_runtime is None
            ):
                raise RuntimeError(
                    "motor-drive-layer 0.8.5 native safety runtime is unavailable"
                )
        except Exception:
            try:
                self.robot.disconnect(disable=False)
            finally:
                with self._state_lock:
                    self._connected = False
            raise

    def configure(self, mode: str | None = None) -> None:
        """显式配置机械臂模式和可选夹爪，供高级控制与维护流程使用。

        ``mode`` 会覆盖机械臂组的默认模式。夹爪启用时始终进入 MIT 模式。
        普通用户不需要主动调用此方法，首次 :meth:`enable` 会自动配置构造机械臂时
        选择的控制模式。任何配置失败都会锁存故障，并尝试紧急失能。
        """
        self._require_operational()
        if self._enabled:
            raise RuntimeError(
                "cannot configure control mode while the arm is enabled; "
                "call disable() first"
            )
        try:
            self.configure_mode(mode or self._mode)
            if self.enable_gripper and self.config.gripper is not None:
                if not self.robot.gripper.mode_mit():
                    raise RuntimeError("ARX-D-CAN gripper did not enter MIT mode")
            for name in self._active_joint_names():
                self.robot._motor_map[name].set_can_timeout_ms(
                    self.config.motor_communication_timeout_ms
                )
        except Exception as exc:
            self._trip_fault(f"configuration failed: {exc}")
            raise
        self._configured = True

    def close(self, *, disable: bool = True) -> None:
        """停止生成控制命令并关闭总线。

        原生 Runtime 的关闭包含 ABI 1.8 确定性失能事务；失败时保留 Runtime、
        ControllerGroup 和 Transport，供调用方读取结构化报告并重试。``disable=False``
        仅影响没有原生 Runtime 的自定义后端。
        """
        errors: list[Exception] = []
        runtime = self._single_safety_runtime
        group = self._single_controller_group
        if runtime is not None:
            try:
                runtime.close()
            except Exception as exc:
                with self._state_lock:
                    self._enabled = True
                    self._faulted = True
                    self._fault_reason = f"close failed: {exc}"
                    self._safe_holding = False
                # ABI 1.8：关闭失败时不能继续释放任何被 Runtime 引用的句柄。
                raise
            self._single_safety_runtime = None
            with self._state_lock:
                self._enabled = False
                self._safe_holding = False
        if group is not None:
            try:
                group.close()
            except Exception as exc:
                errors.append(exc)
            else:
                self._single_controller_group = None
        if errors:
            raise RuntimeError(
                f"ARX-D-CAN ControllerGroup close failed: {errors[0]}"
            ) from errors[0]
        if self._connected:
            try:
                if disable and runtime is None:
                    self.robot.disconnect()
                else:
                    self.robot.disconnect(disable=False)
            except Exception as exc:
                errors.append(exc)
        if not errors:
            with self._state_lock:
                self._connected = False
                self._configured = False
                self._safe_holding = False
                self._enabled = False
        if errors:
            raise RuntimeError(f"ARX-D-CAN close failed: {errors[0]}") from errors[0]

    def enable(
        self,
        *,
        initial_positions: Sequence[float] | None = None,
        initial_velocities: Sequence[float] | None = None,
        initial_torques: Sequence[float] | None = None,
        mit_kp: float | Sequence[float] | None = None,
        mit_kd: float | Sequence[float] | None = None,
    ) -> None:
        """通过原生原子事务使能活动电机并启动命令安全监控。

        机械臂必须已连接；首次使能时，Python 只配置构造时选择的控制模式和电机
        参数，物理使能、当前位置保持、反馈确认和失败回滚均由 ABI 1.8 Runtime
        完成。在 MIT 模式下仍可提供一条完整的后续初始命令。操作失败时抛出的
        :class:`NativeEnableError` 携带结构化使能报告。
        """
        self._require_operational()
        if not self._configured:
            self.configure()
        if initial_positions is not None and self._mode != "mit":
            raise ValueError("initial position seeding is only supported in MIT mode")
        joint_count = len(self.config.arm_joints)
        initial_position_vector = None
        initial_velocity_vector = None
        initial_torque_vector = None
        kp_vector = None
        kd_vector = None
        if initial_positions is not None:
            vectors = {
                "initial_positions": initial_positions,
                "initial_velocities": (
                    [0.0] * joint_count
                    if initial_velocities is None
                    else initial_velocities
                ),
                "initial_torques": (
                    [0.0] * joint_count
                    if initial_torques is None
                    else initial_torques
                ),
            }
            parsed = {}
            for name, values in vectors.items():
                if len(values) != joint_count:
                    raise ValueError(f"{name} must contain {joint_count} values")
                array = np.asarray(values, dtype=np.float64)
                if not np.all(np.isfinite(array)):
                    raise ValueError(f"{name} must be finite")
                parsed[name] = array
            kp_vector = _mit_gain_vector(mit_kp, joint_count=joint_count, name="Kp")
            kd_vector = _mit_gain_vector(mit_kd, joint_count=joint_count, name="Kd")
            kp_vector = self._resolved_mit_gains(kp_vector, gain="kp")
            kd_vector = self._resolved_mit_gains(kd_vector, gain="kd")
            initial_command = self._make_mit_command(
                parsed["initial_positions"],
                parsed["initial_velocities"],
                kp_vector,
                kd_vector,
                parsed["initial_torques"],
            )
            initial_position_vector = np.asarray(initial_command.positions)
            initial_velocity_vector = np.asarray(initial_command.velocities)
            initial_torque_vector = np.asarray(initial_command.feedforward_torques)
        runtime = self._single_safety_runtime
        if runtime is not None:
            try:
                runtime.enable(self._mode)
                if initial_position_vector is not None:
                    commands = self.robot.arm._make_mit_batch_commands(
                        initial_position_vector,
                        vel=initial_velocity_vector,
                        kp=kp_vector,
                        kd=kd_vector,
                        tau=initial_torque_vector,
                    )
                    persistent = bool(
                        np.all(initial_velocity_vector == 0.0)
                        and np.all(initial_torque_vector == 0.0)
                    )
                    runtime.submit_mit(
                        commands,
                        lifetime=(
                            CommandLifetime.HOLD_UNTIL_REPLACED
                            if persistent
                            else CommandLifetime.STREAMING
                        ),
                    )
            except Exception:
                # 健康快照只是补充状态，不能覆盖携带 EnableReport 的原始异常。
                try:
                    self._sync_native_safety_flags(runtime.health)
                except Exception:
                    pass
                raise
            self._sync_native_safety_flags(runtime.health)
            return

        # 仅保留给没有原生句柄的自定义控制器和测试桩。内置产品必须由 ABI 1.8
        # Runtime 独占物理使能事务，不能在 Python 中提前逐组使能。
        try:
            if initial_position_vector is None:
                self.robot.arm.enable()
            else:
                self.robot.arm.enable(
                    mit_position=initial_position_vector,
                    mit_velocity=initial_velocity_vector,
                    mit_kp=kp_vector,
                    mit_kd=kd_vector,
                    mit_tau=initial_torque_vector,
                )
            if self.enable_gripper and self.config.gripper is not None:
                self.robot.gripper.enable()
        except Exception as exc:
            self._trip_fault(f"enable failed: {exc}")
            raise
        with self._state_lock:
            self._enabled = True

    def disable(self) -> None:
        """停止后台控制，并向所有电机发送紧急失能命令。

        成功后会清除保留的机械臂命令。如果无法确认电机已在物理层失能，SDK 会继续
        保持故障状态并将电机报告为已使能，避免呈现不安全的虚假状态。
        """
        self._require_connected()
        runtime = self._single_safety_runtime
        if runtime is not None:
            try:
                runtime.disable()
            except Exception as exc:
                # 健康状态读取失败不能覆盖携带 DisableReport 的原始异常。
                try:
                    health = runtime.health
                except Exception:
                    with self._state_lock:
                        self._enabled = True
                        self._faulted = True
                        self._fault_reason = f"disable failed: {exc}"
                        self._safe_holding = False
                else:
                    self._sync_native_safety_flags(health)
                raise
            else:
                self._sync_native_safety_flags(runtime.health)
            return
        try:
            self.robot.estop()
        except Exception as exc:
            with self._state_lock:
                # 尚未确认物理失能，因此保留保守的软件状态，不假定电机已经安全。
                self._enabled = True
                self._faulted = True
                self._fault_reason = f"disable failed: {exc}"
                self._safe_holding = False
            raise
        with self._state_lock:
            self._enabled = False
            self._safe_holding = False

    def clear_fault(self) -> None:
        """确认反馈可用后，清除 SDK 的故障锁存。

        除非从 ``SAFE_HOLD`` 状态开始恢复，否则会紧急失能电机。从安全保持恢复时
        保留当前控制模式；其他情况由下一次 :meth:`enable` 自动重新配置并使能。
        """
        self._require_connected()
        runtime = self._single_safety_runtime
        if runtime is not None:
            health = runtime.health
            if health.state is SafetyState.SAFE_HOLD:
                runtime.disable()
            elif health.state is SafetyState.FAULT:
                runtime.disable()
                runtime.recover()
            elif health.state is not SafetyState.READY:
                raise RuntimeError(
                    "native safety runtime can only clear SAFE_HOLD or FAULT"
                )
            self._configured = False
            self._sync_native_safety_flags(runtime.health)
            return
        try:
            self.robot.estop()
        except Exception:
            pass
        self.robot.get_state(
            request_feedback=True,
            require_complete=True,
            joint_names=self._active_joint_names(),
        )
        with self._state_lock:
            self._faulted = False
            self._fault_reason = None
            self._safe_holding = False
            self._enabled = False
            self._configured = False

    def clear_motor_faults(self) -> tuple[str, ...]:
        """清除活动电机故障，并返回成功清除故障的电机名称。

        此操作覆盖已配置的机械臂和可选的活动夹爪，完成后 SDK 始终处于未配置、
        未使能状态。清除失败仍会作为 SDK 故障锁存。
        """
        self._require_connected()
        try:
            completed = self.robot.clear_errors(
                joint_names=self._active_joint_names(),
            )
        except Exception as exc:
            with self._state_lock:
                self._enabled = False
                self._configured = False
                self._faulted = True
                self._fault_reason = f"motor fault clear failed: {exc}"
                self._safe_holding = False
            raise

        with self._state_lock:
            self._enabled = False
            self._configured = False
            self._faulted = False
            self._fault_reason = None
            self._safe_holding = False
        return completed

    def recover(self) -> None:
        """清除锁存故障，并在全部安全条件确认后恢复到 ``READY``。

        此方法不会自动重新使能电机。恢复成功后可显式调用 :meth:`enable`；恢复失败
        时机械臂继续保持故障锁存，不会进入部分恢复状态。
        """
        self.clear_fault()

    def configure_mode(self, mode: str = "posvel") -> None:
        """在 ``posvel``（PV）与 ``mit`` 两种控制模式之间切换机械臂组。

        总线必须已连接、没有锁存故障且电机已经失能。
        """
        self._require_operational()
        if self._enabled:
            raise RuntimeError(
                "cannot switch control mode while the arm is enabled; "
                "call disable() first"
            )
        normalized = mode.strip().lower().replace("_", "")
        if normalized in ("posvel", "pv"):
            with self._io_lock:
                if not self.robot.arm.mode_pos_vel():
                    raise RuntimeError("ARX-D-CAN arm did not enter POS_VEL mode")
            self._mode = "posvel"
            return
        if normalized == "mit":
            with self._io_lock:
                if not self.robot.arm.mode_mit():
                    raise RuntimeError("ARX-D-CAN arm did not enter MIT mode")
            self._mode = "mit"
            return
        raise ValueError("mode must be 'posvel' or 'mit'")

    def read_state(self) -> ArxDCanState:
        """请求并返回一组新鲜、完整的机械臂及夹爪状态。

        通信失败、反馈不完整或电机报告故障时直接抛出对应异常，不会把历史缓存伪装成
        当前状态。只需读取最近一次成功状态时，请显式调用 :meth:`read_cached_state`。
        """
        return self._read_fresh_state(serialize_io=True)

    def read_cached_state(self) -> ArxDCanState:
        """从 motor 原生反馈缓存返回状态，不主动请求通信帧。

        原生 Runtime 会在后台刷新各电机缓存，因此该方法可供控制循环读取最新快照，
        无需再创建 Python 反馈线程。缓存的新鲜度和通信故障由
        :attr:`safety_health` 中的状态机统一判断。
        """
        self._require_connected()
        active_joint_names = self._active_joint_names()
        pos, vel, tau = self.robot.get_state(
            request_feedback=False,
            require_complete=True,
            joint_names=active_joint_names,
        )
        status_codes = self.robot.get_status_codes(
            joint_names=active_joint_names,
        )
        return self._build_state(pos, vel, tau, status_codes)

    def refresh_feedback_background(self) -> ArxDCanState:
        """在不占用 SDK 命令锁的情况下请求完整反馈。

        此方法供专用的低频监控线程使用。motor-drive-layer 串口传输与节拍发送总线会
        自行串行化 I/O，因此这里的反馈超时不会通过 ``_io_lock`` 阻塞主命令循环。
        """
        return self._read_fresh_state(serialize_io=False)

    def _read_fresh_state(
        self,
        *,
        serialize_io: bool,
    ) -> ArxDCanState:
        """读取一组新鲜反馈；安全判断由 motor 原生运行时执行。"""
        self._require_connected()
        active_joint_names = self._active_joint_names()

        def read_raw_state():
            pos, vel, tau = self.robot.get_state(
                request_feedback=True,
                require_complete=True,
                joint_names=active_joint_names,
            )
            status_codes = self.robot.get_status_codes(
                joint_names=active_joint_names,
            )
            return pos, vel, tau, status_codes

        try:
            if serialize_io:
                with self._io_lock:
                    pos, vel, tau, status_codes = read_raw_state()
            else:
                pos, vel, tau, status_codes = read_raw_state()
        except Exception as exc:
            runtime = self._single_safety_runtime
            if runtime is not None:
                runtime.report_feedback_failure(0, str(exc))
            raise
        return self._build_state(pos, vel, tau, status_codes)

    def _build_state(
        self,
        pos,
        vel,
        tau,
        status_codes: dict[str, int],
    ) -> ArxDCanState:
        """把 motor 缓存中的原始向量组装为公开状态对象。"""
        disabled_motors = [
            name for name, status in status_codes.items() if status == 0
        ]
        if self._enabled and disabled_motors:
            error = UnexpectedMotorStateError(
                "motors unexpectedly disabled after feedback recovery: "
                + ", ".join(disabled_motors),
                status_codes={name: status_codes[name] for name in disabled_motors},
            )
            raise error
        arm_count = len(self.config.arm_joints)
        arm_pos = pos[:arm_count]
        arm_vel = vel[:arm_count]
        arm_tau = tau[:arm_count]
        arm_pos, arm_vel, arm_tau = self._transform_feedback_vectors(
            arm_pos,
            arm_vel,
            arm_tau,
        )
        gripper_state = None
        if self.config.gripper is not None and len(pos) > arm_count:
            motor_position = float(pos[arm_count])
            gripper_state = GripperState(
                name=self.config.gripper.name,
                motor_id=self.config.gripper.motor_id,
                feedback_id=self.config.gripper.feedback_id,
                opening=self._gripper_opening_from_motor_position(motor_position),
                motor_position=motor_position,
                motor_velocity=float(vel[arm_count]),
                torque=float(tau[arm_count]),
            )
        state = ArxDCanState(
            arm=JointState(
                names=self.config.joint_names,
                positions=tuple(float(value) for value in arm_pos),
                velocities=tuple(float(value) for value in arm_vel),
                torques=tuple(float(value) for value in arm_tau),
            ),
            gripper=gripper_state,
        )
        with self._state_lock:
            self._last_state = state
        return state

    def scan_ids(
        self,
        *,
        start_id: int = 1,
        end_id: int = 16,
        model: str = "4340P",
        timeout_ms: int = 30,
        feedback_base: str = "0x10",
    ) -> list[int]:
        """运行只读电机扫描器并返回通过校验的 CAN ID。

        扫描使用当前实例的通信配置，交由 motor-drive-layer 命令行工具执行，不会配置
        或使能电机。候选反馈 ID 根据 ``feedback_base`` 推导，无效反馈帧会由
        :func:`parse_scan_ids` 排除。
        """
        command = build_scan_command(
            python_executable=sys.executable,
            port=self.config.port,
            baud=self.config.baud,
            transport=self.config.transport,
            model=model,
            start_id=start_id,
            end_id=end_id,
            feedback_base=feedback_base,
            timeout_ms=timeout_ms,
        )
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        if result.returncode != 0:
            raise RuntimeError(
                result.stderr.strip()
                or result.stdout.strip()
                or "motor-drive-layer scan failed"
            )
        return parse_scan_ids(result.stdout, model=model)

    def read_motor_diagnostics(self, *, timeout_ms: int = 100):
        """读取所有活动电机的状态、控制模式和温度，不发送控制命令。

        ``timeout_ms`` 是每次整臂反馈请求和每台电机寄存器请求各自的超时，不是
        整次诊断的总超时。普通用户使用默认的 100 ms 即可；该参数主要用于高级
        故障诊断。
        """
        from .diagnostics import read_motor_diagnostics

        return read_motor_diagnostics(self, timeout_ms=timeout_ms)

    def _controller_for_parallel_batch(self):
        """返回机械臂组的底层 Controller。"""
        self._require_connected()
        return self.robot.arm._controller_for_batch()

    def _validated_joint_positions(
        self,
        positions: Sequence[float],
        *,
        name: str | None = None,
    ) -> tuple[float, ...]:
        """校验逻辑关节目标，任何关节超出 URDF 限位时拒绝整条命令。"""
        values = tuple(float(value) for value in positions)
        joint_count = len(self.config.arm_joints)
        prefix = "" if name is None else f"{name} "
        if len(values) != joint_count:
            raise ValueError(
                f"{prefix}expected {joint_count} joint positions, got {len(values)}"
            )
        if any(not math.isfinite(value) for value in values):
            raise ValueError(f"{prefix}joint positions must be finite")

        violations = []
        for joint, value in zip(self.config.arm_joints, values):
            below = joint.lower_limit is not None and value < joint.lower_limit
            above = joint.upper_limit is not None and value > joint.upper_limit
            if not (below or above):
                continue
            lower = "-inf" if joint.lower_limit is None else f"{joint.lower_limit:.6g}"
            upper = "inf" if joint.upper_limit is None else f"{joint.upper_limit:.6g}"
            violations.append(
                f"{joint.name}={value:.6g} rad ({math.degrees(value):.3f} deg), "
                f"allowed [{lower}, {upper}] rad"
            )
        if violations:
            raise ValueError(
                f"{prefix}joint positions exceed URDF limits: "
                + "; ".join(violations)
            )
        return values

    def _prepare_parallel_joint_positions(
        self,
        positions: Sequence[float],
        *,
        velocities: Sequence[float] | None = None,
        velocity_limits: Sequence[float] | None = None,
        torques: Sequence[float] | None = None,
        mit_kp: float | Sequence[float] | None = None,
        mit_kd: float | Sequence[float] | None = None,
    ) -> _PreparedJointPositionBatch | None:
        """校验一侧关节目标，并生成可交给 ControllerGroup 的命令。"""
        self._require_connected()
        self._require_operational()
        if not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        joint_count = len(self.config.arm_joints)
        values = self._validated_joint_positions(positions)
        logical_position = np.asarray(values, dtype=np.float64)

        def optional_vector(
            vector: Sequence[float] | None,
            *,
            name: str,
        ) -> np.ndarray | None:
            if vector is None:
                return None
            result = np.asarray(vector, dtype=np.float64).reshape(-1)
            if len(result) != joint_count:
                raise ValueError(
                    f"expected {joint_count} joint {name}, got {len(result)}"
                )
            if not np.all(np.isfinite(result)):
                raise ValueError(f"joint {name} must be finite")
            return result

        velocity_target = optional_vector(velocities, name="velocities")
        velocity_limit_target = optional_vector(
            velocity_limits,
            name="velocity limits",
        )
        if velocity_limit_target is not None and np.any(velocity_limit_target <= 0.0):
            raise ValueError("joint velocity limits must be positive")
        torque_target = optional_vector(torques, name="torques")
        kp_target = _mit_gain_vector(mit_kp, joint_count=joint_count, name="Kp")
        kd_target = _mit_gain_vector(mit_kd, joint_count=joint_count, name="Kd")
        configured_velocity_limits = np.asarray(
            [joint.pv_vlim for joint in self.config.arm_joints],
            dtype=np.float64,
        )
        if velocity_target is not None and np.any(
            np.abs(velocity_target) > configured_velocity_limits
        ):
            raise ValueError("joint velocities exceed YAML vlim")
        if velocity_limit_target is not None and np.any(
            velocity_limit_target > configured_velocity_limits
        ):
            raise ValueError("joint velocity limits exceed YAML vlim")

        if self._mode in ("posvel", "pv"):
            if velocity_limit_target is None:
                velocity_limit_target = self._default_joint_velocity_limits()
            if any(
                value is not None
                for value in (velocity_target, torque_target, kp_target, kd_target)
            ):
                raise ValueError(
                    "velocities, torques and MIT gains are only supported in MIT mode"
                )
            motor_position, _, _, velocity_limit = self._transform_command_vectors(
                logical_position,
                velocity_limits=velocity_limit_target,
            )
            commands = self.robot.arm._make_pos_vel_batch_commands(
                motor_position,
                vlim=velocity_limit,
            )
            return _PreparedJointPositionBatch(
                mode="pv",
                commands=commands,
            )

        if self._mode == "mit":
            if velocity_limit_target is not None:
                raise ValueError("velocity limits are only supported in PV mode")
            command = self._make_mit_command(
                logical_position,
                np.zeros(joint_count) if velocity_target is None else velocity_target,
                self._resolved_mit_gains(kp_target, gain="kp"),
                self._resolved_mit_gains(kd_target, gain="kd"),
                np.zeros(joint_count) if torque_target is None else torque_target,
            )
            position, velocity, kp, kd, torque = self._compose_mit_motor_command(
                command
            )
            commands = self.robot.arm._make_mit_batch_commands(
                position,
                vel=velocity,
                kp=kp,
                kd=kd,
                tau=torque,
            )
            return _PreparedJointPositionBatch(
                mode="mit",
                commands=commands,
            )

        raise ValueError("mode must be 'posvel' or 'mit'")

    def _prepare_joint_trajectory_targets(
        self,
        positions: Sequence[float],
        *,
        velocity: float | Sequence[float] | None,
    ) -> tuple[tuple[object, float, float], ...]:
        """把逻辑目标转换为 runtime 轨迹使用的原生坐标。"""
        self._require_connected()
        self._require_operational()
        if not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        joint_count = len(self.config.arm_joints)
        target = np.asarray(
            self._validated_joint_positions(positions),
            dtype=np.float64,
        )

        configured_limits = np.asarray(
            [joint.pv_vlim for joint in self.config.arm_joints],
            dtype=np.float64,
        )
        if velocity is None:
            velocities = self._default_joint_velocity_limits()
        else:
            velocities = np.asarray(velocity, dtype=np.float64)
            if velocities.ndim == 0:
                velocities = np.full(
                    joint_count,
                    float(velocities),
                    dtype=np.float64,
                )
            else:
                velocities = velocities.reshape(-1)
        if len(velocities) != joint_count:
            raise ValueError(
                f"expected {joint_count} trajectory velocities, got {len(velocities)}"
            )
        if (
            np.any(~np.isfinite(velocities))
            or np.any(velocities <= 0.0)
            or np.any(velocities > configured_limits)
        ):
            raise ValueError(
                "trajectory velocities must be finite, positive, and no greater "
                "than each joint's YAML vlim"
            )

        targets = []
        for joint, position, velocity_limit in zip(
            self.config.arm_joints,
            target,
            velocities,
        ):
            raw_velocity_limit = float(velocity_limit)
            if self._mode == "mit":
                _, native_velocity, _ = damiao_model_limits(joint.model)
                configured_velocity_range = (
                    native_velocity
                    if joint.velocity_range is None
                    else joint.velocity_range
                )
                raw_velocity_limit *= native_velocity / configured_velocity_range
            targets.append(
                (
                    self.robot._motor_map[joint.name],
                    joint.direction * float(position),
                    raw_velocity_limit,
                )
            )
        return tuple(targets)

    def _submit_joint_positions(
        self,
        positions: Sequence[float],
        *,
        velocities: Sequence[float] | None = None,
        velocity_limits: Sequence[float] | None = None,
        torques: Sequence[float] | None = None,
        mit_kp: float | Sequence[float] | None = None,
        mit_kd: float | Sequence[float] | None = None,
        require_enabled: bool = True,
        lifetime: CommandLifetime = CommandLifetime.STREAMING,
    ) -> None:
        """在 SDK 内部更新 runtime 的完整逻辑关节目标。

        ``positions`` 始终按 :attr:`joint_names` 排列。POS_VEL 接受逐关节
        ``velocity_limits``；省略时使用 SDK 默认的逐关节速度上限。
        MIT 接受目标 ``velocities``、前馈 ``torques`` 以及当前数据包使用的 Kp/Kd。
        该调用只更新容量为一的最新目标邮箱；固定频率发送、抢占、看门狗和安全状态机
        均由 motor 原生运行时处理。
        """
        if self._dual_runtime_managed:
            raise RuntimeError(
                "this arm is managed by ArxDCanDualArm; submit a complete left/right "
                "command through ArxDCanDualArm"
            )
        self._require_connected()
        self._require_operational()
        if require_enabled and not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        joint_count = len(self.config.arm_joints)
        position_target = self._validated_joint_positions(positions)
        velocity_target: np.ndarray | None = None
        if velocities is not None:
            if len(velocities) != joint_count:
                raise ValueError(
                    f"expected {joint_count} joint velocities, "
                    f"got {len(velocities)}"
                )
            if any(not math.isfinite(float(value)) for value in velocities):
                raise ValueError("joint velocities must be finite")
            velocity_target = np.asarray(velocities, dtype=np.float64)
        velocity_limit_target: np.ndarray | None = None
        if velocity_limits is not None:
            if len(velocity_limits) != joint_count:
                raise ValueError(
                    f"expected {joint_count} joint velocity limits, "
                    f"got {len(velocity_limits)}"
                )
            if any(
                not math.isfinite(float(value)) or float(value) <= 0.0
                for value in velocity_limits
            ):
                raise ValueError("joint velocity limits must be finite and positive")
            velocity_limit_target = np.asarray(velocity_limits, dtype=np.float64)
        torque_target: np.ndarray | None = None
        if torques is not None:
            if len(torques) != joint_count:
                raise ValueError(
                    f"expected {joint_count} joint torques, "
                    f"got {len(torques)}"
                )
            if any(not math.isfinite(float(value)) for value in torques):
                raise ValueError("joint torques must be finite")
            torque_target = np.asarray(torques, dtype=np.float64)
        mit_kp_target = _mit_gain_vector(
            mit_kp,
            joint_count=joint_count,
            name="Kp",
        )
        mit_kd_target = _mit_gain_vector(
            mit_kd,
            joint_count=joint_count,
            name="Kd",
        )
        configured_velocity_limits = np.asarray(
            [joint.pv_vlim for joint in self.config.arm_joints],
            dtype=np.float64,
        )
        if velocity_target is not None and np.any(
            np.abs(velocity_target) > configured_velocity_limits
        ):
            raise ValueError("joint velocities exceed YAML vlim")
        if velocity_limit_target is not None and np.any(
            velocity_limit_target > configured_velocity_limits
        ):
            raise ValueError("joint velocity limits exceed YAML vlim")
        active_mode = self._mode
        if active_mode in ("posvel", "pv"):
            if velocity_limit_target is None:
                velocity_limit_target = self._default_joint_velocity_limits()
            if velocity_target is not None:
                raise ValueError("target velocities are only supported in MIT mode")
            if torque_target is not None:
                raise ValueError("torques are only supported in MIT mode")
            if mit_kp_target is not None or mit_kd_target is not None:
                raise ValueError("MIT Kp/Kd are only supported in MIT mode")
        if active_mode == "mit" and velocity_limit_target is not None:
            raise ValueError("velocity limits are only supported in PV mode")
        logical_position_target = np.array(
            position_target,
            dtype=np.float64,
        )
        try:
            if active_mode in ("posvel", "pv"):
                motor_position_target, _, _, velocity_limit_target = (
                    self._transform_command_vectors(
                        logical_position_target,
                        velocity_limits=velocity_limit_target,
                    )
                )
                runtime = self._single_safety_runtime
                if runtime is not None:
                    commands = self.robot.arm._make_pos_vel_batch_commands(
                        motor_position_target,
                        vlim=velocity_limit_target,
                    )
                    runtime.submit_pos_vel(commands, lifetime=lifetime)
                    self._sync_native_safety_flags(runtime.health)
                else:
                    with self._io_lock:
                        self.robot.arm.send_pos_vel(
                            motor_position_target,
                            vlim=velocity_limit_target,
                            strict=True,
                        )
                return
            if active_mode == "mit":
                command = self._make_mit_command(
                    logical_position_target,
                    (
                        np.zeros(joint_count)
                        if velocity_target is None
                        else velocity_target
                    ),
                    self._resolved_mit_gains(mit_kp_target, gain="kp"),
                    self._resolved_mit_gains(mit_kd_target, gain="kd"),
                    np.zeros(joint_count) if torque_target is None else torque_target,
                )
                runtime = self._single_safety_runtime
                if runtime is not None:
                    position, velocity, kp, kd, torque = (
                        self._compose_mit_motor_command(command)
                    )
                    commands = self.robot.arm._make_mit_batch_commands(
                        position,
                        vel=velocity,
                        kp=kp,
                        kd=kd,
                        tau=torque,
                    )
                    runtime.submit_mit(commands, lifetime=lifetime)
                    self._sync_native_safety_flags(runtime.health)
                elif self._enabled:
                    self._send_mit_command(command, strict=True)
                return
        except Exception as exc:
            if self._single_safety_runtime is not None:
                self._sync_native_safety_flags(self._single_safety_runtime.health)
            raise
        raise ValueError("mode must be 'posvel' or 'mit'")

    def stream_joint_positions(
        self,
        positions: Sequence[float],
        *,
        velocity_limits: Sequence[float] | None = None,
    ) -> None:
        """在 PV 模式下更新实时关节目标，不生成点到点轨迹。

        该接口面向已经连续生成目标的 ROS、遥操作和视觉跟随程序。普通点到点
        运动应使用 :meth:`move_joint_positions`，由 C++ runtime 完成平滑插值。
        """
        if self._mode not in ("posvel", "pv"):
            raise RuntimeError(
                "stream_joint_positions() is only available in PV mode; "
                "use move_joint_positions() for normal MIT motion"
            )
        self._submit_joint_positions(
            positions,
            velocity_limits=velocity_limits,
            lifetime=CommandLifetime.STREAMING,
        )

    def move_joint_positions(
        self,
        positions: Sequence[float],
        *,
        velocity: float | Sequence[float] | None = None,
        profile: str = "min_jerk",
    ) -> ArxDCanState:
        """由 C++ runtime 平滑移动到目标，并阻塞到反馈收敛或轨迹失败。

        ``velocity`` 是实际轨迹速度，单位为 rad/s；标量会应用到全部关节，序列按
        :attr:`joint_names` 排列。留空时使用 SDK 默认轨迹速度。``profile`` 可选
        ``min_jerk``（默认）或 ``linear``。
        """
        if self._dual_runtime_managed:
            raise RuntimeError(
                "this arm is managed by ArxDCanDualArm; call the dual-arm "
                "move_joint_positions()"
            )
        runtime = self._single_safety_runtime
        if runtime is None:
            raise RuntimeError("native joint trajectory runtime is not connected")
        targets = self._prepare_joint_trajectory_targets(
            positions,
            velocity=velocity,
        )
        trajectory_id = runtime.start_joint_trajectory(targets, profile=profile)
        result = runtime.wait_trajectory(trajectory_id)
        if result.status is not TrajectoryStatus.COMPLETED:
            detail = f": {result.error}" if result.error else ""
            raise RuntimeError(
                f"joint trajectory {trajectory_id} {result.status.value}{detail}"
            )
        return self.read_state()

    def record_trajectory(
        self,
        path: str | Path,
        *,
        seconds: float = 10.0,
        hz: float = 100.0,
    ) -> int:
        """在电机失能状态下录制机械臂和夹爪轨迹，并保存为 JSON 文件。

        用户可以在录制期间手动拖动机械臂。返回保存的轨迹点数量。
        """
        self._require_operational()
        if self._enabled:
            raise RuntimeError("disable the arm before recording a trajectory")
        duration = float(seconds)
        sample_hz = float(hz)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("seconds must be finite and positive")
        if not math.isfinite(sample_hz) or not 0.0 < sample_hz <= 500.0:
            raise ValueError("hz must be finite, positive, and at most 500")

        from ..service_tools.trajectory_recording import record, save_trajectory

        timestamps, positions = record(
            self,
            seconds=duration,
            hz=sample_hz,
        )
        save_trajectory(
            Path(path),
            sample_hz,
            positions,
            timestamps=timestamps,
            joint_names=self.joint_names,
        )
        return len(positions)

    def replay_trajectory(self, path: str | Path) -> int:
        """按照录制时间戳回放 JSON 轨迹，并返回发送的轨迹点数量。"""
        self._require_operational()
        if not self._enabled:
            raise RuntimeError("enable the arm before replaying a trajectory")

        from ..service_tools.trajectory_recording import load_trajectory, replay

        _, timestamps, positions = load_trajectory(
            Path(path),
            expected_joint_names=self.joint_names,
        )
        replay(self, timestamps=timestamps, positions=positions)
        return len(positions)

    def set_zero(
        self,
        *,
        joint_names: Sequence[str] | None = None,
        verify_tolerance: float = 0.02,
        verify_velocity: float = 0.05,
        verify_samples: int = 3,
    ) -> tuple[str, ...]:
        """将所选电机的当前位置写为零点，并通过反馈验证。

默认处理当前启用的全部电机；产品夹爪已启用时也会一起调零。更换末端并
禁用夹爪后，默认只处理机械臂关节。``joint_names`` 仅用于显式选择部分电机。
机械臂必须已连接、无故障且处于失能状态。验证要求连续 ``verify_samples``
        个新鲜样本均满足位置和速度容差。
        """
        self._require_operational()
        if self._enabled:
            raise RuntimeError("disable the arm before writing motor zero positions")
        return self.robot.set_zero(
            joint_names=list(joint_names) if joint_names is not None else None,
            verify_tolerance=verify_tolerance,
            verify_velocity=verify_velocity,
            verify_samples=verify_samples,
        )

    def set_gripper_opening(self, value: float) -> None:
        """使用简单的 ``0``～``1000`` 刻度设置夹爪开合度。

        ``0`` 表示完全闭合，``1000`` 表示完全张开，超出范围的值会自动限幅。
        夹爪始终使用机型配置中的 MIT 增益；Yunyi 默认 Kp 为 4.0、Kd 为 0.5。
        """
        self._require_connected()
        if self._dual_runtime_managed:
            raise RuntimeError(
                "this gripper is managed by ArxDCanDualArm; use "
                "set_gripper_openings()"
            )
        self._require_operational()
        if self.config.gripper is None:
            return
        if not self.enable_gripper:
            raise RuntimeError(
                "ARX-D-CAN gripper is disabled; create "
                "ArxDCanArm(enable_gripper=True)"
            )
        if not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        opening = float(value)
        if not math.isfinite(opening):
            raise ValueError("gripper opening must be finite")
        opening = max(0.0, min(1000.0, opening))
        runtime = self._single_safety_runtime
        if runtime is not None:
            motor = self.robot._motor_map[self.config.gripper.name]
            try:
                runtime.set_gripper_openings(((motor, opening),))
            finally:
                self._sync_native_safety_flags(runtime.health)
            return
        self.set_gripper_motor_value(
            self._gripper_motor_position_from_opening(opening)
        )

    def _gripper_motor_position_from_opening(self, opening: float) -> float:
        value = float(opening)
        if not math.isfinite(value):
            raise ValueError("gripper opening must be finite")
        ratio = max(0.0, min(1.0, value / 1000.0))
        return self.config.gripper_closed_value + (
            self.config.gripper_open_value - self.config.gripper_closed_value
        ) * ratio

    def _gripper_opening_from_motor_position(self, position: float) -> float:
        ratio = (
            float(position) - self.config.gripper_closed_value
        ) / (
            self.config.gripper_open_value - self.config.gripper_closed_value
        )
        return 1000.0 * max(0.0, min(1.0, ratio))

    def set_gripper_motor_value(self, value: float) -> None:
        """直接使用电机位置单位发送夹爪目标。

        此兼容方法供已经使用电机坐标的控制器调用。大多数用户应使用
        :meth:`set_gripper_opening`。
        目标会限制在配置的行程范围内，并通过 MIT 模式发送。
        """
        self._require_connected()
        if self._dual_runtime_managed:
            raise RuntimeError(
                "raw gripper commands are disabled while the C++ dual-arm runtime is active"
            )
        if self._safe_holding:
            return
        self._require_operational()
        if self.config.gripper is None:
            return
        if not self.enable_gripper:
            raise RuntimeError("ARX-D-CAN gripper is disabled; create ArxDCanArm(enable_gripper=True)")
        target = float(value)
        if not math.isfinite(target):
            raise ValueError("gripper motor value must be finite")
        lower = min(
            self.config.gripper_closed_value,
            self.config.gripper_open_value,
        )
        upper = max(
            self.config.gripper_closed_value,
            self.config.gripper_open_value,
        )
        target = min(max(target, lower), upper)
        runtime = self._single_safety_runtime
        if runtime is not None:
            self.set_gripper_opening(
                self._gripper_opening_from_motor_position(target)
            )
            return
        if not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        with self._io_lock:
            self._send_gripper_mit(target)

    def _set_dual_runtime_managed(self, managed: bool) -> None:
        """禁止绕过双臂原生状态机直接提交单侧关节命令。"""
        self._dual_runtime_managed = bool(managed)

    def _send_gripper_mit(
        self,
        position: float,
        kp: float | None = None,
        kd: float | None = None,
    ) -> None:
        self.robot.gripper.send_mit(
            np.array([position]),
            kp=None if kp is None else np.array([kp]),
            kd=None if kd is None else np.array([kd]),
            strict=True,
        )
