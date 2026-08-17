"""ARX-D-CAN 机械臂高层控制器。"""
from __future__ import annotations

import math
import subprocess
import sys
import threading
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Sequence

import numpy as np
from motor_drive_layer import (
    ArticoreRuntime,
    DisableReport,
    EnableReport,
    GripperCommand,
    GripperHealth,
    GripperProductBinding,
    JointControlConfig,
    JointPositionTarget,
    JointSafetyLimits,
    RuntimeConfig,
    RuntimeControlMode,
    RuntimeMitCommand,
    RuntimeMotor,
    RuntimePvCommand,
    RuntimeTransportHealth,
    SafetyHealth,
    SafetyState,
)

from ..actuator.arx_d_can import ArxDCan
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
from .gripper import GripperForceLevel
from .safety import _SafetyMixin
from .state import (
    ArxDCanState,
    GripperState,
    JointState,
)


MAX_ORDINARY_MIT_VELOCITY = math.radians(200.0)
RAW_MIT_RESULTANT_TORQUE_RATIO = 0.8


def _load_profile(config_path: str | Path | None, *, model: str | None) -> dict:
    """从包级入口解析配置加载器，保持现有 monkeypatch 钩子有效。"""
    sdk_package = sys.modules[__package__]
    return sdk_package.load_cfg(config_path, model=model)


@dataclass(slots=True, frozen=True)
class _PreparedJointPositionBatch:
    """一侧机械臂已完成验证和坐标换算的批量命令。"""

    mode: str
    commands: tuple[object, ...]


def _runtime_raw_commands(batch: _PreparedJointPositionBatch) -> tuple[object, ...]:
    """把 SDK 内部命令转换为 motor-drive-layer 的正式 Runtime 类型。"""
    if batch.mode == "pv":
        return tuple(
            RuntimePvCommand(
                motor=command.motor,
                position=float(command.pos),
                velocity_limit=float(command.vlim),
            )
            for command in batch.commands
        )
    return tuple(
        RuntimeMitCommand(
            motor=command.motor,
            position=float(command.pos),
            velocity=float(command.vel),
            kp=float(command.kp),
            kd=float(command.kd),
            feedforward_torque=float(command.tau),
        )
        for command in batch.commands
    )


class ArxDCanArm(_SafetyMixin):
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
        self._single_safety_runtime: ArticoreRuntime | None = None
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
    def _effective_control_hz(self) -> float:
        """返回 SDK 内部调度频率，不属于公开控制接口。"""
        runtime = self._single_safety_runtime
        return (
            self.config.control_hz
            if runtime is None
            else float(runtime.control_hz)
        )

    @property
    def enabled(self) -> bool:
        """返回 SDK 是否认为当前活动电机已使能。"""
        runtime = self._single_safety_runtime
        if runtime is not None:
            self._sync_runtime_flags(runtime.health)
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
            self._sync_runtime_flags(runtime.health)
        return self._faulted

    @property
    def fault_reason(self) -> str | None:
        """返回锁存的故障说明；状态正常时返回 ``None``。"""
        runtime = self._single_safety_runtime
        if runtime is not None:
            self._sync_runtime_flags(runtime.health)
        return self._fault_reason

    @property
    def safe_holding(self) -> bool:
        """返回看门狗是否正在重复发送最后一条安全命令。"""
        runtime = self._single_safety_runtime
        if runtime is not None:
            self._sync_runtime_flags(runtime.health)
        return self._safe_holding

    @property
    def safety_health(self) -> SafetyHealth:
        """返回 motor 原生安全状态机快照。"""
        runtime = self._single_safety_runtime
        if runtime is not None:
            health = runtime.health
            self._sync_runtime_flags(health)
            return health
        connected = self.connected
        state = (
            SafetyState.RUNNING
            if self.enabled
            else SafetyState.READY
            if connected
            else SafetyState.DISCONNECTED
        )
        transport = RuntimeTransportHealth(
            connected=connected,
            healthy=connected,
            consecutive_send_failures=0,
            consecutive_feedback_failures=0,
            last_feedback_age_ns=None,
            tx_frames=0,
            rx_frames=0,
            send_errors=0,
            receive_errors=0,
            last_tx_age_ns=None,
            last_rx_age_ns=None,
            last_error=None,
        )
        inactive = RuntimeTransportHealth(
            connected=False,
            healthy=False,
            consecutive_send_failures=0,
            consecutive_feedback_failures=0,
            last_feedback_age_ns=None,
            tx_frames=0,
            rx_frames=0,
            send_errors=0,
            receive_errors=0,
            last_tx_age_ns=None,
            last_rx_age_ns=None,
            last_error=None,
        )
        return SafetyHealth(
            state=state,
            safe_holding=self.safe_holding,
            disable_confirmed=not self.enabled,
            last_successful_command_age_ns=None,
            last_fresh_feedback_age_ns=None,
            consecutive_send_failures=0,
            consecutive_feedback_failures=0,
            left_transport=transport,
            right_transport=inactive,
            grippers=(),
            motor_faults=(),
            unconfirmed_disable=(),
            fault_reason=self.fault_reason,
        )

    @property
    def last_enable_report(self) -> EnableReport | None:
        """返回最近一次原子使能报告；未创建原生 Runtime 时返回 ``None``。"""
        runtime = self._single_safety_runtime
        return None if runtime is None else runtime.last_enable_report()

    @property
    def last_disable_report(self) -> DisableReport | None:
        """返回最近一次确定性失能报告；未创建原生 Runtime 时返回 ``None``。"""
        runtime = self._single_safety_runtime
        return None if runtime is None else runtime.last_disable_report()

    @property
    def gripper_safety_health(self) -> GripperHealth | None:
        """返回产品夹爪的原生防堵转状态；无原生夹爪时返回 ``None``。"""
        return next(
            (item for item in self.safety_health.grippers if item.side == 0),
            None,
        )

    @property
    def communication_health(self) -> RuntimeTransportHealth:
        """直接返回 motor 提供的结构化通信健康状态。"""
        return self.safety_health.left_transport

    def _runtime_motors(
        self,
        *,
        side: int = 0,
        label: str | None = None,
    ) -> tuple[RuntimeMotor, ...]:
        """生成供 Articore C++ runtime 使用的通用电机描述。"""
        prefix = "" if label is None else f"{label}/"
        descriptors = []
        for joint in self.config.arm_joints:
            descriptors.append(
                RuntimeMotor(
                    motor=self.robot._motor_map[joint.name],
                    side=side,
                    name=f"{prefix}{joint.name}",
                    safe_kp=self.config.safe_hold_mit_kp,
                    safe_kd=self.config.safe_hold_mit_kd,
                )
            )
        if self.enable_gripper and self.config.gripper is not None:
            gripper = self.config.gripper
            descriptors.append(
                RuntimeMotor(
                    motor=self.robot._motor_map[gripper.name],
                    side=side,
                    name=f"{prefix}{gripper.name}",
                    is_gripper=True,
                )
            )
        return tuple(descriptors)

    def _runtime_joint_configs(self) -> tuple[JointControlConfig, ...]:
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
                JointControlConfig(
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

    def _runtime_joint_limits(self) -> tuple[JointSafetyLimits, ...]:
        """由 URDF 硬限位和产品余量生成 Runtime 2.0 分层限位。"""
        margin = self.config.soft_limit_margin
        braking_zone = self.config.soft_limit_braking_zone
        braking_acceleration = self.config.braking_acceleration
        if any(
            not math.isfinite(value) or value < 0.0
            for value in (margin, braking_zone, braking_acceleration)
        ) or braking_acceleration == 0.0:
            raise ValueError("joint safety values must be finite and non-negative")
        limits = []
        for joint in self.config.arm_joints:
            position_range, _, _ = damiao_model_limits(joint.model)
            logical_hard_lower = (
                -position_range if joint.lower_limit is None else joint.lower_limit
            )
            logical_hard_upper = (
                position_range if joint.upper_limit is None else joint.upper_limit
            )
            logical_soft_lower = logical_hard_lower + margin
            logical_soft_upper = logical_hard_upper - margin
            if logical_soft_lower >= logical_soft_upper:
                raise ValueError(
                    f"{joint.name}: soft limit margin leaves no valid joint range"
                )
            soft_span = logical_soft_upper - logical_soft_lower
            if braking_zone > soft_span:
                raise ValueError(
                    f"{joint.name}: soft limit braking zone exceeds the soft range"
                )
            hard_motor = (
                joint.direction * logical_hard_lower,
                joint.direction * logical_hard_upper,
            )
            soft_motor = (
                joint.direction * logical_soft_lower,
                joint.direction * logical_soft_upper,
            )
            limits.append(
                JointSafetyLimits(
                    motor=self.robot._motor_map[joint.name],
                    hard_lower_position=min(hard_motor),
                    hard_upper_position=max(hard_motor),
                    soft_lower_position=min(soft_motor),
                    soft_upper_position=max(soft_motor),
                    soft_limit_braking_zone=braking_zone,
                    braking_acceleration=braking_acceleration,
                )
            )
        return tuple(limits)

    def _runtime_gripper_bindings(
        self,
    ) -> tuple[GripperProductBinding, ...]:
        """把实际安装的夹爪绑定到 motor 内置产品标定。"""
        gripper = self.config.gripper
        if not self.enable_gripper or gripper is None:
            return ()
        if not self.config.gripper_profile:
            raise ValueError(
                f"{self.config.model}: installed gripper requires gripper_profile"
            )
        return (
            GripperProductBinding(
                motor=self.robot._motor_map[gripper.name],
                profile_id=self.config.gripper_profile,
            ),
        )

    def _runtime_config(self) -> RuntimeConfig:
        return RuntimeConfig(
            control_hz=max(1, round(self.config.control_hz)),
            command_timeout_ms=max(1, round(self.config.command_timeout_s * 1000)),
            enable_grace_ms=max(1, round(self.config.enable_grace_s * 1000)),
            safe_hold_hz=max(1, round(self.config.safe_hold_hz)),
            feedback_check_hz=max(1, round(self.config.feedback_check_hz)),
            feedback_failure_threshold=self.config.feedback_fault_threshold,
            feedback_max_age_ms=max(
                1, round(self.config.max_cached_feedback_age_s * 1000)
            ),
            safe_hold_failure_threshold=self.config.safe_hold_failure_threshold,
            safe_pv_velocity_limit=self.config.safe_hold_pv_velocity_limit,
            gripper_control_hz=max(1, round(self.config.control_hz)),
        )

    def _create_single_safety_runtime(self) -> None:
        """在底层对象提供原生句柄时创建单通道常驻安全运行时。"""
        if self._dual_runtime_managed:
            return
        if not hasattr(self.robot.arm, "_controller_for_batch"):
            return
        controller = self._controller_for_parallel_batch()
        if not getattr(controller, "_ptr", None):
            return
        descriptors = self._runtime_motors()
        if not all(getattr(item.motor, "_ptr", None) for item in descriptors):
            return
        group = ControllerGroup([controller])
        if not getattr(group, "_ptr", None):
            group.close()
            return
        runtime: ArticoreRuntime | None = None
        try:
            runtime = ArticoreRuntime(
                config=self._runtime_config(),
                controller_group=group,
                left_controller=controller,
                right_controller=None,
                motors=descriptors,
            )
            runtime.configure_joints(self._runtime_joint_configs())
            runtime.configure_joint_safety_limits(self._runtime_joint_limits())
            runtime.configure_gripper_products(
                self._runtime_gripper_bindings()
            )
            runtime.connect()
        except Exception:
            if runtime is not None:
                try:
                    runtime.close()
                except Exception:
                    pass
            group.close()
            raise
        self._single_controller_group = group
        self._single_safety_runtime = runtime

    def _release_single_runtime(self) -> None:
        """按 Runtime → ControllerGroup 顺序释放单臂底层租用。"""
        error: Exception | None = None
        runtime = self._single_safety_runtime
        if runtime is not None:
            try:
                runtime.close()
            except Exception as exc:
                error = exc
            if getattr(runtime, "closed", True):
                self._single_safety_runtime = None
            else:
                raise error or RuntimeError("ArticoreRuntime did not close")
        group = self._single_controller_group
        if group is not None:
            group.close()
            self._single_controller_group = None
        if error is not None:
            raise error

    def _sync_runtime_flags(self, health: SafetyHealth) -> None:
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

        电机模式和设备通信参数会在创建 Runtime 前配置；连接不会使能电机。
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
        try:
            self._configure()
            self._create_single_safety_runtime()
            if not self._dual_runtime_managed and self._single_safety_runtime is None:
                raise RuntimeError(
                    "motor-drive-layer 0.10.7 ArticoreRuntime is unavailable"
                )
        except Exception:
            try:
                self.robot.disconnect(disable=False)
            finally:
                with self._state_lock:
                    self._connected = False
            raise

    def _configure(self) -> None:
        """在创建 Runtime 前配置控制模式和电机通信超时。"""
        self._require_operational()
        if self._enabled:
            raise RuntimeError(
                "cannot configure control mode while the arm is enabled; "
                "call disable() first"
            )
        try:
            self.configure_mode(self._mode)
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

    def close(self) -> None:
        """停止生成控制命令并关闭总线。

        motor-drive-layer 负责 Runtime 句柄和租用生命周期；SDK 只按
        Runtime → ControllerGroup → Controller 的顺序关闭。
        """
        errors: list[Exception] = []
        try:
            self._release_single_runtime()
        except Exception as exc:
            errors.append(exc)
        if self._connected:
            try:
                self.robot.disconnect(disable=False)
            except Exception as exc:
                errors.append(exc)
            else:
                self._connected = False
        if not self._connected:
            with self._state_lock:
                self._configured = False
                self._safe_holding = False
                self._enabled = False
        if errors:
            raise errors[0]

    def enable(self) -> None:
        """通过原生原子事务使能活动电机并启动命令安全监控。

        机械臂必须已连接，物理使能、当前位置保持、反馈确认和失败回滚均由
        motor-drive-layer 的正式 :class:`ArticoreRuntime` 完成。
        """
        self._require_operational()
        if not self._configured:
            raise RuntimeError("motor mode was not configured before Runtime creation")
        runtime = self._single_safety_runtime
        if runtime is None:
            raise RuntimeError("native safety runtime is not connected")
        try:
            runtime.enable(
                RuntimeControlMode.PV
                if self._mode in {"pv", "posvel"}
                else RuntimeControlMode.MIT
            )
        except Exception:
            try:
                self._sync_runtime_flags(runtime.health)
            except Exception:
                pass
            raise
        self._sync_runtime_flags(runtime.health)

    def disable(self) -> None:
        """停止后台控制，并向所有电机发送紧急失能命令。

        成功后会清除保留的机械臂命令。如果无法确认电机已在物理层失能，SDK 会继续
        保持故障状态并将电机报告为已使能，避免呈现不安全的虚假状态。
        """
        self._require_connected()
        runtime = self._single_safety_runtime
        if runtime is None:
            raise RuntimeError("native safety runtime is not connected")
        try:
            runtime.disable()
        except Exception as exc:
            try:
                health = runtime.health
            except Exception:
                with self._state_lock:
                    self._enabled = True
                    self._faulted = True
                    self._fault_reason = f"disable failed: {exc}"
                    self._safe_holding = False
            else:
                self._sync_runtime_flags(health)
            raise
        self._sync_runtime_flags(runtime.health)

    def recover(self) -> None:
        """确认反馈和物理失能后，将 Runtime 故障恢复到 ``READY``。

        除非从 ``SAFE_HOLD`` 状态开始恢复，否则会紧急失能电机。Runtime 保留连接
        前已经配置的电机模式，恢复过程中不绕过其租用重新配置底层对象。
        """
        self._require_connected()
        runtime = self._single_safety_runtime
        if runtime is None:
            raise RuntimeError("native safety runtime is not connected")
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
        self._configured = True
        self._sync_runtime_flags(runtime.health)

    def clear_motor_faults(self) -> tuple[str, ...]:
        """清除活动电机故障，并返回成功清除故障的电机名称。

        未连接时只临时打开 Controller 和 Motor，不配置控制模式或创建 Runtime；
        已连接时先释放 Runtime 租用，完成后按原产品配置重建 Runtime。
        """
        if self._dual_runtime_managed:
            raise RuntimeError(
                "this arm is managed by ArxDCanDualArm; clear both sides through "
                "the dual-arm maintenance API"
            )
        if not self._connected:
            return self._clear_motor_faults_maintenance()
        recreate_runtime = self._single_safety_runtime is not None
        if recreate_runtime:
            self._release_single_runtime()
        try:
            completed = self.robot.clear_errors(
                joint_names=self._active_joint_names(),
            )
        except Exception as exc:
            with self._state_lock:
                self._enabled = False
                self._configured = recreate_runtime
                self._faulted = True
                self._fault_reason = f"motor fault clear failed: {exc}"
                self._safe_holding = False
            raise
        finally:
            if recreate_runtime:
                self._create_single_safety_runtime()

        with self._state_lock:
            self._enabled = False
            self._configured = recreate_runtime
            self._faulted = False
            self._fault_reason = None
            self._safe_holding = False
        return completed

    def _clear_motor_faults_maintenance(self) -> tuple[str, ...]:
        """通过不配置控制模式的临时连接清除单臂电机故障。"""
        connected = False
        try:
            self.robot.connect()
            connected = True
            completed = self.robot.clear_errors(
                joint_names=self._active_joint_names(),
            )
        finally:
            if connected:
                self.robot.disconnect(disable=False)
        with self._state_lock:
            self._enabled = False
            self._configured = False
            self._faulted = False
            self._fault_reason = None
            self._safe_holding = False
        return completed

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
            normalized = "posvel"
        elif normalized != "mit":
            raise ValueError("mode must be 'posvel' or 'mit'")

        recreate_runtime = self._single_safety_runtime is not None
        if recreate_runtime:
            self._release_single_runtime()
        with self._io_lock:
            configured = (
                self.robot.arm.mode_pos_vel()
                if normalized == "posvel"
                else self.robot.arm.mode_mit()
            )
        if not configured:
            raise RuntimeError(f"ARX-D-CAN arm did not enter {normalized.upper()} mode")
        self._mode = normalized
        if recreate_runtime:
            self._configured = True
            self._create_single_safety_runtime()

    def read_state(self) -> ArxDCanState:
        """返回 Runtime 后台持续刷新的完整机械臂及夹爪状态。

        Runtime 创建后独占主动反馈请求；SDK 只读取电机缓存，新鲜度与通信错误由
        :attr:`safety_health` 统一报告。
        """
        return self.read_cached_state()

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
        arm_pos = np.asarray(arm_pos, dtype=np.float64)
        arm_vel = np.asarray(arm_vel, dtype=np.float64)
        arm_tau = np.asarray(arm_tau, dtype=np.float64)
        gripper_state = None
        if self.config.gripper is not None and len(pos) > arm_count:
            motor_position = float(pos[arm_count])
            runtime = self._single_safety_runtime
            health = None if runtime is None else next(
                (item for item in runtime.health.grippers if item.side == 0),
                None,
            )
            gripper_state = GripperState(
                name=self.config.gripper.name,
                motor_id=self.config.gripper.motor_id,
                feedback_id=self.config.gripper.feedback_id,
                opening=0.0 if health is None else health.opening,
                motor_position=motor_position,
                motor_velocity=float(vel[arm_count]),
                torque=float(tau[arm_count]),
            )
        return ArxDCanState(
            arm=JointState(
                names=self.config.joint_names,
                positions=tuple(float(value) for value in arm_pos),
                velocities=tuple(float(value) for value in arm_vel),
                torques=tuple(float(value) for value in arm_tau),
            ),
            gripper=gripper_state,
        )

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
        enforce_limits: bool = True,
    ) -> tuple[float, ...]:
        """校验逻辑关节目标，并按需执行 URDF 位置限位检查。"""
        values = tuple(float(value) for value in positions)
        joint_count = len(self.config.arm_joints)
        prefix = "" if name is None else f"{name} "
        if len(values) != joint_count:
            raise ValueError(
                f"{prefix}expected {joint_count} joint positions, got {len(values)}"
            )
        if any(not math.isfinite(value) for value in values):
            raise ValueError(f"{prefix}joint positions must be finite")

        if not enforce_limits:
            return values

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
        require_enabled: bool = True,
        enforce_position_limits: bool = True,
    ) -> _PreparedJointPositionBatch:
        """校验一侧关节目标，并生成可交给 ControllerGroup 的命令。"""
        self._require_connected()
        self._require_operational()
        if require_enabled and not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        joint_count = len(self.config.arm_joints)
        values = self._validated_joint_positions(
            positions,
            enforce_limits=enforce_position_limits,
        )
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
                velocity_limit_target = np.full(
                    joint_count,
                    min(1.0, *(joint.pv_vlim for joint in self.config.arm_joints)),
                )
            if any(
                value is not None
                for value in (velocity_target, torque_target, kp_target, kd_target)
            ):
                raise ValueError(
                    "velocities, torques and MIT gains are only supported in MIT mode"
                )
            commands = self.robot.arm._make_pos_vel_batch_commands(
                logical_position,
                vlim=velocity_limit_target,
            )
            return _PreparedJointPositionBatch(
                mode="pv",
                commands=commands,
            )

        if self._mode == "mit":
            if velocity_limit_target is not None:
                raise ValueError("velocity limits are only supported in PV mode")
            commands = self.robot.arm._make_mit_batch_commands(
                logical_position,
                vel=(
                    np.zeros(joint_count)
                    if velocity_target is None
                    else velocity_target
                ),
                kp=(
                    np.asarray([joint.mit_kp for joint in self.config.arm_joints])
                    if kp_target is None
                    else kp_target
                ),
                kd=(
                    np.asarray([joint.mit_kd for joint in self.config.arm_joints])
                    if kd_target is None
                    else kd_target
                ),
                tau=np.zeros(joint_count) if torque_target is None else torque_target,
            )
            return _PreparedJointPositionBatch(
                mode="mit",
                commands=commands,
            )

        raise ValueError("mode must be 'posvel' or 'mit'")

    def _limit_raw_mit_resultant_torque(
        self,
        *,
        positions: Sequence[float],
        velocities: Sequence[float] | None,
        kp: float | Sequence[float] | None,
        kd: float | Sequence[float] | None,
        feedforward_torques: Sequence[float] | None,
        current_positions: Sequence[float],
        current_velocities: Sequence[float],
    ) -> tuple[
        tuple[float, ...],
        tuple[float, ...],
        tuple[float, ...],
        tuple[float, ...],
    ]:
        """按最新反馈把公开 raw MIT 的合成力矩缩放到 URDF effort 的 80%。"""
        joint_count = len(self.config.arm_joints)

        def vector(
            values: Sequence[float] | None,
            *,
            name: str,
            default: Sequence[float],
        ) -> np.ndarray:
            result = np.asarray(default if values is None else values, dtype=np.float64)
            result = result.reshape(-1)
            if len(result) != joint_count:
                raise ValueError(
                    f"expected {joint_count} joint {name}, got {len(result)}"
                )
            if not np.all(np.isfinite(result)):
                raise ValueError(f"joint {name} must be finite")
            return result

        target_positions = vector(
            positions,
            name="positions",
            default=(),
        )
        target_velocities = vector(
            velocities,
            name="velocities",
            default=np.zeros(joint_count),
        )
        actual_positions = vector(
            current_positions,
            name="feedback positions",
            default=(),
        )
        actual_velocities = vector(
            current_velocities,
            name="feedback velocities",
            default=(),
        )
        stiffness = _mit_gain_vector(kp, joint_count=joint_count, name="Kp")
        if stiffness is None:
            stiffness = np.asarray(
                [joint.mit_kp for joint in self.config.arm_joints],
                dtype=np.float64,
            )
        damping = _mit_gain_vector(kd, joint_count=joint_count, name="Kd")
        if damping is None:
            damping = np.asarray(
                [joint.mit_kd for joint in self.config.arm_joints],
                dtype=np.float64,
            )
        feedforward = vector(
            feedforward_torques,
            name="feedforward torques",
            default=np.zeros(joint_count),
        )

        proportional = np.zeros(joint_count, dtype=np.float64)
        derivative = np.zeros(joint_count, dtype=np.float64)
        limits = np.zeros(joint_count, dtype=np.float64)
        feedforward_limits = np.zeros(joint_count, dtype=np.float64)
        for index, joint in enumerate(self.config.arm_joints):
            _, native_velocity, native_torque = damiao_model_limits(joint.model)
            configured_velocity = (
                native_velocity
                if joint.velocity_range is None
                else joint.velocity_range
            )
            configured_torque = (
                native_torque
                if joint.torque_range is None
                else joint.torque_range
            )
            velocity_command_scale = native_velocity / configured_velocity
            torque_feedback_scale = configured_torque / native_torque
            proportional[index] = (
                torque_feedback_scale
                * stiffness[index]
                * (target_positions[index] - actual_positions[index])
            )
            derivative[index] = (
                torque_feedback_scale
                * damping[index]
                * velocity_command_scale
                * (target_velocities[index] - actual_velocities[index])
            )
            logical_limit = (
                configured_torque
                if joint.effort_limit is None
                else joint.effort_limit
            )
            feedforward_limits[index] = logical_limit
            limits[index] = RAW_MIT_RESULTANT_TORQUE_RATIO * logical_limit

        # 保持后续协议映射的输入范围一致，避免底层再次裁剪 tau_ff 后改变这里估算的
        # 合成结果；安全比例仍施加在下面的完整 P + D + FF 上。
        feedforward = np.clip(
            feedforward,
            -feedforward_limits,
            feedforward_limits,
        )
        requested = proportional + derivative + feedforward
        scale = np.ones(joint_count, dtype=np.float64)
        saturated = np.abs(requested) > limits
        scale[saturated] = limits[saturated] / np.abs(requested[saturated])
        return (
            tuple(float(value) for value in target_velocities),
            tuple(float(value) for value in stiffness * scale),
            tuple(float(value) for value in damping * scale),
            tuple(float(value) for value in feedforward * scale),
        )

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
        enforce_position_limits: bool = True,
    ) -> None:
        """供 SDK 内部高级控制器提交一帧原始 PV/MIT 命令。"""
        if self._dual_runtime_managed:
            raise RuntimeError(
                "this arm is managed by ArxDCanDualArm; submit a complete left/right "
                "command through ArxDCanDualArm"
            )
        runtime = self._single_safety_runtime
        if runtime is None:
            raise RuntimeError("native safety runtime is not connected")
        batch = self._prepare_parallel_joint_positions(
            positions,
            velocities=velocities,
            velocity_limits=velocity_limits,
            torques=torques,
            mit_kp=mit_kp,
            mit_kd=mit_kd,
            require_enabled=require_enabled,
            enforce_position_limits=enforce_position_limits,
        )
        try:
            (runtime.submit_pv if batch.mode == "pv" else runtime.submit_mit)(
                _runtime_raw_commands(batch)
            )
        finally:
            self._sync_runtime_flags(runtime.health)

    def _ordinary_joint_position_targets(
        self,
        positions: Sequence[float],
    ) -> tuple[JointPositionTarget, ...]:
        self._require_connected()
        self._require_operational()
        if not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        values = self._validated_joint_positions(positions)
        return tuple(
            JointPositionTarget(
                motor=self.robot._motor_map[joint.name],
                position=joint.direction * float(position),
            )
            for joint, position in zip(self.config.arm_joints, values)
        )

    def _ordinary_joint_velocity(self, velocity: float, *, mode: str) -> float:
        value = float(velocity)
        hardware_maximum = min(
            joint.pv_vlim for joint in self.config.arm_joints
        )
        maximum = (
            min(hardware_maximum, MAX_ORDINARY_MIT_VELOCITY)
            if mode == "mit"
            else hardware_maximum
        )
        if not math.isfinite(value) or not 0.0 < value <= maximum:
            limit = (
                f"{maximum:g} rad/s (200 deg/s)"
                if mode == "mit" and maximum == MAX_ORDINARY_MIT_VELOCITY
                else f"{maximum:g} rad/s"
            )
            raise ValueError(
                f"velocity must be finite, positive, and at most {limit}"
            )
        return value

    def _set_joint_position(
        self,
        positions: Sequence[float],
        *,
        velocity: float,
        mode: str,
    ) -> None:
        if self._dual_runtime_managed:
            raise RuntimeError(
                "this arm is managed by ArxDCanDualArm; submit a complete left/right "
                f"set_joint_{mode}() command"
            )
        if self._mode not in ({"pv", "posvel"} if mode == "pv" else {"mit"}):
            raise RuntimeError(f"set_joint_{mode}() requires {mode.upper()} mode")
        runtime = self._single_safety_runtime
        if runtime is None:
            raise RuntimeError("native joint position runtime is not connected")
        targets = self._ordinary_joint_position_targets(positions)
        reference_velocity = self._ordinary_joint_velocity(velocity, mode=mode)
        try:
            getattr(runtime, f"set_joint_{mode}")(targets, reference_velocity)
        finally:
            self._sync_runtime_flags(runtime.health)

    def set_joint_mit(
        self,
        positions: Sequence[float],
        *,
        velocity: float = 1.0,
    ) -> None:
        """以统一速度设置普通 MIT 最终位置；中间 reference 由 Runtime 生成。"""
        self._set_joint_position(positions, velocity=velocity, mode="mit")

    def set_joint_pv(
        self,
        positions: Sequence[float],
        *,
        velocity: float = 1.0,
    ) -> None:
        """以统一速度设置普通 PV 最终位置；中间 reference 由 Runtime 生成。"""
        self._set_joint_position(positions, velocity=velocity, mode="pv")

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
        if self._dual_runtime_managed:
            raise RuntimeError(
                "this arm is managed by ArxDCanDualArm; use robot.set_zero()"
            )
        recreate_runtime = self._single_safety_runtime is not None
        if recreate_runtime:
            self._release_single_runtime()
        try:
            return self.robot.set_zero(
                joint_names=list(joint_names) if joint_names is not None else None,
                verify_tolerance=verify_tolerance,
                verify_velocity=verify_velocity,
                verify_samples=verify_samples,
            )
        finally:
            if recreate_runtime:
                self._create_single_safety_runtime()

    def set_gripper_opening(
        self,
        value: float,
        *,
        speed: float = 1000.0,
        force_level: GripperForceLevel = GripperForceLevel.LEVEL_5,
    ) -> None:
        """使用简单的 ``0``～``1000`` 刻度设置夹爪开合度。

        ``0`` 表示完全闭合，``1000`` 表示完全张开，超出范围的值会自动限幅。
        ``speed`` 使用 ``(0, 1000]`` 产品归一化刻度；``1000`` 对应配置中的最大
        标定速度。``force_level`` 必须使用 :class:`GripperForceLevel` 枚举，具体
        接触、过载和保持参数由产品配置固定，普通用户不能逐项覆盖。
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
        normalized_speed = float(speed)
        if not math.isfinite(normalized_speed) or not 0.0 < normalized_speed <= 1000.0:
            raise ValueError("gripper speed must be finite and in (0, 1000]")
        level = GripperForceLevel(force_level)
        runtime = self._single_safety_runtime
        if runtime is None:
            raise RuntimeError("native safety runtime is not connected")
        motor = self.robot._motor_map[self.config.gripper.name]
        try:
            runtime.set_grippers(
                (
                    GripperCommand(
                        motor=motor,
                        opening=opening,
                        speed=normalized_speed,
                        force_level=int(level),
                    ),
                )
            )
        finally:
            self._sync_runtime_flags(runtime.health)

    def _set_dual_runtime_managed(self, managed: bool) -> None:
        """禁止绕过双臂原生状态机直接提交单侧关节命令。"""
        self._dual_runtime_managed = bool(managed)
