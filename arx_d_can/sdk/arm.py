"""ARX-D-CAN 机械臂高层控制器。"""
from __future__ import annotations

import math
import subprocess
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Sequence

import numpy as np

from ..actuator import ArxDCan
from ..driver import build_scan_command, parse_scan_ids
from ..errors import (
    CommunicationError,
    MotorFaultError,
    StaleFeedbackError,
    UnexpectedMotorStateError,
)
from ..kinematics.coupled_joint_transform import CoupledJointTransform
from .config import (
    ArxDCanConfig,
    _actuator_config_from_sdk,
    _config_from_loaded,
    _connection_channel,
    _mit_gain_vector,
)
from .coupled_control import _CoupledControlMixin
from .gripper_force_control import GripperForceController
from .safety import _SafetyMixin
from .state import (
    ArxDCanState,
    CommunicationHealth,
    CoupledControlStats,
    CoupledTorqueSaturation,
    CoupledTorqueTelemetry,
    GripperState,
    JointState,
    MitCommand,
)


def _load_profile(config_path: str | Path | None, *, model: str | None) -> dict:
    """从包级入口解析配置加载器，保持现有 monkeypatch 钩子有效。"""
    sdk_package = sys.modules[__package__]
    return sdk_package.load_cfg(config_path, model=model)


class ArxDCanArm(_SafetyMixin, _CoupledControlMixin):
    """通过 dm-serial 或 Linux SocketCAN 控制 ARX 机械臂的高层 SDK。"""

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
        self._joint_transform = (
            None
            if self.config.joint_transform_path is None
            else CoupledJointTransform.load(
                self.config.joint_transform_path,
                joint_names=self.config.joint_names,
            )
        )
        if self._joint_transform is not None:
            transformed_indices = self._joint_transform.transformed_indices
            loaded_config = dict(loaded_config)
            loaded_config["joints"] = [
                replace(
                    joint,
                    kp=0.0,
                    kd=0.0,
                    lower_limit=None,
                    upper_limit=None,
                )
                if index in transformed_indices
                else joint
                for index, joint in enumerate(loaded_config["joints"])
            ]
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
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._watchdog_deadline: float | None = None
        self._feedback_error_count = 0
        self._last_communication_error: CommunicationError | None = None
        self._using_fallback_state = False
        self._last_fresh_feedback_at: float | None = None
        self._last_joint_command: tuple[float, ...] | None = None
        self._last_mit_command: MitCommand | None = None
        self._last_gripper_command: float | None = None
        self._last_state: ArxDCanState | None = None
        self._coupled_control_stop = threading.Event()
        self._coupled_control_wakeup = threading.Event()
        self._coupled_control_thread: threading.Thread | None = None
        self._coupled_torque_saturation = CoupledTorqueSaturation(
            active=False,
            motor_names=(),
            requested_torques=(),
            limited_torques=(),
            applied_torques=(),
            saturation_scale=1.0,
            timestamp=time.monotonic(),
        )
        self._coupled_torque_telemetry = CoupledTorqueTelemetry(
            motor_names=(),
            motor_positions=(),
            motor_velocities=(),
            transformed_torques=(),
            motor_kd_gains=(),
            damping_torques=(),
            requested_torques=(),
            limited_torques=(),
            applied_torques=(),
            estimated_total_torques=(),
            saturation_scale=1.0,
            timestamp=time.monotonic(),
        )
        self._coupled_control_stats = CoupledControlStats(
            target_hz=self.config.control_hz,
            achieved_hz=0.0,
            cycle_count=0,
            overrun_count=0,
            feedback_stall_cycles=0,
            stale_feedback_faults=0,
            maximum_feedback_age_s=0.0,
            torque_command_count=0,
            torque_saturation_count=0,
        )
        self._coupled_feedback_update_counts: dict[str, int] = {}
        self._coupled_previous_motor_tau = np.zeros(
            len(self.config.arm_joints),
            dtype=np.float64,
        )
        self._coupled_filtered_virtual_velocity = np.zeros(
            len(self.config.arm_joints),
            dtype=np.float64,
        )
        self._coupled_velocity_filter_initialized = np.zeros(
            len(self.config.arm_joints),
            dtype=bool,
        )
        self._coupled_hold_candidate_since: dict[tuple[int, int], float] = {}
        self._mode = self.config.arm_control_mode.strip().lower().replace("_", "")
        self._gripper_command_lock = threading.RLock()
        self._gripper_force_controller: GripperForceController | None = None
        if (
            self.enable_gripper
            and self.config.gripper is not None
            and self.config.gripper_force_control_enabled
        ):
            self._gripper_force_controller = GripperForceController(
                self.config.gripper_force_control,
                open_value=self.config.gripper_open_value,
                closed_value=self.config.gripper_closed_value,
                normal_kp=self.config.gripper.mit_kp,
                normal_kd=self.config.gripper.mit_kd,
            )

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
        return self._enabled

    @property
    def has_gripper(self) -> bool:
        """返回当前控制器是否包含配置的夹爪。"""
        return self.enable_gripper and self.config.gripper is not None

    @property
    def faulted(self) -> bool:
        """返回故障是否已锁存，以及普通命令是否已被阻止。"""
        return self._faulted

    @property
    def fault_reason(self) -> str | None:
        """返回锁存的故障说明；状态正常时返回 ``None``。"""
        return self._fault_reason

    @property
    def safe_holding(self) -> bool:
        """返回看门狗是否正在重复发送最后一条安全命令。"""
        return self._safe_holding

    @property
    def coupled_torque_saturation(self) -> CoupledTorqueSaturation:
        """返回最近一次耦合电机力矩饱和状态。"""
        with self._state_lock:
            return self._coupled_torque_saturation

    @property
    def coupled_torque_telemetry(self) -> CoupledTorqueTelemetry:
        """返回最新的耦合电机反馈和各阶段力矩命令。"""
        with self._state_lock:
            return self._coupled_torque_telemetry

    @property
    def coupled_control_stats(self) -> CoupledControlStats:
        """返回耦合控制循环的实测时序和缓存反馈健康状态。"""
        with self._state_lock:
            return self._coupled_control_stats

    @property
    def communication_health(self) -> CommunicationHealth:
        """返回反馈回退状态和最近一次通信错误详情。"""
        with self._state_lock:
            last_fresh_feedback_at = self._last_fresh_feedback_at
            return CommunicationHealth(
                consecutive_feedback_failures=self._feedback_error_count,
                has_fresh_feedback=last_fresh_feedback_at is not None,
                using_fallback_state=self._using_fallback_state,
                last_error=self._last_communication_error,
                last_fresh_feedback_age_s=(
                    None
                    if last_fresh_feedback_at is None
                    else max(0.0, time.monotonic() - last_fresh_feedback_at)
                ),
            )

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
            self._feedback_error_count = 0
            self._last_communication_error = None
            self._using_fallback_state = False
            self._last_fresh_feedback_at = None
            self._last_joint_command = None
            self._last_mit_command = None
            self._last_gripper_command = None
            self._last_state = None

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
        except Exception as exc:
            self._trip_fault(f"configuration failed: {exc}")
            raise
        self._configured = True

    def close(self, *, disable: bool = True) -> None:
        """停止生成控制命令并关闭总线。

        默认先失能所有电机。``disable=False`` 仅供从未配置、使能或控制机器人的
        只读客户端使用，以避免退出时发送电机控制帧。
        """
        self._stop_coupled_control()
        self._stop_watchdog()
        error: Exception | None = None
        if self._connected:
            try:
                if disable:
                    self.robot.disconnect()
                else:
                    self.robot.disconnect(disable=False)
            except Exception as exc:
                error = exc
        with self._state_lock:
            self._connected = False
            self._configured = False
            self._safe_holding = False
            self._watchdog_deadline = None
            if error is None:
                self._enabled = False
            else:
                # 总线虽已关闭，但尚未确认电机已在物理层失能。
                self._enabled = True
                self._faulted = True
                self._fault_reason = f"close failed: {error}"
        if error is not None:
            raise RuntimeError(f"ARX-D-CAN close failed: {error}") from error

    def enable(
        self,
        *,
        initial_positions: Sequence[float] | None = None,
        initial_velocities: Sequence[float] | None = None,
        initial_torques: Sequence[float] | None = None,
        mit_kp: float | Sequence[float] | None = None,
        mit_kd: float | Sequence[float] | None = None,
    ) -> None:
        """使能活动电机并启动命令安全监控。

        机械臂必须已连接；首次使能时会自动配置构造机械臂时选择的控制模式。在 MIT
        模式下，可在使能时发送一条完整的初始命令，避免电机短暂采用无关目标。初始
        向量顺序与 :attr:`joint_names` 一致；未提供的速度和力矩默认为零，未提供的
        增益使用机型配置。操作失败会锁存故障，并尝试失能机器人。
        """
        self._require_operational()
        if not self._configured:
            self.configure()
        if initial_positions is not None and self._mode != "mit":
            raise ValueError("initial position seeding is only supported in MIT mode")
        self._reset_coupled_motor_torque_state()
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
            if self._joint_transform is None:
                initial_position_vector = np.asarray(initial_command.positions)
                initial_velocity_vector = np.asarray(initial_command.velocities)
                initial_torque_vector = np.asarray(initial_command.feedforward_torques)
            else:
                # 使能时不得把虚拟关节增益直接写入耦合电机。在内环取得已使能的新鲜
                # 反馈前，只施加变换后的前馈力矩，不从拟合正逆模型的残差推断 PD 误差。
                initial_position_vector = (
                    self._joint_transform.virtual_positions_to_motor(
                        initial_command.positions
                    )
                )
                initial_velocity_vector = (
                    self._joint_transform.virtual_velocities_to_motor(
                        initial_command.positions,
                        initial_command.velocities,
                    )
                )
                initial_torque_vector = (
                    self._joint_transform.virtual_torques_to_motor(
                        initial_command.positions,
                        initial_command.feedforward_torques,
                    )
                )
                transformed_indices = sorted(
                    self._joint_transform.transformed_indices
                )
                initial_velocity_vector[transformed_indices] = 0.0
                kp_vector[transformed_indices] = 0.0
                kd_vector[transformed_indices] = np.asarray(
                    [
                        self.config.arm_joints[index].coupled_motor_kd
                        for index in transformed_indices
                    ],
                    dtype=np.float64,
                )
                initial_torque_vector = self._limit_coupled_motor_torques(
                    initial_torque_vector
                )
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
            with self._gripper_command_lock:
                if self.enable_gripper and self.config.gripper is not None:
                    self.robot.gripper.enable()
                    if self._gripper_force_controller is not None:
                        self._gripper_force_controller.reset()
        except Exception as exc:
            self._trip_fault(f"enable failed: {exc}")
            raise
        with self._state_lock:
            self._enabled = True
            if initial_positions is not None:
                self._last_joint_command = tuple(float(value) for value in initial_positions)
                self._last_mit_command = initial_command
            self._watchdog_deadline = time.monotonic() + max(
                self.config.enable_grace_s,
                self.config.command_timeout_s,
            )
        self._start_watchdog()
        self._start_coupled_control()

    def disable(self) -> None:
        """停止后台控制，并向所有电机发送紧急失能命令。

        成功后会清除保留的机械臂命令。如果无法确认电机已在物理层失能，SDK 会继续
        保持故障状态并将电机报告为已使能，避免呈现不安全的虚假状态。
        """
        self._require_connected()
        self._stop_coupled_control()
        self._stop_watchdog()
        try:
            self.robot.estop()
        except Exception as exc:
            with self._gripper_command_lock:
                if self._gripper_force_controller is not None:
                    self._gripper_force_controller.reset()
            with self._state_lock:
                # 尚未确认物理失能，因此保留保守的软件状态，不假定电机已经安全。
                self._enabled = True
                self._faulted = True
                self._fault_reason = f"disable failed: {exc}"
                self._safe_holding = False
                self._watchdog_deadline = None
            raise
        with self._gripper_command_lock:
            if self._gripper_force_controller is not None:
                self._gripper_force_controller.reset()
        with self._state_lock:
            self._enabled = False
            self._safe_holding = False
            self._watchdog_deadline = None
            self._last_joint_command = None
            self._last_mit_command = None
        self._reset_coupled_motor_torque_state()

    def clear_fault(self) -> None:
        """确认反馈可用后，清除 SDK 的故障锁存。

        除非从 ``SAFE_HOLD`` 状态开始恢复，否则会紧急失能电机。从安全保持恢复时
        保留当前控制模式；其他情况由下一次 :meth:`enable` 自动重新配置并使能。
        """
        self._require_connected()
        was_safe_holding = self._safe_holding
        self._stop_watchdog()
        if not was_safe_holding:
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
            self._enabled = was_safe_holding
            self._configured = was_safe_holding
            self._feedback_error_count = 0
            self._last_communication_error = None
            self._using_fallback_state = False
            self._watchdog_deadline = None

    def clear_motor_faults(self) -> tuple[str, ...]:
        """清除活动电机故障，并返回成功清除故障的电机名称。

        此操作覆盖已配置的机械臂和可选的活动夹爪，完成后 SDK 始终处于未配置、
        未使能状态。清除失败仍会作为 SDK 故障锁存。
        """
        self._require_connected()
        self._stop_watchdog()
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
                self._feedback_error_count = 0
                self._last_communication_error = None
                self._using_fallback_state = False
                self._watchdog_deadline = None
            raise

        with self._state_lock:
            self._enabled = False
            self._configured = False
            self._faulted = False
            self._fault_reason = None
            self._safe_holding = False
            self._feedback_error_count = 0
            self._last_communication_error = None
            self._using_fallback_state = False
            self._watchdog_deadline = None
        return completed

    def recover(self) -> None:
        """清除锁存故障，并安全地恢复使能状态。

        已失能时，:meth:`enable` 会自动恢复配置的控制模式；从 ``SAFE_HOLD`` 恢复时
        保留原模式，避免在电机使能期间重新写入模式。如果恢复失败，机械臂会保持
        故障状态，不会进入部分恢复状态。
        """
        self.clear_fault()
        try:
            self.enable()
        except Exception:
            if not self._faulted:
                self._trip_fault("fault recovery failed")
            raise

    def configure_mode(self, mode: str = "posvel") -> None:
        """在 ``posvel``（PV）与 ``mit`` 两种控制模式之间切换机械臂组。

        总线必须已连接、没有锁存故障且电机已经失能。切换到 POS_VEL 前，会先停止
        耦合 MIT 工作线程，再修改电机模式。
        """
        self._require_operational()
        if self._enabled:
            raise RuntimeError(
                "cannot switch control mode while the arm is enabled; "
                "call disable() first"
            )
        normalized = mode.strip().lower().replace("_", "")
        if normalized in ("posvel", "pv"):
            self._stop_coupled_control()
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
        return self._read_fresh_state(
            serialize_io=True,
            allow_fallback=False,
        )

    def read_cached_state(self) -> ArxDCanState:
        """返回最近一次成功读取的状态，不发送任何通信帧。

        缓存不存在时抛出异常。数据年龄及最近通信错误可通过
        :attr:`communication_health` 查看。
        """
        self._require_connected()
        with self._state_lock:
            state = self._last_state
        if state is None:
            raise RuntimeError("ARX-D-CAN has no cached state; call read_state() first")
        return state

    def refresh_feedback_background(self) -> ArxDCanState:
        """在不占用 SDK 命令锁的情况下请求完整反馈。

        此方法供专用的低频监控线程使用。motor-drive-layer 串口传输与节拍发送总线会
        自行串行化 I/O，因此这里的反馈超时不会通过 ``_io_lock`` 阻塞主命令循环。
        """
        return self._read_fresh_state(
            serialize_io=False,
            allow_fallback=True,
        )

    def _read_fresh_state(
        self,
        *,
        serialize_io: bool,
        allow_fallback: bool,
    ) -> ArxDCanState:
        """执行新鲜状态采集、内部回退处理和故障状态转换。

        ``serialize_io`` 决定读取过程是否由 SDK 命令锁保护；由于通信层会自行串行化
        I/O，后台监控可关闭此保护。``allow_fallback`` 只供内部后台监控使用。
        新鲜读取会更新健康计数；连续失败或电机状态异常可能进入安全保持或触发硬故障。
        """
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
        except CommunicationError as exc:
            self._record_communication_error(
                exc,
                using_fallback=allow_fallback,
            )
            with self._state_lock:
                self._feedback_error_count += 1
                feedback_error_count = self._feedback_error_count
            if self._enabled and feedback_error_count >= max(
                1, self.config.feedback_fault_threshold
            ):
                self._begin_safe_hold(
                    f"feedback failed {feedback_error_count} consecutive times: "
                    f"{exc}"
                )
            with self._state_lock:
                last_state = self._last_state
            if allow_fallback and self._enabled and last_state is not None:
                return last_state
            raise
        except MotorFaultError as exc:
            self._trip_fault(str(exc))
            raise
        except Exception as exc:
            if self._enabled:
                self._trip_fault(f"unexpected state-read failure: {exc}")
            raise
        with self._state_lock:
            self._feedback_error_count = 0
            self._last_communication_error = None
            self._using_fallback_state = False
            self._last_fresh_feedback_at = time.monotonic()
        disabled_motors = [
            name for name, status in status_codes.items() if status == 0
        ]
        if self._enabled and disabled_motors:
            error = UnexpectedMotorStateError(
                "motors unexpectedly disabled after feedback recovery: "
                + ", ".join(disabled_motors),
                status_codes={name: status_codes[name] for name in disabled_motors},
            )
            if not self._safe_holding:
                self._begin_safe_hold(str(error))
            else:
                with self._state_lock:
                    self._fault_reason = (
                        f"{error}; holding last successful command"
                    )
        elif self._safe_holding:
            self._resume_from_safe_hold()
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
        """读取所有活动电机的状态、控制模式和温度，不发送控制命令。"""
        from ..diagnostics import read_motor_diagnostics

        return read_motor_diagnostics(self, timeout_ms=timeout_ms)

    def send_joint_positions(
        self,
        positions: Sequence[float],
        *,
        velocities: Sequence[float] | None = None,
        velocity_limits: Sequence[float] | None = None,
        torques: Sequence[float] | None = None,
        mit_kp: float | Sequence[float] | None = None,
        mit_kd: float | Sequence[float] | None = None,
        require_enabled: bool = True,
    ) -> None:
        """校验并发送一条完整的逻辑关节命令。

        ``positions`` 始终按 :attr:`joint_names` 排列。POS_VEL 接受逐关节
        ``velocity_limits``；MIT 接受目标 ``velocities``、前馈 ``torques`` 以及
        当前数据包使用的 Kp/Kd。发送格式由构造机械臂时的 ``control_mode`` 决定，
        此方法不会切换控制模式。未提供的 MIT 增益使用机型配置。成功发送会刷新
        看门狗；发送失败且已有安全目标时进入安全保持，否则将异常传递给调用者。
        """
        self._require_connected()
        if self._safe_holding:
            return
        self._require_operational()
        if require_enabled and not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        joint_count = len(self.config.arm_joints)
        if len(positions) != joint_count:
            raise ValueError(
                f"expected {joint_count} joint positions, got {len(positions)}"
            )
        if any(not math.isfinite(float(value)) for value in positions):
            raise ValueError("joint positions must be finite")
        target = {
            joint.name: float(value)
            for joint, value in zip(self.config.arm_joints, positions)
        }
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
        active_mode = self._mode
        if active_mode in ("posvel", "pv"):
            if velocity_target is not None:
                raise ValueError("target velocities are only supported in MIT mode")
            if torque_target is not None:
                raise ValueError("torques are only supported in MIT mode")
            if mit_kp_target is not None or mit_kd_target is not None:
                raise ValueError("MIT Kp/Kd are only supported in MIT mode")
        if active_mode == "mit" and velocity_limit_target is not None:
            raise ValueError("velocity limits are only supported in PV mode")
        logical_position_target = np.array(
            [target[joint.name] for joint in self.config.arm_joints],
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
                with self._io_lock:
                    self.robot.arm.send_pos_vel(
                        motor_position_target,
                        vlim=velocity_limit_target,
                        strict=True,
                    )
                self._record_successful_command(
                    joint_positions=tuple(
                        target[joint.name] for joint in self.config.arm_joints
                    )
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
                thread = self._coupled_control_thread
                background_running = (
                    self._joint_transform is not None
                    and thread is not None
                    and thread.is_alive()
                )
                if self._enabled and not background_running:
                    self._send_mit_command(command, strict=True)
                self._record_successful_command(
                    joint_positions=command.positions,
                    mit_command=command,
                )
                self._start_coupled_control()
                self._coupled_control_wakeup.set()
                return
        except Exception as exc:
            if isinstance(exc, CommunicationError):
                self._record_communication_error(
                    exc,
                    using_fallback=False if isinstance(exc, StaleFeedbackError) else None,
                )
            if isinstance(exc, StaleFeedbackError):
                self._trip_fault(str(exc))
                raise
            holding = self._begin_safe_hold(f"joint command failed: {exc}")
            with self._state_lock:
                has_hold_target = self._last_joint_command is not None
            if holding and has_hold_target:
                return
            raise
        raise ValueError("mode must be 'posvel' or 'mit'")

    def hold_current_position(self) -> ArxDCanState:
        """读取新鲜状态，以当前位置为目标发送命令，并返回该状态样本。"""
        state = self.read_state()
        self.send_joint_positions(state.arm.positions)
        return state

    def hold_joint_positions(
        self,
        positions: Sequence[float],
        *,
        seconds: float | None = None,
        hz: float | None = None,
        gripper: float | None = None,
    ) -> ArxDCanState:
        """持续保持一组关节位置，并可同时保持夹爪开合度。

        ``seconds=None`` 表示持续运行到调用线程被中断。刷新频率默认使用机型配置值。
        此方法负责重复发送命令，普通用户不需要自行编写控制循环。
        """
        if seconds is not None and (
            not math.isfinite(seconds) or seconds < 0.0
        ):
            raise ValueError("seconds must be finite and non-negative or None")
        command_hz = self.config.control_hz if hz is None else float(hz)
        if not math.isfinite(command_hz) or command_hz <= 0.0:
            raise ValueError("hz must be finite and positive")

        period = 1.0 / command_hz
        started = time.monotonic()
        cycle = 0
        while seconds is None or time.monotonic() - started < seconds:
            self.send_joint_positions(positions)
            if gripper is not None:
                self.set_gripper(gripper)
            cycle += 1
            remaining = started + cycle * period - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
        return self.read_state()

    def move_joint_positions(
        self,
        positions: Sequence[float],
        *,
        seconds: float = 3.0,
        hz: float | None = None,
        profile: str = "min_jerk",
    ) -> ArxDCanState:
        """从当前位置平滑移动到目标关节位置，并返回到位后的新鲜状态。

        轨迹插值和发送节拍由 SDK 内部处理。``profile`` 可选 ``min_jerk`` 或
        ``linear``，普通用户通常只需传入目标位置和运动时间。
        """
        duration = float(seconds)
        command_hz = self.config.control_hz if hz is None else float(hz)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("seconds must be finite and positive")
        if not math.isfinite(command_hz) or command_hz <= 0.0:
            raise ValueError("hz must be finite and positive")

        from ..trajectory import plan_joint_position_trajectory

        initial = self.read_state().arm.positions
        points = plan_joint_position_trajectory(
            initial,
            positions,
            duration=duration,
            hz=command_hz,
            profile=profile,
        )
        started = time.monotonic()
        for point in points:
            remaining = started + point.time - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            self.send_joint_positions(point.positions)
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

    def move_gripper(
        self,
        value: float,
        *,
        seconds: float = 2.0,
        hz: float | None = None,
    ) -> GripperState:
        """在指定时间内持续发送夹爪目标，并返回最终的新鲜夹爪状态。"""
        duration = float(seconds)
        command_hz = self.config.control_hz if hz is None else float(hz)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("seconds must be finite and positive")
        if not math.isfinite(command_hz) or command_hz <= 0.0:
            raise ValueError("hz must be finite and positive")

        period = 1.0 / command_hz
        started = time.monotonic()
        cycle = 0
        while time.monotonic() - started < duration:
            self.set_gripper(value)
            cycle += 1
            remaining = started + cycle * period - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
        state = self.read_state()
        if state.gripper is None:
            raise RuntimeError("ARX-D-CAN gripper feedback is unavailable")
        return state.gripper

    def set_zero(
        self,
        *,
        joint_names: Sequence[str] | None = None,
        verify_tolerance: float = 0.02,
        verify_velocity: float = 0.05,
        verify_samples: int = 3,
    ) -> tuple[str, ...]:
        """将所选电机的当前位置写为零点，并通过反馈验证。

        机械臂必须已连接、无故障且处于失能状态。验证要求连续 ``verify_samples``
        个新鲜样本均满足位置和速度容差。机型定义的耦合电机不允许执行此操作，因为
        其公开零点并非独立物理电机的零点。
        """
        self._require_operational()
        if self._enabled:
            raise RuntimeError("disable the arm before writing motor zero positions")
        if self._joint_transform is not None:
            coupled_names = {
                self.config.arm_joints[index].name
                for index in self._joint_transform.transformed_indices
            }
            requested_names = (
                set(self.config.joint_names)
                if joint_names is None
                else {str(name) for name in joint_names}
            )
            protected = sorted(coupled_names.intersection(requested_names))
            if protected:
                raise RuntimeError(
                    "model-defined coupled joints cannot be motor-zeroed: "
                    + ", ".join(protected)
                )
        return self.robot.set_zero(
            joint_names=list(joint_names) if joint_names is not None else None,
            verify_tolerance=verify_tolerance,
            verify_velocity=verify_velocity,
            verify_samples=verify_samples,
        )

    def set_gripper(self, value: float) -> None:
        """使用简单的 ``0``～``1000`` 刻度设置夹爪开合度。

        ``0`` 表示完全闭合，``1000`` 表示完全张开，超出范围的值会自动限幅。
        夹爪始终使用机型配置中的 MIT 增益；Yunyi 默认 Kp 为 4.0、Kd 为 0.5。
        """
        self._require_connected()
        if self._safe_holding:
            return
        self._require_operational()
        if self.config.gripper is None:
            return
        if not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        self.set_gripper_motor_value(
            self._gripper_motor_position_from_opening(value)
        )

    def open_gripper(self) -> None:
        """使用默认 MIT 增益完全张开夹爪。"""
        self.set_gripper(1000.0)

    def close_gripper(self) -> None:
        """使用默认 MIT 增益完全闭合夹爪。"""
        self.set_gripper(0.0)

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
        :meth:`set_gripper`、:meth:`open_gripper` 或 :meth:`close_gripper`。
        目标会限制在配置的行程范围内，并通过 MIT 模式发送。
        """
        self._require_connected()
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
        with self._gripper_command_lock:
            if not self._enabled:
                raise RuntimeError("ARX-D-CAN arm is not enabled")
            if self._gripper_force_controller is not None:
                with self._io_lock:
                    position, _, torque = self.robot.gripper.read_state(
                        request_feedback=True
                    )
                if len(position) != 1 or len(torque) != 1:
                    raise RuntimeError("gripper feedback must contain exactly one motor")
                command = self._gripper_force_controller.update(
                    requested_position=target,
                    actual_position=float(position[0]),
                    actual_torque=float(torque[0]),
                    now=time.monotonic(),
                )
                try:
                    with self._io_lock:
                        self.robot.gripper.send_mit(
                            np.array([command.position]),
                            kp=np.array([command.kp]),
                            kd=np.array([command.kd]),
                            strict=True,
                        )
                except Exception as exc:
                    if isinstance(exc, CommunicationError):
                        self._record_communication_error(exc, using_fallback=None)
                    self._gripper_force_controller.reset()
                    holding = self._begin_safe_hold(f"gripper command failed: {exc}")
                    with self._state_lock:
                        has_hold_target = self._last_gripper_command is not None
                    if holding and has_hold_target:
                        return
                    raise
                self._record_successful_command(
                    gripper_position=float(command.position)
                )
                return
            try:
                with self._io_lock:
                    self.robot.gripper.send_mit(
                        np.array([target]),
                        strict=True,
                    )
            except Exception as exc:
                if isinstance(exc, CommunicationError):
                    self._record_communication_error(exc, using_fallback=None)
                holding = self._begin_safe_hold(f"gripper command failed: {exc}")
                with self._state_lock:
                    has_hold_target = self._last_gripper_command is not None
                if holding and has_hold_target:
                    return
                raise
            self._record_successful_command(gripper_position=target)
