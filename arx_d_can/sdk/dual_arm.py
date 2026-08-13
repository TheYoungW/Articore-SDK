"""两条独立 CAN 通道组成的双臂控制接口。"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from ..driver import ControllerGroup
from .arm import ArxDCanArm
from .native_safety import (
    DisableReport,
    EnableReport,
    GripperForceLevel,
    GripperSafetyHealth,
    NativeMotorDescriptor,
    NativeSafetyRuntime,
    SafetyHealth,
    SafetyState,
    TransportHealth,
)
from .state import ArxDCanState


@dataclass(slots=True, frozen=True)
class ArxDCanDualArmState:
    """一帧左右臂状态；两条 CAN 通道的状态保持独立。"""

    left: ArxDCanState
    right: ArxDCanState
    left_gripper: GripperSafetyHealth | None = None
    right_gripper: GripperSafetyHealth | None = None


class ArxDCanDualArm:
    """组合两台独立的 :class:`ArxDCanArm`。

    当前默认机型是 Yunyi V1.0 左右臂，以后其他双臂产品可以分别传入自己的
    ``left_model`` 和 ``right_model``，无需新增产品专用 Python 类。连接参数默认
    来自产品配置；Yunyi 左臂使用 CH0、右臂使用 CH1。控制模式默认使用 MIT，
    构造参数仅用于显式覆盖。
    """

    def __init__(
        self,
        *,
        left_model: str = "yunyi_v1_0_left",
        right_model: str = "yunyi_v1_0_right",
        transport: str | None = None,
        left_channel: str | None = None,
        right_channel: str | None = None,
        baud: int | None = None,
        control_mode: str = "mit",
        left_gripper: bool | None = None,
        right_gripper: bool | None = None,
    ) -> None:
        normalized_mode = str(control_mode).strip().lower()
        if normalized_mode not in ("pv", "mit"):
            raise ValueError("control_mode must be 'pv' or 'mit'")
        self.left = ArxDCanArm(
            model=left_model,
            channel=left_channel,
            transport=transport,
            baud=baud,
            control_mode=normalized_mode,
            enable_gripper=left_gripper,
        )
        self.right = ArxDCanArm(
            model=right_model,
            channel=right_channel,
            transport=transport,
            baud=baud,
            control_mode=normalized_mode,
            enable_gripper=right_gripper,
        )
        self._controller_group: ControllerGroup | None = None
        self._safety_runtime: NativeSafetyRuntime | None = None

    @property
    def connected(self) -> bool:
        """返回左右两条通道是否都已连接。"""
        return self.left.connected and self.right.connected

    @property
    def enabled(self) -> bool:
        """返回左右臂是否都已使能。"""
        if self._safety_runtime is not None:
            self._sync_python_safety_flags(self._safety_runtime.health)
        return self.left.enabled and self.right.enabled

    @property
    def safety_health(self) -> SafetyHealth:
        """返回 C++ 双臂安全状态机的一致快照。"""
        runtime = self._safety_runtime
        if runtime is not None:
            health = runtime.health
            self._sync_python_safety_flags(health)
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
        return SafetyHealth(
            state=state,
            fault_reason=None,
            last_successful_command_age_s=None,
            last_fresh_feedback_age_s=None,
            consecutive_send_failures=0,
            consecutive_feedback_failures=0,
            left_transport=transport,
            right_transport=transport,
            motor_faults=(),
            unconfirmed_disable_motors=(),
            safe_holding=False,
            disable_confirmed=not self.enabled,
        )

    @property
    def last_enable_report(self) -> EnableReport | None:
        """返回最近一次双臂原子使能报告。"""
        runtime = self._safety_runtime
        return None if runtime is None else runtime.last_enable_report

    @property
    def last_disable_report(self) -> DisableReport | None:
        """返回最近一次双臂确定性失能报告。"""
        runtime = self._safety_runtime
        return None if runtime is None else runtime.last_disable_report

    def _native_motor_descriptors(self) -> tuple[NativeMotorDescriptor, ...]:
        return (
            *self.left._native_motor_descriptors(side=0, label="left"),
            *self.right._native_motor_descriptors(side=1, label="right"),
        )

    def _native_joint_control_configs(self):
        return (
            *self.left._native_joint_control_configs(),
            *self.right._native_joint_control_configs(),
        )

    def _native_joint_safety_limits(self):
        return (
            *self.left._native_joint_safety_limits(),
            *self.right._native_joint_safety_limits(),
        )

    def _native_gripper_force_profiles(self):
        return (
            *self.left._native_gripper_force_profiles(),
            *self.right._native_gripper_force_profiles(),
        )

    def _create_safety_runtime(
        self,
        group: ControllerGroup,
        left_controller: object,
        right_controller: object,
    ) -> NativeSafetyRuntime | None:
        # 测试桩可能没有原生句柄；真实 motor-drive-layer 0.9.0 对象必须具备。
        if not all(
            getattr(value, "_ptr", None)
            for value in (group, left_controller, right_controller)
        ):
            return None
        left = self.left.config
        right = self.right.config
        runtime = NativeSafetyRuntime(
            controller_group=group,
            left_controller=left_controller,
            right_controller=right_controller,
            motors=self._native_motor_descriptors(),
            joints=self._native_joint_control_configs(),
            joint_safety_limits=self._native_joint_safety_limits(),
            gripper_force_profiles=self._native_gripper_force_profiles(),
            control_hz=min(left.control_hz, right.control_hz),
            command_timeout_s=min(left.command_timeout_s, right.command_timeout_s),
            enable_grace_s=min(left.enable_grace_s, right.enable_grace_s),
            safe_hold_hz=min(left.safe_hold_hz, right.safe_hold_hz),
            feedback_check_hz=min(left.feedback_check_hz, right.feedback_check_hz),
            feedback_failure_threshold=min(
                left.feedback_fault_threshold, right.feedback_fault_threshold
            ),
            feedback_max_age_s=min(
                left.max_cached_feedback_age_s,
                right.max_cached_feedback_age_s,
            ),
            safe_hold_failure_threshold=min(
                left.safe_hold_failure_threshold,
                right.safe_hold_failure_threshold,
            ),
            safe_pv_velocity_limit=min(
                left.safe_hold_pv_velocity_limit,
                right.safe_hold_pv_velocity_limit,
            ),
            # ABI 字段为兼容保留；1.8 正常运行时夹爪跟随机械臂控制频率。
            gripper_control_hz=min(left.control_hz, right.control_hz),
            gripper_fault_action=(
                "disable"
                if "disable" in {
                    left.gripper_fault_action.strip().lower(),
                    right.gripper_fault_action.strip().lower(),
                }
                else "hold"
            ),
        )
        runtime.connect()
        return runtime

    def _sync_python_safety_flags(self, health: SafetyHealth) -> None:
        active = health.state in {
            SafetyState.ENABLED,
            SafetyState.RUNNING,
            SafetyState.SAFE_HOLD,
        } or (health.state is SafetyState.FAULT and not health.disable_confirmed)
        for arm in (self.left, self.right):
            with arm._state_lock:
                arm._enabled = active
                arm._safe_holding = health.state is SafetyState.SAFE_HOLD
                arm._faulted = health.state in {
                    SafetyState.SAFE_HOLD,
                    SafetyState.FAULT,
                }
                arm._fault_reason = health.fault_reason

    def connect(self) -> None:
        """连接左右臂，并为两条通道创建常驻并行发送线程。"""
        if self.connected and self._controller_group is not None:
            return
        self.left._set_dual_runtime_managed(True)
        self.right._set_dual_runtime_managed(True)
        try:
            self.left.connect()
            self.right.connect()
            left_controller = self.left._controller_for_parallel_batch()
            right_controller = self.right._controller_for_parallel_batch()
            group = ControllerGroup([left_controller, right_controller])
            self._controller_group = group
            self._safety_runtime = self._create_safety_runtime(
                group, left_controller, right_controller
            )
            if self._safety_runtime is None:
                raise RuntimeError(
                    "motor-drive-layer 0.9.0 dual-arm safety runtime is unavailable"
                )
        except Exception:
            if self._safety_runtime is not None:
                # Runtime 关闭失败时仍依赖后续所有句柄；异常直接向上传播，绝不能
                # 继续释放 ControllerGroup、Controller 或 Transport。
                self._safety_runtime.close()
                self._safety_runtime = None
            if self._controller_group is not None:
                self._controller_group.close()
                self._controller_group = None
            if self.right.connected:
                self.right.close()
            self.left.close()
            self.left._set_dual_runtime_managed(False)
            self.right._set_dual_runtime_managed(False)
            raise

    def enable(self) -> None:
        """按构造时确定的模式，通过一个原生原子事务使能双臂。

        Python 只在调用前配置左右臂控制模式和电机参数；Runtime 负责并行刷新
        CH0/CH1 反馈、生成当前位置保持目标、物理使能、确认和失败回滚。普通用户
        无需传入参数。
        """
        runtime = self._safety_runtime
        if runtime is None:
            raise RuntimeError("dual-arm safety runtime is not connected")
        if not self.left._configured:
            self.left._configure()
        if not self.right._configured:
            self.right._configure()
        try:
            runtime.enable(self.left._mode)
            self._sync_python_safety_flags(runtime.health)
        except Exception:
            # 健康快照只是补充状态，不能覆盖携带 EnableReport 的原始异常。
            try:
                self._sync_python_safety_flags(runtime.health)
            except Exception:
                pass
            raise

    def configure_mode(self, mode: str) -> None:
        """在双臂失能时将左右机械臂切换到同一种 PV 或 MIT 模式。"""
        normalized = str(mode).strip().lower().replace("_", "")
        if normalized in {"pv", "posvel"}:
            normalized = "pv"
        elif normalized != "mit":
            raise ValueError("mode must be 'pv' or 'mit'")
        runtime = self._safety_runtime
        if runtime is None or not self.connected:
            raise RuntimeError("dual-arm safety runtime is not connected")
        self._sync_python_safety_flags(runtime.health)
        if self.enabled:
            raise RuntimeError(
                "cannot switch control mode while the dual arm is enabled; "
                "call disable() first"
            )

        previous = (self.left._mode, self.right._mode)
        try:
            self.left.configure_mode(normalized)
            self.right.configure_mode(normalized)
        except Exception as exc:
            rollback_errors = []
            for arm, previous_mode in zip((self.left, self.right), previous):
                try:
                    arm.configure_mode(previous_mode)
                except Exception as rollback_exc:
                    rollback_errors.append(rollback_exc)
            if rollback_errors:
                try:
                    runtime.estop("dual-arm mode switch rollback failed")
                finally:
                    self._sync_python_safety_flags(runtime.health)
            raise RuntimeError("dual-arm control mode switch failed") from exc

        self.left._configured = False
        self.right._configured = False

    def disable(self) -> None:
        """失能左右臂；一侧失败时仍继续处理另一侧。"""
        runtime = self._safety_runtime
        if runtime is None:
            raise RuntimeError("dual-arm safety runtime is not connected")
        try:
            runtime.disable()
        except Exception as exc:
            # 不允许后续 health 读取失败覆盖结构化 NativeDisableError。
            try:
                health = runtime.health
            except Exception:
                for arm in (self.left, self.right):
                    with arm._state_lock:
                        arm._enabled = True
                        arm._faulted = True
                        arm._fault_reason = f"disable failed: {exc}"
                        arm._safe_holding = False
            else:
                self._sync_python_safety_flags(health)
            raise
        else:
            self._sync_python_safety_flags(runtime.health)

    def close(self) -> None:
        """按 Runtime → ControllerGroup → Transport 的顺序关闭双臂。

        ABI 2.0 的 Runtime 关闭包含确定性失能事务。若无法确认所有电机失能，
        此方法保留全部底层句柄，以便调用方检查 ``last_disable_report`` 并重试。
        """
        errors: list[Exception] = []
        runtime = self._safety_runtime
        if runtime is not None:
            # close() 失败时必须原样保留 runtime、group、两侧 controller/transport。
            runtime.close()
            self._safety_runtime = None
        group = self._controller_group
        if group is not None:
            try:
                group.close()
            except Exception as exc:
                errors.append(exc)
            else:
                self._controller_group = None
        if errors:
            raise RuntimeError("failed to close dual-arm ControllerGroup") from errors[0]
        self.left._set_dual_runtime_managed(False)
        self.right._set_dual_runtime_managed(False)
        for arm in (self.left, self.right):
            if not arm.connected:
                continue
            try:
                arm.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError("failed to close one or more arms") from errors[0]

    def read_state(self) -> ArxDCanDualArmState:
        """分别读取左右臂状态。"""
        states = []
        errors = []
        for side, arm in enumerate((self.left, self.right)):
            try:
                states.append(arm.read_state())
            except Exception as exc:
                errors.append(exc)
                if self._safety_runtime is not None:
                    self._safety_runtime.report_feedback_failure(side, str(exc))
        if errors:
            raise errors[0]
        health = self._safety_runtime.health if self._safety_runtime else None
        return ArxDCanDualArmState(
            left=states[0],
            right=states[1],
            left_gripper=health.left_gripper if health else None,
            right_gripper=health.right_gripper if health else None,
        )

    def read_cached_state(self) -> ArxDCanDualArmState:
        """分别返回左右臂最近一次成功反馈，不发送新的查询帧。"""
        health = self._safety_runtime.health if self._safety_runtime else None
        return ArxDCanDualArmState(
            left=self.left.read_cached_state(),
            right=self.right.read_cached_state(),
            left_gripper=health.left_gripper if health else None,
            right_gripper=health.right_gripper if health else None,
        )

    def _submit_joint_positions(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        left_velocities: Sequence[float] | None = None,
        right_velocities: Sequence[float] | None = None,
        left_velocity_limits: Sequence[float] | None = None,
        right_velocity_limits: Sequence[float] | None = None,
        left_torques: Sequence[float] | None = None,
        right_torques: Sequence[float] | None = None,
        left_mit_kp: float | Sequence[float] | None = None,
        right_mit_kp: float | Sequence[float] | None = None,
        left_mit_kd: float | Sequence[float] | None = None,
        right_mit_kd: float | Sequence[float] | None = None,
        enforce_position_limits: bool = True,
    ) -> None:
        """在 SDK 内部原子更新左右臂容量为一的最新关节目标。

        PV 模式可分别指定左右臂逐关节速度上限；省略时使用 SDK 默认值。MIT 模式
        可分别指定左右臂目标速度、前馈力矩和逐关节 Kp/Kd。
        """
        runtime = self._safety_runtime
        if runtime is None:
            raise RuntimeError("dual-arm safety runtime is not connected")
        self._sync_python_safety_flags(runtime.health)
        left_batch = self.left._prepare_parallel_joint_positions(
            left,
            velocities=left_velocities,
            velocity_limits=left_velocity_limits,
            torques=left_torques,
            mit_kp=left_mit_kp,
            mit_kd=left_mit_kd,
            enforce_position_limits=enforce_position_limits,
        )
        right_batch = self.right._prepare_parallel_joint_positions(
            right,
            velocities=right_velocities,
            velocity_limits=right_velocity_limits,
            torques=right_torques,
            mit_kp=right_mit_kp,
            mit_kd=right_mit_kd,
            enforce_position_limits=enforce_position_limits,
        )
        if left_batch.mode != right_batch.mode:
            raise RuntimeError("left and right arms must use the same control mode")

        commands = left_batch.commands + right_batch.commands
        try:
            (runtime.submit_pos_vel if left_batch.mode == "pv" else runtime.submit_mit)(
                commands
            )
        finally:
            self._sync_python_safety_flags(runtime.health)

    def _set_joint_position(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        velocity: float,
        mode: str,
    ) -> None:
        expected = {"pv", "posvel"} if mode == "pv" else {"mit"}
        if self.left._mode not in expected or self.right._mode not in expected:
            raise RuntimeError(f"set_joint_{mode}() requires dual-arm {mode.upper()} mode")
        runtime = self._safety_runtime
        if runtime is None:
            raise RuntimeError("dual-arm safety runtime is not connected")
        self._sync_python_safety_flags(runtime.health)
        left_targets = self.left._ordinary_joint_position_targets(left)
        right_targets = self.right._ordinary_joint_position_targets(right)
        left_velocity = self.left._ordinary_joint_velocity(velocity)
        right_velocity = self.right._ordinary_joint_velocity(velocity)
        if left_velocity != right_velocity:
            raise RuntimeError("left and right ordinary joint velocities must match")
        try:
            getattr(runtime, f"set_joint_{mode}")(
                left_targets + right_targets,
                left_velocity,
            )
        finally:
            self._sync_python_safety_flags(runtime.health)

    def set_joint_mit(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        velocity: float = 1.0,
    ) -> None:
        """原子设置双臂普通 MIT 最终位置，并由 Runtime 以统一速度推进。"""
        self._set_joint_position(
            left=left,
            right=right,
            velocity=velocity,
            mode="mit",
        )

    def set_joint_pv(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        velocity: float = 1.0,
    ) -> None:
        """原子设置双臂普通 PV 最终位置，并由 Runtime 以统一速度推进。"""
        self._set_joint_position(
            left=left,
            right=right,
            velocity=velocity,
            mode="pv",
        )

    def estop(self, reason: str = "emergency stop") -> None:
        """锁存双臂故障并尝试失能所有关节和夹爪。"""
        runtime = self._safety_runtime
        if runtime is None:
            raise RuntimeError("dual-arm safety runtime is not connected")
        runtime.estop(reason)
        self._sync_python_safety_flags(runtime.health)

    def recover(self) -> None:
        """验证双通道、反馈、故障码和物理失能后只恢复到 READY。"""
        runtime = self._safety_runtime
        if runtime is None:
            raise RuntimeError("dual-arm safety runtime is not connected")
        state = runtime.health.state
        if state is SafetyState.SAFE_HOLD:
            runtime.disable()
        elif state is SafetyState.FAULT:
            runtime.disable()
            runtime.recover()
        elif state is not SafetyState.READY:
            raise RuntimeError("dual arm can only recover from SAFE_HOLD or FAULT")
        self._sync_python_safety_flags(runtime.health)

    def set_gripper_openings(
        self,
        *,
        left: float,
        right: float,
        speed: float = 1000.0,
        force_level: GripperForceLevel = GripperForceLevel.LEVEL_5,
    ) -> None:
        """原子提交全部已安装夹爪的开合度、归一化速度和夹持力等级。"""
        values = (float(left), float(right))
        if any(not math.isfinite(value) for value in values):
            raise ValueError("gripper openings must be finite")
        values = tuple(max(0.0, min(1000.0, value)) for value in values)
        normalized_speed = float(speed)
        if not math.isfinite(normalized_speed) or not 0.0 < normalized_speed <= 1000.0:
            raise ValueError("gripper speed must be finite and in (0, 1000]")
        level = GripperForceLevel(force_level)
        active = tuple(
            (arm, value)
            for arm, value in zip((self.left, self.right), values)
            if arm.has_gripper
        )
        if not active:
            raise RuntimeError("no active product gripper is configured")
        runtime = self._safety_runtime
        if runtime is None:
            raise RuntimeError("dual-arm safety runtime is not connected")
        commands = tuple(
            (
                arm.robot._motor_map[arm.config.gripper.name],
                value,
                normalized_speed,
                level,
            )
            for arm, value in active
        )
        runtime.set_gripper_commands(commands)


__all__ = ["ArxDCanDualArm", "ArxDCanDualArmState"]
