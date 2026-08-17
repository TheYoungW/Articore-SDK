"""两条独立 CAN 通道组成的双臂控制接口。"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Sequence

from motor_drive_layer import (
    ArticoreRuntime,
    DisableReport,
    EnableReport,
    GripperCommand,
    GripperHealth,
    RuntimeConfig,
    RuntimeControlMode,
    RuntimeMotor,
    RuntimeTransportHealth,
    SafetyHealth,
    SafetyState,
)

from ..driver import ControllerGroup
from .arm import ArxDCanArm, _runtime_raw_commands
from .gripper import GripperForceLevel
from .state import ArxDCanState


@dataclass(slots=True, frozen=True)
class ArxDCanDualArmState:
    """一帧左右臂状态；两条 CAN 通道的状态保持独立。"""

    left: ArxDCanState
    right: ArxDCanState
    left_gripper: GripperHealth | None = None
    right_gripper: GripperHealth | None = None


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
        self._safety_runtime: ArticoreRuntime | None = None

    @property
    def connected(self) -> bool:
        """返回左右两条通道是否都已连接。"""
        return self.left.connected and self.right.connected

    @property
    def _effective_control_hz(self) -> float:
        """返回 SDK 内部双臂调度频率，不属于公开控制接口。"""
        runtime = self._safety_runtime
        if runtime is not None:
            return float(runtime.control_hz)
        return min(self.left.config.control_hz, self.right.config.control_hz)

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
        return SafetyHealth(
            state=state,
            safe_holding=False,
            disable_confirmed=not self.enabled,
            last_successful_command_age_ns=None,
            last_fresh_feedback_age_ns=None,
            consecutive_send_failures=0,
            consecutive_feedback_failures=0,
            left_transport=transport,
            right_transport=transport,
            grippers=(),
            motor_faults=(),
            unconfirmed_disable=(),
            fault_reason=None,
        )

    @property
    def last_enable_report(self) -> EnableReport | None:
        """返回最近一次双臂原子使能报告。"""
        runtime = self._safety_runtime
        return None if runtime is None else runtime.last_enable_report()

    @property
    def last_disable_report(self) -> DisableReport | None:
        """返回最近一次双臂确定性失能报告。"""
        runtime = self._safety_runtime
        return None if runtime is None else runtime.last_disable_report()

    def _runtime_motors(self) -> tuple[RuntimeMotor, ...]:
        return (
            *self.left._runtime_motors(side=0, label="left"),
            *self.right._runtime_motors(side=1, label="right"),
        )

    def _runtime_joint_configs(self):
        return (
            *self.left._runtime_joint_configs(),
            *self.right._runtime_joint_configs(),
        )

    def _runtime_joint_limits(self):
        return (
            *self.left._runtime_joint_limits(),
            *self.right._runtime_joint_limits(),
        )

    def _runtime_gripper_bindings(self):
        return (
            *self.left._runtime_gripper_bindings(),
            *self.right._runtime_gripper_bindings(),
        )

    def _create_safety_runtime(
        self,
        group: ControllerGroup,
        left_controller: object,
        right_controller: object,
    ) -> ArticoreRuntime | None:
        # 测试桩可能没有原生句柄；真实 motor-drive-layer 0.10.6 对象必须具备。
        if not all(
            getattr(value, "_ptr", None)
            for value in (group, left_controller, right_controller)
        ):
            return None
        left = self.left.config
        right = self.right.config
        runtime = ArticoreRuntime(
            config=RuntimeConfig(
                control_hz=max(1, round(min(left.control_hz, right.control_hz))),
                command_timeout_ms=max(
                    1, round(min(left.command_timeout_s, right.command_timeout_s) * 1000)
                ),
                enable_grace_ms=max(
                    1, round(min(left.enable_grace_s, right.enable_grace_s) * 1000)
                ),
                safe_hold_hz=max(1, round(min(left.safe_hold_hz, right.safe_hold_hz))),
                feedback_check_hz=max(
                    1, round(min(left.feedback_check_hz, right.feedback_check_hz))
                ),
                feedback_failure_threshold=min(
                    left.feedback_fault_threshold, right.feedback_fault_threshold
                ),
                feedback_max_age_ms=max(
                    1,
                    round(
                        min(
                            left.max_cached_feedback_age_s,
                            right.max_cached_feedback_age_s,
                        )
                        * 1000
                    ),
                ),
                safe_hold_failure_threshold=min(
                    left.safe_hold_failure_threshold,
                    right.safe_hold_failure_threshold,
                ),
                safe_pv_velocity_limit=min(
                    left.safe_hold_pv_velocity_limit,
                    right.safe_hold_pv_velocity_limit,
                ),
                gripper_control_hz=max(
                    1, round(min(left.control_hz, right.control_hz))
                ),
            ),
            controller_group=group,
            left_controller=left_controller,
            right_controller=right_controller,
            motors=self._runtime_motors(),
        )
        try:
            runtime.configure_joints(self._runtime_joint_configs())
            runtime.configure_joint_safety_limits(self._runtime_joint_limits())
            runtime.configure_gripper_products(self._runtime_gripper_bindings())
            runtime.connect()
        except Exception:
            # connect() 失败时局部 Runtime 仍持有 Group/Controller/Motor 租用。
            # 必须先释放 Runtime，调用方随后才能安全关闭 ControllerGroup。
            try:
                runtime.close()
            except Exception:
                pass
            raise
        return runtime

    def _release_runtime(self) -> None:
        """释放共享 Runtime 的租用，但保留 ControllerGroup。"""
        runtime = self._safety_runtime
        if runtime is None:
            return
        error: Exception | None = None
        try:
            runtime.close()
        except Exception as exc:
            error = exc
        if getattr(runtime, "closed", True):
            self._safety_runtime = None
        else:
            raise error or RuntimeError("ArticoreRuntime did not close")
        if error is not None:
            raise error

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
                    "motor-drive-layer 0.10.6 dual-arm ArticoreRuntime is unavailable"
                )
        except Exception:
            if self._safety_runtime is not None:
                self._release_runtime()
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
            runtime.enable(
                RuntimeControlMode.PV
                if self.left._mode in {"pv", "posvel"}
                else RuntimeControlMode.MIT
            )
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

        group = self._controller_group
        if group is None:
            raise RuntimeError("dual-arm ControllerGroup is not connected")
        previous = (self.left._mode, self.right._mode)
        self._release_runtime()
        try:
            self.left.configure_mode(normalized)
            self.right.configure_mode(normalized)
        except Exception as exc:
            for arm, previous_mode in zip((self.left, self.right), previous):
                try:
                    arm.configure_mode(previous_mode)
                except Exception:
                    pass
            raise RuntimeError("dual-arm control mode switch failed") from exc
        self.left._configured = True
        self.right._configured = True
        self._safety_runtime = self._create_safety_runtime(
            group,
            self.left._controller_for_parallel_batch(),
            self.right._controller_for_parallel_batch(),
        )
        if self._safety_runtime is None:
            raise RuntimeError("failed to recreate dual-arm ArticoreRuntime")

    def disable(self) -> None:
        """失能左右臂；一侧失败时仍继续处理另一侧。"""
        runtime = self._safety_runtime
        if runtime is None:
            raise RuntimeError("dual-arm safety runtime is not connected")
        try:
            runtime.disable()
        except Exception as exc:
            # 不允许后续 health 读取失败覆盖结构化 RuntimeTransactionError。
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

        native 句柄和资源租用由 motor-drive-layer 的正式 Runtime 封装管理。
        """
        errors: list[Exception] = []
        try:
            self._release_runtime()
        except Exception as exc:
            errors.append(exc)
        group = self._controller_group
        if group is not None:
            try:
                group.close()
            except Exception as exc:
                errors.append(exc)
            else:
                self._controller_group = None
        if self._controller_group is not None:
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
            raise errors[0]

    def read_state(self) -> ArxDCanDualArmState:
        """读取 Runtime 后台持续刷新的左右臂缓存状态。"""
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
        left_gripper = self._gripper_health(health, 0)
        right_gripper = self._gripper_health(health, 1)
        return ArxDCanDualArmState(
            left=self._state_with_runtime_gripper(states[0], left_gripper),
            right=self._state_with_runtime_gripper(states[1], right_gripper),
            left_gripper=left_gripper,
            right_gripper=right_gripper,
        )

    def read_cached_state(self) -> ArxDCanDualArmState:
        """分别返回左右臂最近一次成功反馈，不发送新的查询帧。"""
        health = self._safety_runtime.health if self._safety_runtime else None
        left_gripper = self._gripper_health(health, 0)
        right_gripper = self._gripper_health(health, 1)
        return ArxDCanDualArmState(
            left=self._state_with_runtime_gripper(
                self.left.read_cached_state(),
                left_gripper,
            ),
            right=self._state_with_runtime_gripper(
                self.right.read_cached_state(),
                right_gripper,
            ),
            left_gripper=left_gripper,
            right_gripper=right_gripper,
        )

    @staticmethod
    def _gripper_health(
        health: SafetyHealth | None,
        side: int,
    ) -> GripperHealth | None:
        if health is None:
            return None
        return next((item for item in health.grippers if item.side == side), None)

    @staticmethod
    def _state_with_runtime_gripper(
        state: ArxDCanState,
        health: GripperHealth | None,
    ) -> ArxDCanState:
        """用 Runtime 产品 profile 计算的开合度补全公开夹爪状态。"""
        if state.gripper is None or health is None:
            return state
        return replace(state, gripper=replace(state.gripper, opening=health.opening))

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
            batch = type(left_batch)(mode=left_batch.mode, commands=commands)
            (runtime.submit_pv if left_batch.mode == "pv" else runtime.submit_mit)(
                _runtime_raw_commands(batch)
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
        left_velocity = self.left._ordinary_joint_velocity(velocity, mode=mode)
        right_velocity = self.right._ordinary_joint_velocity(velocity, mode=mode)
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

    def submit_raw_mit(
        self,
        *,
        left_positions: Sequence[float],
        right_positions: Sequence[float],
        left_velocities: Sequence[float] | None = None,
        right_velocities: Sequence[float] | None = None,
        kp: float | Sequence[float] | None = None,
        kd: float | Sequence[float] | None = None,
        left_feedforward_torques: Sequence[float] | None = None,
        right_feedforward_torques: Sequence[float] | None = None,
    ) -> None:
        """原子提交一帧完整双臂 raw MIT 命令。

        这是供高级控制器使用的流式接口，要求双臂以 MIT 模式连接并已经使能。每次
        调用必须同时提供左右臂目标；Runtime 只保留最新完整帧，并按底层实际控制
        周期发送。调用方必须在命令看门狗超时前持续更新，否则 Runtime 会进入安全
        保持。

        位置单位为 rad，速度单位为 rad/s，前馈力矩单位为 N·m。``kp`` 和 ``kd``
        对左右臂使用同一组标量或七关节向量；省略时使用产品配置值。省略速度或前馈
        力矩时对应值为零。SDK 会使用提交时的最新缓存 ``q/dq`` 估算完整的
        ``Kp·位置误差 + Kd·速度误差 + tau_ff``；超过逐关节 URDF ``effort`` 的
        80% 时，按比例同步缩小该关节的 Kp、Kd 和 ``tau_ff``。这是提交帧级保护，
        后续下沉到 Runtime 后才能在每个底层发送周期重新计算。
        """
        state = self.read_cached_state()
        left_velocity, left_kp, left_kd, left_torques = (
            self.left._limit_raw_mit_resultant_torque(
                positions=left_positions,
                velocities=left_velocities,
                kp=kp,
                kd=kd,
                feedforward_torques=left_feedforward_torques,
                current_positions=state.left.arm.positions,
                current_velocities=state.left.arm.velocities,
            )
        )
        right_velocity, right_kp, right_kd, right_torques = (
            self.right._limit_raw_mit_resultant_torque(
                positions=right_positions,
                velocities=right_velocities,
                kp=kp,
                kd=kd,
                feedforward_torques=right_feedforward_torques,
                current_positions=state.right.arm.positions,
                current_velocities=state.right.arm.velocities,
            )
        )
        self._submit_joint_positions(
            left=left_positions,
            right=right_positions,
            left_velocities=left_velocity,
            right_velocities=right_velocity,
            left_torques=left_torques,
            right_torques=right_torques,
            left_mit_kp=left_kp,
            right_mit_kp=right_kp,
            left_mit_kd=left_kd,
            right_mit_kd=right_kd,
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

    def set_zero(
        self,
        *,
        verify_tolerance: float = 0.02,
        verify_velocity: float = 0.05,
        verify_samples: int = 3,
    ) -> tuple[tuple[str, ...], tuple[str, ...]]:
        """暂停共享 Runtime 后设置左右臂全部已安装电机的零点。"""
        runtime = self._safety_runtime
        group = self._controller_group
        if runtime is None or group is None:
            raise RuntimeError("dual-arm ArticoreRuntime is not connected")
        self._sync_python_safety_flags(runtime.health)
        if self.enabled:
            raise RuntimeError("disable the dual arm before writing motor zero positions")

        self._release_runtime()
        try:
            left = self.left.robot.set_zero(
                joint_names=None,
                verify_tolerance=verify_tolerance,
                verify_velocity=verify_velocity,
                verify_samples=verify_samples,
            )
            right = self.right.robot.set_zero(
                joint_names=None,
                verify_tolerance=verify_tolerance,
                verify_velocity=verify_velocity,
                verify_samples=verify_samples,
            )
        finally:
            self._safety_runtime = self._create_safety_runtime(
                group,
                self.left._controller_for_parallel_batch(),
                self.right._controller_for_parallel_batch(),
            )
        return left, right

    def clear_motor_faults(self) -> tuple[tuple[str, ...], tuple[str, ...]]:
        """清除左右臂全部已安装电机故障，并保持所有电机失能。

        未连接时使用维护路径：只打开 Controller 和 Motor，不配置 MIT/PV、不创建
        Runtime，清错完成后立即关闭。这样即使故障电机暂时不能写入控制模式，也能
        先完成恢复。已经正常连接时则先释放 Runtime 租用，清错后重建 Runtime。
        """
        runtime = self._safety_runtime
        group = self._controller_group
        if runtime is None and group is None and not self.connected:
            return self._clear_motor_faults_maintenance()
        if runtime is None or group is None or not self.connected:
            raise RuntimeError("dual-arm connection is incomplete")
        self._release_runtime()
        try:
            left = self.left.robot.clear_errors(
                joint_names=self.left._active_joint_names()
            )
            right = self.right.robot.clear_errors(
                joint_names=self.right._active_joint_names()
            )
        finally:
            self._safety_runtime = self._create_safety_runtime(
                group,
                self.left._controller_for_parallel_batch(),
                self.right._controller_for_parallel_batch(),
            )
        for arm in (self.left, self.right):
            with arm._state_lock:
                arm._configured = True
                arm._enabled = False
                arm._faulted = False
                arm._fault_reason = None
                arm._safe_holding = False
        return left, right

    def _clear_motor_faults_maintenance(
        self,
    ) -> tuple[tuple[str, ...], tuple[str, ...]]:
        """通过不配置控制模式的临时连接清除双臂电机故障。"""
        arms = (("left", self.left), ("right", self.right))
        opened: list[tuple[str, ArxDCanArm]] = []
        results: dict[str, tuple[str, ...]] = {}
        errors: list[str] = []
        for label, arm in arms:
            try:
                arm.robot.connect()
                opened.append((label, arm))
            except Exception as exc:
                errors.append(f"{label} maintenance connect failed: {exc}")
        for label, arm in opened:
            try:
                results[label] = arm.robot.clear_errors(
                    joint_names=arm._active_joint_names()
                )
            except Exception as exc:
                errors.append(f"{label} motor fault clear failed: {exc}")
        for label, arm in reversed(opened):
            try:
                arm.robot.disconnect(disable=False)
            except Exception as exc:
                errors.append(f"{label} maintenance close failed: {exc}")
        if errors:
            raise RuntimeError("dual-arm maintenance failed: " + "; ".join(errors))
        return results["left"], results["right"]

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
            GripperCommand(
                motor=arm.robot._motor_map[arm.config.gripper.name],
                opening=value,
                speed=normalized_speed,
                force_level=int(level),
            )
            for arm, value in active
        )
        runtime.set_grippers(commands)


__all__ = ["ArxDCanDualArm", "ArxDCanDualArmState"]
