"""两条独立 CAN 通道组成的双臂控制接口。"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Sequence

from ..driver import CallError, ControllerGroup
from ..errors import TransportError
from .arm import ArxDCanArm
from .native_safety import (
    GripperSafetyHealth,
    NativeMotorDescriptor,
    NativeSafetyRuntime,
    SafetyHealth,
    SafetyState,
    TransportHealth,
)
from .state import ArxDCanState


def _positions(
    values: Sequence[float],
    *,
    expected_count: int,
    name: str,
) -> tuple[float, ...]:
    positions = tuple(float(value) for value in values)
    if len(positions) != expected_count:
        raise ValueError(
            f"{name} must contain exactly {expected_count} joint positions"
        )
    if any(not math.isfinite(value) for value in positions):
        raise ValueError(f"{name} must contain only finite values")
    return positions


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
    ``left_model`` 和 ``right_model``，无需新增产品专用 Python 类。
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
        control_mode: str = "pv",
        left_gripper: bool | None = None,
        right_gripper: bool | None = None,
    ) -> None:
        normalized_mode = str(control_mode).strip().lower()
        if normalized_mode not in ("pv", "mit"):
            raise ValueError("control_mode must be 'pv' or 'mit'")
        if transport == "dm-device":
            left_channel = "0" if left_channel is None else left_channel
            right_channel = "1" if right_channel is None else right_channel

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

    def _native_motor_descriptors(self) -> tuple[NativeMotorDescriptor, ...]:
        descriptors = []
        for side, label, arm in ((0, "left", self.left), (1, "right", self.right)):
            for joint in arm.config.arm_joints:
                descriptors.append(
                    NativeMotorDescriptor(
                        motor=arm.robot._motor_map[joint.name],
                        side=side,
                        name=f"{label}/{joint.name}",
                        safe_kp=arm.config.safe_hold_mit_kp,
                        safe_kd=arm.config.safe_hold_mit_kd,
                    )
                )
            if arm.enable_gripper and arm.config.gripper is not None:
                gripper = arm.config.gripper
                force = arm.config.gripper_force_control
                closed = arm.config.gripper_closed_value
                opened = arm.config.gripper_open_value
                descriptors.append(
                    NativeMotorDescriptor(
                        motor=arm.robot._motor_map[gripper.name],
                        side=side,
                        name=f"{label}/{gripper.name}",
                        is_gripper=True,
                        safe_kp=force.hold_kp,
                        safe_kd=force.hold_kd,
                        overload_torque=force.overload_torque,
                        retreat_distance=force.retreat_distance,
                        contact_torque=force.contact_torque,
                        motion_window_s=force.motion_window_s,
                        stall_movement=force.stall_movement,
                        min_position_error=force.min_position_error,
                        contact_hold_s=force.contact_hold_s,
                        overload_hold_s=force.overload_hold_s,
                        hold_offset=force.hold_offset,
                        retreat_retry_s=force.overload_retreat_interval_s,
                        open_position=opened,
                        closed_position=closed,
                        normal_kp=gripper.mit_kp,
                        normal_kd=gripper.mit_kd,
                        close_speed=force.close_speed,
                        max_step_interval_s=force.max_step_interval_s,
                        closing_direction=1.0 if closed > opened else -1.0,
                        lower_position=min(closed, opened),
                        upper_position=max(closed, opened),
                    )
                )
        return tuple(descriptors)

    def _create_safety_runtime(
        self,
        group: ControllerGroup,
        left_controller: object,
        right_controller: object,
    ) -> NativeSafetyRuntime | None:
        # Test doubles and third-party ControllerGroup-compatible objects do
        # not expose native handles. Real motor-drive-layer objects always do.
        if not (
            self.left._supports_parallel_joint_batch()
            and self.right._supports_parallel_joint_batch()
        ):
            return None
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
            gripper_control_hz=min(
                left.gripper_control_hz, right.gripper_control_hz
            ),
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
                if not active:
                    arm._watchdog_deadline = None

    def connect(self) -> None:
        """连接左右臂，并为两条通道创建常驻并行发送线程。"""
        if self.connected and self._controller_group is not None:
            return
        self.left.connect()
        try:
            self.right.connect()
            left_controller = self.left._controller_for_parallel_batch()
            right_controller = self.right._controller_for_parallel_batch()
            group = ControllerGroup([left_controller, right_controller])
            self._controller_group = group
            self._safety_runtime = self._create_safety_runtime(
                group, left_controller, right_controller
            )
            if self._safety_runtime is not None:
                self.left._set_dual_runtime_managed(True)
                self.right._set_dual_runtime_managed(True)
        except Exception:
            if self._safety_runtime is not None:
                self._safety_runtime.close()
                self._safety_runtime = None
            if self._controller_group is not None:
                self._controller_group.close()
                self._controller_group = None
            if self.right.connected:
                self.right.close(disable=False)
            self.left.close(disable=False)
            raise

    def enable(self) -> None:
        """按构造时确定的模式配置并使能双臂。"""
        self.left.enable()
        try:
            self.right.enable()
            runtime = self._safety_runtime
            if runtime is not None:
                self.left._stop_watchdog()
                self.right._stop_watchdog()
                runtime.enable(self.left._mode)
        except Exception:
            runtime = self._safety_runtime
            if runtime is not None:
                try:
                    runtime.estop("dual-arm enable failed")
                except Exception:
                    pass
            else:
                self.left.disable()
            raise

    def disable(self) -> None:
        """失能左右臂；一侧失败时仍继续处理另一侧。"""
        runtime = self._safety_runtime
        if runtime is not None:
            for arm in (self.left, self.right):
                arm._stop_coupled_control()
                arm._stop_watchdog()
            try:
                runtime.disable()
            finally:
                self._sync_python_safety_flags(runtime.health)
            return
        errors: list[Exception] = []
        for arm in (self.left, self.right):
            if not arm.connected:
                continue
            try:
                arm.disable()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError("failed to disable one or more arms") from errors[0]

    def close(self, *, disable: bool = True) -> None:
        """释放并行发送线程后关闭左右通道。"""
        errors: list[Exception] = []
        runtime = self._safety_runtime
        self._safety_runtime = None
        self.left._set_gripper_command_sink(None)
        self.right._set_gripper_command_sink(None)
        self.left._set_dual_runtime_managed(False)
        self.right._set_dual_runtime_managed(False)
        if runtime is not None:
            try:
                runtime.close()
            except Exception as exc:
                errors.append(exc)
        group = self._controller_group
        self._controller_group = None
        if group is not None:
            try:
                group.close()
            except Exception as exc:
                errors.append(exc)
        for arm in (self.left, self.right):
            if not arm.connected:
                continue
            try:
                arm.close(disable=disable)
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

    def _targets(
        self,
        left: Sequence[float],
        right: Sequence[float],
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """在发送前同时校验左右臂命令，避免长度错误造成部分发送。"""
        return (
            _positions(left, expected_count=len(self.left.joint_names), name="left"),
            _positions(
                right,
                expected_count=len(self.right.joint_names),
                name="right",
            ),
        )

    def send_joint_positions(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
    ) -> None:
        """在同一批次内并行发送左右臂完整关节目标。"""
        if self._safety_runtime is not None:
            self._sync_python_safety_flags(self._safety_runtime.health)
        left_target, right_target = self._targets(left, right)
        if not (
            self.left._supports_parallel_joint_batch()
            and self.right._supports_parallel_joint_batch()
        ):
            # 带耦合关节变换的 MIT 机型使用独立内环，不能绕过其
            # 反馈与力矩变换直接交给 ControllerGroup。
            self.left.send_joint_positions(left_target)
            self.right.send_joint_positions(right_target)
            return

        group = self._controller_group
        if group is None:
            # 未连接时保持单臂接口原有的错误语义。
            self.left.send_joint_positions(left_target)
            self.right.send_joint_positions(right_target)
            return

        left_batch = self.left._prepare_parallel_joint_positions(left_target)
        if left_batch is None:
            return
        right_batch = self.right._prepare_parallel_joint_positions(right_target)
        if right_batch is None:
            return
        if left_batch.mode != right_batch.mode:
            raise RuntimeError("left and right arms must use the same control mode")

        commands = left_batch.commands + right_batch.commands
        try:
            with self.left._io_lock, self.right._io_lock:
                runtime = self._safety_runtime
                if left_batch.mode == "pv":
                    if runtime is None:
                        group.send_pos_vel(commands)
                    else:
                        runtime.submit_pos_vel(commands)
                else:
                    if runtime is None:
                        group.send_mit(commands)
                    else:
                        runtime.submit_mit(commands)
        except Exception as exc:
            error: Exception = exc
            if isinstance(exc, CallError):
                error = TransportError(
                    f"parallel joint send failed: {exc}",
                    operation=f"send_{left_batch.mode}",
                    motor_names=self.left.joint_names + self.right.joint_names,
                    retryable=True,
                )
            if self._safety_runtime is not None:
                self._sync_python_safety_flags(self._safety_runtime.health)
                handled = (False, False)
            else:
                handled = (
                    self.left._handle_joint_command_failure(error),
                    self.right._handle_joint_command_failure(error),
                )
            if all(handled):
                return
            if error is exc:
                raise
            raise error from exc

        self.left._complete_parallel_joint_positions(left_batch)
        self.right._complete_parallel_joint_positions(right_batch)

    def estop(self, reason: str = "emergency stop") -> None:
        """锁存双臂故障并尝试失能所有关节和夹爪。"""
        runtime = self._safety_runtime
        if runtime is None:
            errors = []
            for arm in (self.left, self.right):
                try:
                    arm.disable()
                except Exception as exc:
                    errors.append(exc)
            if errors:
                raise RuntimeError("dual-arm emergency stop failed") from errors[0]
            return
        runtime.estop(reason)
        self._sync_python_safety_flags(runtime.health)

    def recover(self) -> None:
        """验证双通道、反馈、故障码和物理失能后只恢复到 READY。"""
        runtime = self._safety_runtime
        if runtime is None:
            self.left.clear_fault()
            self.right.clear_fault()
            return
        runtime.recover()
        self._sync_python_safety_flags(runtime.health)

    def move_joint_positions(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        seconds: float = 3.0,
        profile: str = "min_jerk",
    ) -> ArxDCanDualArmState:
        """使用同一时间轴对左右臂进行关节空间插值。"""
        from ..trajectory import plan_joint_position_trajectory

        left_target, right_target = self._targets(left, right)
        duration = float(seconds)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("seconds must be finite and positive")
        command_hz = min(self.left.config.control_hz, self.right.config.control_hz)
        initial = self.read_state()
        left_points = plan_joint_position_trajectory(
            initial.left.arm.positions,
            left_target,
            duration=duration,
            hz=command_hz,
            profile=profile,
        )
        right_points = plan_joint_position_trajectory(
            initial.right.arm.positions,
            right_target,
            duration=duration,
            hz=command_hz,
            profile=profile,
        )
        started = time.monotonic()
        for left_point, right_point in zip(left_points, right_points):
            remaining = started + left_point.time - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            self.send_joint_positions(
                left=left_point.positions,
                right=right_point.positions,
            )
        return self.read_state()

    def hold_joint_positions(
        self,
        *,
        left: Sequence[float],
        right: Sequence[float],
        seconds: float | None = None,
    ) -> ArxDCanDualArmState:
        """按照机型控制频率持续保持左右臂目标。"""
        left_target, right_target = self._targets(left, right)
        if seconds is not None and (
            not math.isfinite(seconds) or seconds < 0.0
        ):
            raise ValueError("seconds must be finite and non-negative or None")
        command_hz = min(self.left.config.control_hz, self.right.config.control_hz)
        period = 1.0 / command_hz
        started = time.monotonic()
        cycle = 0
        while seconds is None or time.monotonic() - started < seconds:
            self.send_joint_positions(left=left_target, right=right_target)
            cycle += 1
            remaining = started + cycle * period - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
        return self.read_state()

    def move_grippers(
        self,
        *,
        left: float,
        right: float,
        seconds: float = 2.0,
    ) -> ArxDCanDualArmState:
        """按同一节拍控制左右夹爪，并返回最终双臂状态。"""
        if not self.left.has_gripper or not self.right.has_gripper:
            raise RuntimeError("both arms must configure an active gripper")
        duration = float(seconds)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("seconds must be finite and positive")
        command_hz = min(self.left.config.control_hz, self.right.config.control_hz)
        period = 1.0 / command_hz
        started = time.monotonic()
        cycle = 0
        while time.monotonic() - started < duration:
            self.set_grippers(left=left, right=right)
            cycle += 1
            remaining = started + cycle * period - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
        return self.read_state()

    def set_grippers(self, *, left: float, right: float) -> None:
        """原子提交左右夹爪的 0～1000 开合度目标。"""
        values = (float(left), float(right))
        if any(not math.isfinite(value) for value in values):
            raise ValueError("gripper openings must be finite")
        values = tuple(max(0.0, min(1000.0, value)) for value in values)
        active = tuple(
            (arm, value)
            for arm, value in zip((self.left, self.right), values)
            if arm.has_gripper
        )
        if not active:
            raise RuntimeError("no active product gripper is configured")
        runtime = self._safety_runtime
        if runtime is None:
            for arm, value in active:
                arm.set_gripper(value)
            return
        targets = tuple(
            (arm.robot._motor_map[arm.config.gripper.name], value)
            for arm, value in active
        )
        runtime.set_gripper_openings(targets)

    def open_grippers(self) -> None:
        """完全张开所有已启用的产品夹爪。"""
        self.set_grippers(left=1000.0, right=1000.0)

    def close_grippers(self) -> None:
        """完全闭合所有已启用的产品夹爪。"""
        self.set_grippers(left=0.0, right=0.0)

    def record_trajectory(
        self,
        path: str | Path,
        *,
        seconds: float = 10.0,
        hz: float = 100.0,
    ) -> int:
        """在双臂失能状态下录制左右关节和夹爪轨迹。"""
        if self.left.enabled or self.right.enabled:
            raise RuntimeError("disable both arms before recording a trajectory")
        from ..service_tools.dual_trajectory_recording import record, save_trajectory

        timestamps, samples = record(self, seconds=seconds, hz=hz)
        save_trajectory(
            Path(path),
            hz=hz,
            timestamps=timestamps,
            samples=samples,
            left_joint_names=self.left.joint_names,
            right_joint_names=self.right.joint_names,
        )
        return len(samples)

    def replay_trajectory(self, path: str | Path) -> int:
        """按照录制时间戳回放一条双臂轨迹。"""
        if not self.enabled:
            raise RuntimeError("enable both arms before replaying a trajectory")
        from ..service_tools.dual_trajectory_recording import load_trajectory, replay

        timestamps, samples = load_trajectory(
            Path(path),
            expected_left_joint_names=self.left.joint_names,
            expected_right_joint_names=self.right.joint_names,
        )
        replay(self, timestamps=timestamps, samples=samples)
        return len(samples)


__all__ = ["ArxDCanDualArm", "ArxDCanDualArmState"]
