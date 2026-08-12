"""双臂 MIT 重力补偿。"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
import time

import numpy as np

from ...sdk import ArxDCanDualArm
from .single_arm import (
    GravityCompensationSample,
    GravityProvider,
    _GravityTorqueCalculator,
    _mode_name,
)


@dataclass(slots=True, frozen=True)
class DualArmGravityCompensationSample:
    """同一次原子双臂提交对应的左右补偿样本。"""

    left: GravityCompensationSample
    right: GravityCompensationSample


class DualArmGravityCompensationMode:
    """计算左右重力矩，并通过双臂 Runtime 原子提交 14 轴 MIT 目标。"""

    def __init__(
        self,
        robot: ArxDCanDualArm,
        *,
        hz: float | None = None,
        transition_seconds: float = 0.5,
        gravity_scale: float = 1.0,
        left_joint_scales: Sequence[float] | None = None,
        right_joint_scales: Sequence[float] | None = None,
        damping: float | Sequence[float] = 0.0,
        left_gravity_provider: GravityProvider | None = None,
        right_gravity_provider: GravityProvider | None = None,
    ) -> None:
        update_hz = (
            min(
                robot.left.config.feedback_check_hz,
                robot.right.config.feedback_check_hz,
            )
            if hz is None
            else float(hz)
        )
        if not math.isfinite(update_hz) or update_hz <= 0.0:
            raise ValueError("hz 必须是有限正数")
        command_timeout = min(
            robot.left.config.command_timeout_s,
            robot.right.config.command_timeout_s,
        )
        if 1.0 / update_hz >= command_timeout * 0.5:
            raise ValueError("hz 过低，无法在 Runtime 命令超时前稳定更新重力矩")
        if not math.isfinite(transition_seconds) or transition_seconds < 0.0:
            raise ValueError("transition_seconds 必须是有限非负数")
        self.robot = robot
        self.hz = float(update_hz)
        self.transition_seconds = float(transition_seconds)
        self._period = 1.0 / self.hz
        common = {
            "gravity_scale": gravity_scale,
            "damping": damping,
        }
        self._left = _GravityTorqueCalculator(
            robot.left,
            joint_scales=left_joint_scales,
            gravity_provider=left_gravity_provider,
            **common,
        )
        self._right = _GravityTorqueCalculator(
            robot.right,
            joint_scales=right_joint_scales,
            gravity_provider=right_gravity_provider,
            **common,
        )
        self._active = False
        self._owns_connection = False
        self._active_started = 0.0
        self._last_sample: DualArmGravityCompensationSample | None = None

    @property
    def active(self) -> bool:
        """返回双臂重力补偿是否已启动。"""
        return self._active

    @property
    def last_sample(self) -> DualArmGravityCompensationSample | None:
        """返回最近一次成功原子提交的左右样本。"""
        return self._last_sample

    def _checked_state(self, *, fresh: bool = False):
        health = self.robot.safety_health
        if health.safe_holding or health.fault_reason:
            raise RuntimeError(health.fault_reason or "双臂已进入安全保持")
        state = self.robot.read_state() if fresh else self.robot.read_cached_state()
        values = []
        for side, calculator in (
            (state.left, self._left),
            (state.right, self._right),
        ):
            positions = np.asarray(side.arm.positions, dtype=np.float64)
            velocities = np.asarray(side.arm.velocities, dtype=np.float64)
            if (
                len(positions) != calculator.joint_count
                or len(velocities) != calculator.joint_count
            ):
                raise RuntimeError("双臂反馈关节数量发生变化")
            if np.any(~np.isfinite(positions)) or np.any(~np.isfinite(velocities)):
                raise RuntimeError("双臂反馈包含非有限值")
            calculator.validate_positions(positions)
            values.append((positions, velocities))
        return values[0], values[1]

    def _submit(
        self,
        *,
        left_hold: np.ndarray,
        right_hold: np.ndarray,
        left_state: tuple[np.ndarray, np.ndarray],
        right_state: tuple[np.ndarray, np.ndarray],
        gravity_alpha: float,
    ) -> DualArmGravityCompensationSample:
        left_positions, left_velocities = left_state
        right_positions, right_velocities = right_state
        left_gravity, left_limited = self._left.compute(left_positions)
        right_gravity, right_limited = self._right.compute(right_positions)
        left_kp = (1.0 - gravity_alpha) * self._left.default_kp
        right_kp = (1.0 - gravity_alpha) * self._right.default_kp
        left_kd = (
            (1.0 - gravity_alpha) * self._left.default_kd
            + gravity_alpha * self._left.damping
        )
        right_kd = (
            (1.0 - gravity_alpha) * self._right.default_kd
            + gravity_alpha * self._right.damping
        )
        left_torques = gravity_alpha * left_gravity
        right_torques = gravity_alpha * right_gravity
        self.robot._submit_joint_positions(
            left=left_hold,
            right=right_hold,
            left_velocities=self._left.zeros,
            right_velocities=self._right.zeros,
            left_torques=left_torques,
            right_torques=right_torques,
            left_mit_kp=left_kp,
            right_mit_kp=right_kp,
            left_mit_kd=left_kd,
            right_mit_kd=right_kd,
        )
        elapsed = max(0.0, time.monotonic() - self._active_started)
        sample = DualArmGravityCompensationSample(
            left=GravityCompensationSample(
                elapsed_s=elapsed,
                positions=tuple(float(value) for value in left_positions),
                velocities=tuple(float(value) for value in left_velocities),
                commanded_torques=tuple(float(value) for value in left_torques),
                limited_joints=left_limited,
            ),
            right=GravityCompensationSample(
                elapsed_s=elapsed,
                positions=tuple(float(value) for value in right_positions),
                velocities=tuple(float(value) for value in right_velocities),
                commanded_torques=tuple(float(value) for value in right_torques),
                limited_joints=right_limited,
            ),
        )
        self._last_sample = sample
        return sample

    def _transition(
        self,
        left_hold: np.ndarray,
        right_hold: np.ndarray,
        *,
        entering: bool,
    ) -> None:
        steps = max(1, math.ceil(self.transition_seconds * self.hz))
        if self.transition_seconds == 0.0:
            steps = 1
        next_tick = time.monotonic()
        first = 0 if self.transition_seconds > 0.0 else 1
        for index in range(first, steps + 1):
            progress = index / steps
            alpha = progress if entering else 1.0 - progress
            left_state, right_state = self._checked_state()
            self._submit(
                left_hold=left_hold,
                right_hold=right_hold,
                left_state=left_state,
                right_state=right_state,
                gravity_alpha=alpha,
            )
            if index == steps:
                break
            next_tick += self._period
            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()

    def start(self) -> DualArmGravityCompensationSample:
        """连接双臂，在当前姿态使能，并平滑进入重力补偿。"""
        if self._active:
            raise RuntimeError("双臂重力补偿已经启动")
        if _mode_name(self.robot.left) != "mit" or _mode_name(self.robot.right) != "mit":
            raise RuntimeError(
                "双臂重力补偿要求创建机器人时设置 control_mode='mit'"
            )
        self._owns_connection = not self.robot.connected
        try:
            if self._owns_connection:
                self.robot.connect()
            if self.robot.enabled:
                raise RuntimeError("进入重力补偿前双臂必须处于失能状态")
            left_state, right_state = self._checked_state(fresh=True)
            left_position = left_state[0]
            right_position = right_state[0]
            self._left.compute(left_position)
            self._right.compute(right_position)
            self.robot.enable(
                left_initial_positions=left_position,
                right_initial_positions=right_position,
            )
            self._active = True
            self._active_started = time.monotonic()
            self._transition(left_position, right_position, entering=True)
            assert self._last_sample is not None
            return self._last_sample
        except Exception:
            self._active = False
            if self.robot.connected and self.robot.enabled:
                try:
                    self.robot.disable()
                except Exception:
                    pass
            if self._owns_connection and self.robot.connected:
                try:
                    self.robot.close()
                except Exception:
                    pass
            raise

    def step(self) -> DualArmGravityCompensationSample:
        """根据最新缓存原子更新一次左右臂重力前馈目标。"""
        if not self._active:
            raise RuntimeError("双臂重力补偿尚未启动")
        left_state, right_state = self._checked_state()
        return self._submit(
            left_hold=left_state[0],
            right_hold=right_state[0],
            left_state=left_state,
            right_state=right_state,
            gravity_alpha=1.0,
        )

    def run(self, *, seconds: float = 0.0) -> None:
        """持续更新双臂补偿；``seconds=0`` 时运行到用户中断。"""
        if not self._active:
            raise RuntimeError("双臂重力补偿尚未启动")
        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError("seconds 必须是有限非负数")
        deadline = None if seconds == 0.0 else time.monotonic() + seconds
        next_tick = time.monotonic()
        while deadline is None or time.monotonic() < deadline:
            self.step()
            next_tick += self._period
            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()

    def stop(self) -> None:
        """恢复双臂当前位置 MIT 保持、失能并释放自建连接。"""
        errors: list[Exception] = []
        try:
            if self._active and self.robot.connected and self.robot.enabled:
                health = self.robot.safety_health
                if not health.safe_holding and not health.fault_reason:
                    left_state, right_state = self._checked_state()
                    self._transition(
                        left_state[0],
                        right_state[0],
                        entering=False,
                    )
        except Exception as exc:
            errors.append(exc)
        finally:
            if self.robot.connected and self.robot.enabled:
                try:
                    self.robot.disable()
                except Exception as exc:
                    errors.append(exc)
            self._active = False
            if self._owns_connection and self.robot.connected:
                try:
                    self.robot.close()
                except Exception as exc:
                    errors.append(exc)
            self._owns_connection = False
        if errors:
            raise RuntimeError(f"停止双臂重力补偿失败：{errors[0]}") from errors[0]

    def shutdown(self) -> None:
        """兼容旧接口；等价于 :meth:`stop`。"""
        self.stop()

    def __enter__(self) -> "DualArmGravityCompensationMode":
        self.start()
        return self

    def __exit__(self, *_args) -> None:
        self.stop()


__all__ = [
    "DualArmGravityCompensationMode",
    "DualArmGravityCompensationSample",
]
