"""双臂 Runtime 原生重力补偿的生命周期兼容层。"""
from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
import time

from motor_drive_layer import GravityCompensationPhase

from ...sdk import ArxDCanDualArm
from .core import GravityCompensationSample


@dataclass(slots=True, frozen=True)
class DualArmGravityCompensationSample:
    """同一个 Runtime 状态快照对应的左右补偿样本。"""

    left: GravityCompensationSample
    right: GravityCompensationSample


def _is_zero(value: float | Sequence[float]) -> bool:
    values = (value,) if isinstance(value, (int, float)) else tuple(value)
    return all(float(item) == 0.0 for item in values)


class DualArmGravityCompensationMode:
    """管理双臂原生重力补偿，并提供旧示教接口所需的只读采样。

    控制周期、模型计算、渐入渐出和力矩限制全部由 Articore Runtime 执行。
    ``step()`` 只读取缓存反馈与 Runtime 状态，不再从 Python 提交 MIT 命令。
    """

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
        left_gravity_provider: Callable[..., object] | None = None,
        right_gravity_provider: Callable[..., object] | None = None,
    ) -> None:
        if not math.isfinite(transition_seconds) or transition_seconds < 0.0:
            raise ValueError("transition_seconds 必须是有限非负数")
        if not math.isfinite(gravity_scale) or gravity_scale != 1.0:
            raise ValueError("Runtime 原生重力补偿不支持 gravity_scale")
        if left_joint_scales is not None or right_joint_scales is not None:
            raise ValueError("Runtime 原生重力补偿不支持逐关节 gravity scale")
        if not _is_zero(damping):
            raise ValueError("Runtime 原生重力补偿当前不支持自定义拖拽阻尼")
        if left_gravity_provider is not None or right_gravity_provider is not None:
            raise ValueError("Runtime 原生重力补偿不接受 Python gravity provider")

        self.robot = robot
        self.transition_seconds = float(transition_seconds)
        self._requested_hz = None if hz is None else float(hz)
        initial_hz = robot._effective_control_hz if hz is None else float(hz)
        self._set_update_hz(initial_hz)
        self._active = False
        self._owns_connection = False
        self._active_started = 0.0
        self._last_sample: DualArmGravityCompensationSample | None = None

    def _set_update_hz(self, value: float) -> None:
        update_hz = float(value)
        if not math.isfinite(update_hz) or update_hz <= 0.0:
            raise ValueError("hz 必须是有限正数")
        self._update_hz = update_hz
        self._period = 1.0 / update_hz

    @property
    def active(self) -> bool:
        """返回该兼容层是否已启动原生重力补偿。"""
        return self._active

    @property
    def last_sample(self) -> DualArmGravityCompensationSample | None:
        return self._last_sample

    @staticmethod
    def _torques_for_arm(status, arm, *, start_index: int) -> tuple[float, ...]:
        by_motor = {
            id(motor): float(torque)
            for motor, torque in zip(
                status.joints,
                status.gravity_feedforward_torque,
            )
        }
        motors = tuple(
            arm.robot._motor_map[joint.name] for joint in arm.config.arm_joints
        )
        if motors and all(id(motor) in by_motor for motor in motors):
            return tuple(by_motor[id(motor)] for motor in motors)
        count = len(arm.joint_names)
        values = status.gravity_feedforward_torque[start_index:start_index + count]
        return tuple(float(value) for value in values)

    def _sample(self) -> DualArmGravityCompensationSample:
        health = self.robot.safety_health
        if health.safe_holding or health.fault_reason:
            raise RuntimeError(health.fault_reason or "双臂已进入安全保持")
        state = self.robot.read_cached_state()
        status = self.robot.gravity_compensation_status
        left_count = len(self.robot.left.joint_names)
        left_positions = tuple(float(value) for value in state.left.arm.positions)
        left_velocities = tuple(float(value) for value in state.left.arm.velocities)
        right_positions = tuple(float(value) for value in state.right.arm.positions)
        right_velocities = tuple(float(value) for value in state.right.arm.velocities)
        vectors = (
            ("left positions", left_positions, left_count),
            ("left velocities", left_velocities, left_count),
            ("right positions", right_positions, len(self.robot.right.joint_names)),
            ("right velocities", right_velocities, len(self.robot.right.joint_names)),
        )
        for name, values, expected_count in vectors:
            if len(values) != expected_count or not all(
                math.isfinite(value) for value in values
            ):
                raise RuntimeError(f"invalid gravity compensation {name}")
        elapsed = max(0.0, time.monotonic() - self._active_started)
        sample = DualArmGravityCompensationSample(
            left=GravityCompensationSample(
                elapsed_s=elapsed,
                positions=left_positions,
                velocities=left_velocities,
                commanded_torques=self._torques_for_arm(
                    status,
                    self.robot.left,
                    start_index=0,
                ),
            ),
            right=GravityCompensationSample(
                elapsed_s=elapsed,
                positions=right_positions,
                velocities=right_velocities,
                commanded_torques=self._torques_for_arm(
                    status,
                    self.robot.right,
                    start_index=left_count,
                ),
            ),
        )
        self._last_sample = sample
        return sample

    def start(self) -> DualArmGravityCompensationSample:
        """连接并使能双臂，然后由 Runtime 平滑进入重力补偿。"""
        if self._active:
            raise RuntimeError("双臂重力补偿已经启动")
        if self.robot.left._mode != "mit" or self.robot.right._mode != "mit":
            raise RuntimeError(
                "双臂重力补偿要求创建机器人时设置 control_mode='mit'"
            )
        self._owns_connection = not self.robot.connected
        try:
            if self._owns_connection:
                self.robot.connect()
            if self._requested_hz is None:
                self._set_update_hz(self.robot._effective_control_hz)
            if self.robot.enabled:
                raise RuntimeError("进入重力补偿前双臂必须处于失能状态")
            self.robot.enable()
            transition_ms = max(1, round(self.transition_seconds * 1000.0))
            self.robot.start_gravity_compensation(transition_ms=transition_ms)
            self._active = True
            self._active_started = time.monotonic()
            return self._sample()
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
        """读取一次反馈和 Runtime 实际发送的重力前馈。"""
        if not self._active:
            raise RuntimeError("双臂重力补偿尚未启动")
        return self._sample()

    def run(self, *, seconds: float = 0.0) -> None:
        """保持进程并监测状态；控制循环始终在 Runtime 内运行。"""
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

    def _wait_until_inactive(self) -> None:
        deadline = time.monotonic() + max(2.0, self.transition_seconds + 1.0)
        while self.robot.gravity_compensation_status.phase is not GravityCompensationPhase.INACTIVE:
            if time.monotonic() >= deadline:
                raise RuntimeError("等待 Runtime 退出重力补偿超时")
            time.sleep(min(0.02, self._period))

    def stop(self) -> None:
        """平滑退出重力补偿，失能双臂并释放自建连接。"""
        errors: list[Exception] = []
        try:
            if self._active and self.robot.connected and self.robot.enabled:
                self.robot.stop_gravity_compensation()
                self._wait_until_inactive()
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
