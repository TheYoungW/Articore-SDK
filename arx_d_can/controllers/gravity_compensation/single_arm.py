"""单臂 MIT 重力补偿。"""
from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
import time

import numpy as np

from ...sdk import ArxDCanArm


GravityProvider = Callable[[np.ndarray], np.ndarray]


@dataclass(slots=True, frozen=True)
class GravityCompensationSample:
    """一帧重力补偿反馈和实际提交的力矩。"""

    elapsed_s: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    commanded_torques: tuple[float, ...]
    limited_joints: tuple[str, ...] = ()


def _vector(
    value: float | Sequence[float],
    *,
    joint_count: int,
    name: str,
    strictly_positive: bool = False,
) -> np.ndarray:
    """把标量或逐关节参数规范为有限浮点向量。"""
    result = np.asarray(value, dtype=np.float64)
    if result.ndim == 0:
        result = np.full(joint_count, float(result), dtype=np.float64)
    else:
        result = result.reshape(-1)
    if len(result) != joint_count:
        raise ValueError(f"{name} 必须包含 {joint_count} 个值")
    if np.any(~np.isfinite(result)):
        raise ValueError(f"{name} 必须全部为有限值")
    if strictly_positive:
        if np.any(result <= 0.0):
            raise ValueError(f"{name} 必须全部大于 0")
    elif np.any(result < 0.0):
        raise ValueError(f"{name} 不能为负数")
    return result


def _mode_name(arm: ArxDCanArm) -> str:
    """返回构造机械臂时确定的控制模式。"""
    return str(getattr(arm, "_mode", arm.config.arm_control_mode)).lower()


class _GravityTorqueCalculator:
    """只负责模型加载、重力矩计算和产品力矩限幅。"""

    def __init__(
        self,
        arm: ArxDCanArm,
        *,
        gravity_scale: float,
        joint_scales: Sequence[float] | None,
        damping: float | Sequence[float],
        gravity_provider: GravityProvider | None,
    ) -> None:
        self.arm = arm
        self.joint_count = len(arm.joint_names)
        if self.joint_count == 0:
            raise ValueError("重力补偿至少需要一个关节")
        if not math.isfinite(gravity_scale) or gravity_scale <= 0.0:
            raise ValueError("gravity_scale 必须是有限正数")
        self.gravity_scale = float(gravity_scale)
        self.joint_scales = _vector(
            1.0 if joint_scales is None else joint_scales,
            joint_count=self.joint_count,
            name="joint_scales",
            strictly_positive=True,
        )
        self.damping = _vector(
            damping,
            joint_count=self.joint_count,
            name="damping",
        )
        self.zeros = np.zeros(self.joint_count, dtype=np.float64)
        self.default_kp = np.asarray(
            [joint.mit_kp for joint in arm.config.arm_joints],
            dtype=np.float64,
        )
        self.default_kd = np.asarray(
            [joint.mit_kd for joint in arm.config.arm_joints],
            dtype=np.float64,
        )
        self.effort_limits = np.asarray(
            [
                math.inf if joint.effort_limit is None else joint.effort_limit
                for joint in arm.config.arm_joints
            ],
            dtype=np.float64,
        )
        self.provider = gravity_provider or self._build_provider()

    def _build_provider(self) -> GravityProvider:
        if self.arm.config.urdf_path is None:
            raise ValueError("重力补偿需要配置 URDF 模型")
        try:
            from ...dynamics import compute_generalized_gravity
            from ...kinematics import load_robot_model
        except (ImportError, ModuleNotFoundError) as exc:
            raise RuntimeError(
                "重力补偿需要 Pinocchio，请安装 dynamics 可选依赖"
            ) from exc

        model = load_robot_model(
            self.arm.config.urdf_path,
            controlled_joint_names=self.arm.joint_names,
        )
        data = model.createData()

        def compute(positions: np.ndarray) -> np.ndarray:
            return compute_generalized_gravity(model, positions, data)

        return compute

    def compute(self, positions: np.ndarray) -> tuple[np.ndarray, tuple[str, ...]]:
        """计算逻辑关节力矩，并按 URDF effort 限制安全限幅。"""
        raw = np.asarray(self.provider(positions), dtype=np.float64).reshape(-1)
        if len(raw) != self.joint_count:
            raise RuntimeError(
                f"重力模型返回 {len(raw)} 个力矩，期望 {self.joint_count} 个"
            )
        requested = self.gravity_scale * self.joint_scales * raw
        if np.any(~np.isfinite(requested)):
            raise RuntimeError("重力模型返回了非有限力矩")
        limited = np.clip(requested, -self.effort_limits, self.effort_limits)
        limited_joints = tuple(
            name
            for name, expected, actual in zip(
                self.arm.joint_names,
                requested,
                limited,
            )
            if not math.isclose(float(expected), float(actual), abs_tol=1e-9)
        )
        return limited, limited_joints

class GravityCompensationMode:
    """使用 MIT 前馈力矩抵消单臂重力。

    机械臂必须在创建时选择 ``control_mode="mit"``。Python 按产品控制频率读取
    每次 MIT 发送所更新的原生反馈缓存并计算重力矩；通信监控、故障保持和夹爪
    防堵转均由 motor 原生 Runtime 执行。
    """

    def __init__(
        self,
        arm: ArxDCanArm,
        *,
        hz: float | None = None,
        transition_seconds: float = 0.5,
        gravity_scale: float = 1.0,
        joint_scales: Sequence[float] | None = None,
        damping: float | Sequence[float] = 0.0,
        gravity_provider: GravityProvider | None = None,
    ) -> None:
        update_hz = arm.config.control_hz if hz is None else float(hz)
        if not math.isfinite(update_hz) or update_hz <= 0.0:
            raise ValueError("hz 必须是有限正数")
        if 1.0 / update_hz >= arm.config.command_timeout_s * 0.5:
            raise ValueError("hz 过低，无法在 Runtime 命令超时前稳定更新重力矩")
        if not math.isfinite(transition_seconds) or transition_seconds < 0.0:
            raise ValueError("transition_seconds 必须是有限非负数")
        self.arm = arm
        self.hz = float(update_hz)
        self.transition_seconds = float(transition_seconds)
        self._period = 1.0 / self.hz
        self._calculator = _GravityTorqueCalculator(
            arm,
            gravity_scale=gravity_scale,
            joint_scales=joint_scales,
            damping=damping,
            gravity_provider=gravity_provider,
        )
        self._active = False
        self._owns_connection = False
        self._active_started = 0.0
        self._last_sample: GravityCompensationSample | None = None

    @property
    def active(self) -> bool:
        """返回重力补偿是否已启动。"""
        return self._active

    @property
    def last_sample(self) -> GravityCompensationSample | None:
        """返回最近一次成功提交的补偿样本。"""
        return self._last_sample

    def _checked_state(self, *, fresh: bool = False):
        if self.arm.faulted or self.arm.safe_holding:
            raise RuntimeError(self.arm.fault_reason or "机械臂已进入安全保持")
        state = self.arm.read_state() if fresh else self.arm.read_cached_state()
        positions = np.asarray(state.arm.positions, dtype=np.float64)
        velocities = np.asarray(state.arm.velocities, dtype=np.float64)
        count = self._calculator.joint_count
        if len(positions) != count or len(velocities) != count:
            raise RuntimeError("机械臂反馈关节数量发生变化")
        if np.any(~np.isfinite(positions)) or np.any(~np.isfinite(velocities)):
            raise RuntimeError("机械臂反馈包含非有限值")
        return positions, velocities

    def _submit(
        self,
        *,
        hold_position: np.ndarray,
        positions: np.ndarray,
        velocities: np.ndarray,
        gravity_alpha: float,
    ) -> GravityCompensationSample:
        gravity, limited_joints = self._calculator.compute(positions)
        kp = (1.0 - gravity_alpha) * self._calculator.default_kp
        kd = (
            (1.0 - gravity_alpha) * self._calculator.default_kd
            + gravity_alpha * self._calculator.damping
        )
        torques = gravity_alpha * gravity
        self.arm._submit_joint_positions(
            hold_position,
            velocities=self._calculator.zeros,
            torques=torques,
            mit_kp=kp,
            mit_kd=kd,
            enforce_position_limits=False,
        )
        sample = GravityCompensationSample(
            elapsed_s=max(0.0, time.monotonic() - self._active_started),
            positions=tuple(float(value) for value in positions),
            velocities=tuple(float(value) for value in velocities),
            commanded_torques=tuple(float(value) for value in torques),
            limited_joints=limited_joints,
        )
        self._last_sample = sample
        return sample

    def _transition(self, hold_position: np.ndarray, *, entering: bool) -> None:
        steps = max(1, math.ceil(self.transition_seconds * self.hz))
        if self.transition_seconds == 0.0:
            steps = 1
        next_tick = time.monotonic()
        first = 0 if self.transition_seconds > 0.0 else 1
        for index in range(first, steps + 1):
            progress = index / steps
            alpha = progress if entering else 1.0 - progress
            positions, velocities = self._checked_state()
            self._submit(
                hold_position=hold_position,
                positions=positions,
                velocities=velocities,
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

    def start(self) -> GravityCompensationSample:
        """连接、在当前位置使能，并平滑进入重力补偿。"""
        if self._active:
            raise RuntimeError("重力补偿已经启动")
        if _mode_name(self.arm) != "mit":
            raise RuntimeError(
                "重力补偿要求创建机械臂时设置 control_mode='mit'"
            )
        self._owns_connection = not self.arm.connected
        try:
            if self._owns_connection:
                self.arm.connect()
            if self.arm.enabled:
                raise RuntimeError("进入重力补偿前机械臂必须处于失能状态")
            initial_position, _ = self._checked_state(fresh=True)
            # 使能前先验证模型输出，避免错误模型在电机使能后才暴露。
            self._calculator.compute(initial_position)
            self.arm.enable()
            self._active = True
            self._active_started = time.monotonic()
            self._transition(initial_position, entering=True)
            assert self._last_sample is not None
            return self._last_sample
        except Exception:
            self._active = False
            if self.arm.connected and self.arm.enabled:
                try:
                    self.arm.disable()
                except Exception:
                    pass
            if self._owns_connection and self.arm.connected:
                try:
                    self.arm.close()
                except Exception:
                    pass
            raise

    def step(self) -> GravityCompensationSample:
        """根据最新原生反馈缓存更新一次重力前馈目标。"""
        if not self._active:
            raise RuntimeError("重力补偿尚未启动")
        positions, velocities = self._checked_state()
        return self._submit(
            hold_position=positions,
            positions=positions,
            velocities=velocities,
            gravity_alpha=1.0,
        )

    def run(
        self,
        *,
        seconds: float = 0.0,
        on_sample: Callable[[GravityCompensationSample], None] | None = None,
    ) -> None:
        """持续更新补偿目标；``seconds=0`` 时运行到用户中断。"""
        if not self._active:
            raise RuntimeError("重力补偿尚未启动")
        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError("seconds 必须是有限非负数")
        deadline = None if seconds == 0.0 else time.monotonic() + seconds
        next_tick = time.monotonic()
        while deadline is None or time.monotonic() < deadline:
            sample = self.step()
            if on_sample is not None:
                on_sample(sample)
            next_tick += self._period
            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()

    def stop(self) -> None:
        """恢复当前位置 MIT 保持、失能，并关闭本对象建立的连接。"""
        errors: list[Exception] = []
        try:
            if (
                self._active
                and self.arm.connected
                and self.arm.enabled
                and not self.arm.faulted
                and not self.arm.safe_holding
            ):
                hold_position, _ = self._checked_state()
                self._transition(hold_position, entering=False)
        except Exception as exc:
            errors.append(exc)
        finally:
            if self.arm.connected and self.arm.enabled:
                try:
                    self.arm.disable()
                except Exception as exc:
                    errors.append(exc)
            self._active = False
            if self._owns_connection and self.arm.connected:
                try:
                    self.arm.close()
                except Exception as exc:
                    errors.append(exc)
            self._owns_connection = False
        if errors:
            raise RuntimeError(f"停止重力补偿失败：{errors[0]}") from errors[0]

    def shutdown(self) -> None:
        """兼容旧接口；等价于 :meth:`stop`。"""
        self.stop()

    def __enter__(self) -> "GravityCompensationMode":
        self.start()
        return self

    def __exit__(self, *_args) -> None:
        self.stop()


__all__ = ["GravityCompensationMode", "GravityCompensationSample"]
