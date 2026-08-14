"""双臂重力补偿共用的模型计算实现。"""
from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math

import numpy as np

from ...driver import damiao_model_limits
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
    clipped_joints: tuple[str, ...] = ()


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
        margin = float(arm.config.soft_limit_margin)
        if not math.isfinite(margin) or margin < 0.0:
            raise ValueError("soft_limit_margin 必须是有限非负数")
        lower_command_limits = []
        upper_command_limits = []
        for joint in arm.config.arm_joints:
            position_range, _, _ = damiao_model_limits(joint.model)
            hard_lower = (
                -position_range if joint.lower_limit is None else joint.lower_limit
            )
            hard_upper = (
                position_range if joint.upper_limit is None else joint.upper_limit
            )
            soft_lower = hard_lower + margin
            soft_upper = hard_upper - margin
            if soft_lower >= soft_upper:
                raise ValueError(
                    f"{joint.name}: soft limit margin leaves no valid joint range"
                )
            lower_command_limits.append(soft_lower)
            upper_command_limits.append(soft_upper)
        self.lower_command_limits = np.asarray(
            lower_command_limits,
            dtype=np.float64,
        )
        self.upper_command_limits = np.asarray(
            upper_command_limits,
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

    def clip_hold_position(
        self,
        positions: np.ndarray,
    ) -> tuple[np.ndarray, tuple[str, ...]]:
        """将反馈生成的保持目标裁剪到 Runtime 软命令限位内。"""
        values = np.asarray(positions, dtype=np.float64).reshape(-1)
        if len(values) != self.joint_count or np.any(~np.isfinite(values)):
            raise RuntimeError("重力补偿保持目标无效")
        clipped = np.clip(
            values,
            self.lower_command_limits,
            self.upper_command_limits,
        )
        clipped_joints = tuple(
            name
            for name, expected, actual in zip(
                self.arm.joint_names,
                values,
                clipped,
            )
            if not math.isclose(float(expected), float(actual), abs_tol=1e-12)
        )
        return clipped, clipped_joints


__all__: list[str] = []
