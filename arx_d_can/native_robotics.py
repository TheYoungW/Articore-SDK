"""Thin NumPy facade over the private Articore Runtime robot model ABI."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from arx_d_can._motor_abi import (
    IkOptions as RuntimeIkOptions,
    JacobianReference,
    NativeRobotModel,
    RobotPose as RuntimeRobotPose,
    RobotSide,
)


_PRODUCTS = {
    "yunyi_v1_0": ("yunyi_v1_0", None),
    "yunyi_v1_0_left": ("yunyi_v1_0", RobotSide.LEFT),
    "yunyi_v1_0_right": ("yunyi_v1_0", RobotSide.RIGHT),
}


@dataclass(slots=True, frozen=True)
class NativeIkResult:
    q: np.ndarray
    success: bool
    error: float
    iterations: int


class NativeArmModel:
    """Product-owned seven-axis model; no Python Pinocchio dependency."""

    def __init__(
        self,
        product: str = "yunyi_v1_0",
        side: str | RobotSide | int | None = None,
    ) -> None:
        try:
            product_id, configured_side = _PRODUCTS[product]
        except KeyError as exc:
            raise ValueError(f"unsupported native robot product: {product}") from exc
        if side is None:
            if configured_side is None:
                raise ValueError("side is required for the dual-arm yunyi_v1_0 product")
            selected_side = configured_side
        elif isinstance(side, str):
            selected_side = RobotSide.LEFT if side.lower() == "left" else (
                RobotSide.RIGHT if side.lower() == "right" else None
            )
            if selected_side is None:
                raise ValueError("side must be 'left' or 'right'")
        else:
            selected_side = RobotSide(side)
        if configured_side is not None and selected_side != configured_side:
            raise ValueError(f"{product} cannot be loaded for {selected_side.name.lower()} side")
        self._native = NativeRobotModel(product_id, selected_side)

    @property
    def dof(self) -> int:
        return self._native.info.dof

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._native.info.joint_names

    @property
    def lower_position_limits(self) -> np.ndarray:
        return np.asarray(self._native.info.lower_limits, dtype=np.float64)

    @property
    def upper_position_limits(self) -> np.ndarray:
        return np.asarray(self._native.info.upper_limits, dtype=np.float64)

    @property
    def end_effector_frame(self) -> str:
        return self._native.info.end_effector_frame

    def fk(self, q: Sequence[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        pose = self._native.fk(q)
        return (
            np.asarray(pose.position, dtype=np.float64),
            np.asarray(pose.rotation, dtype=np.float64),
            np.asarray(pose.homogeneous, dtype=np.float64),
        )

    def jacobian(
        self,
        q: Sequence[float],
        reference: JacobianReference = JacobianReference.LOCAL,
    ) -> np.ndarray:
        return np.asarray(self._native.jacobian(q, reference), dtype=np.float64)

    def gravity(self, q: Sequence[float]) -> np.ndarray:
        return np.asarray(self._native.gravity(q), dtype=np.float64)

    def mass_matrix(self, q: Sequence[float]) -> np.ndarray:
        return np.asarray(self._native.mass_matrix(q), dtype=np.float64)

    def coriolis_matrix(self, q: Sequence[float], dq: Sequence[float]) -> np.ndarray:
        return np.asarray(self._native.coriolis_matrix(q, dq), dtype=np.float64)

    def nonlinear_effects(self, q: Sequence[float], dq: Sequence[float]) -> np.ndarray:
        return np.asarray(self._native.nonlinear_effects(q, dq), dtype=np.float64)

    def rnea(
        self, q: Sequence[float], dq: Sequence[float], ddq: Sequence[float]
    ) -> np.ndarray:
        return np.asarray(self._native.rnea(q, dq, ddq), dtype=np.float64)

    def aba(
        self, q: Sequence[float], dq: Sequence[float], torque: Sequence[float]
    ) -> np.ndarray:
        return np.asarray(self._native.aba(q, dq, torque), dtype=np.float64)

    def ik(
        self,
        target_position: Sequence[float],
        target_rotation: Sequence[Sequence[float]],
        initial_q: Sequence[float],
        *,
        max_iterations: int = 1000,
        max_retries: int = 8,
        tolerance: float = 1e-4,
        step_size: float = 0.5,
        damping: float = 1e-6,
        random_seed: int = 0,
    ) -> NativeIkResult:
        position = np.asarray(target_position, dtype=np.float64).reshape(3)
        rotation = np.asarray(target_rotation, dtype=np.float64).reshape(3, 3)
        homogeneous = np.eye(4, dtype=np.float64)
        homogeneous[:3, :3] = rotation
        homogeneous[:3, 3] = position
        target = RuntimeRobotPose(
            tuple(float(value) for value in position),
            tuple(tuple(float(value) for value in row) for row in rotation),
            tuple(tuple(float(value) for value in row) for row in homogeneous),
        )
        result = self._native.ik(
            target,
            initial_q,
            RuntimeIkOptions(
                max_iterations=max_iterations,
                max_retries=max_retries,
                tolerance=tolerance,
                step_size=step_size,
                damping=damping,
                random_seed=random_seed,
            ),
        )
        return NativeIkResult(
            q=np.asarray(result.q, dtype=np.float64),
            success=result.success,
            error=result.error,
            iterations=result.iterations,
        )

    def close(self) -> None:
        self._native.close()

    def __enter__(self) -> NativeArmModel:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def load_native_robot_model(
    product: str = "yunyi_v1_0", side: str | RobotSide | int | None = None
) -> NativeArmModel:
    return NativeArmModel(product, side)


__all__ = [
    "JacobianReference",
    "NativeArmModel",
    "NativeIkResult",
    "RobotSide",
    "load_native_robot_model",
]
