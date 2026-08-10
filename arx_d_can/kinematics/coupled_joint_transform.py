"""仅在运行时使用的耦合关节坐标转换。"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Sequence

import numpy as np


@dataclass(frozen=True, slots=True)
class _Pair:
    indices: tuple[int, int]
    reference_deg: np.ndarray


class CoupledJointTransform:
    """在公开虚拟关节坐标与物理电机坐标之间双向转换。"""

    def __init__(
        self,
        *,
        powers: Sequence[Sequence[int]],
        forward_coefficients: Sequence[Sequence[float]],
        inverse_coefficients: Sequence[Sequence[float]],
        pairs: Sequence[_Pair],
    ) -> None:
        self._powers = tuple((int(power[0]), int(power[1])) for power in powers)
        self._forward = np.asarray(forward_coefficients, dtype=np.float64)
        self._inverse = np.asarray(inverse_coefficients, dtype=np.float64)
        self._pairs = tuple(pairs)
        expected_shape = (2, len(self._powers))
        if not self._powers or self._forward.shape != expected_shape:
            raise ValueError("invalid forward coupled-joint model")
        if self._inverse.shape != expected_shape:
            raise ValueError("invalid inverse coupled-joint model")
        if not np.all(np.isfinite(self._forward)) or not np.all(np.isfinite(self._inverse)):
            raise ValueError("coupled-joint model coefficients must be finite")

    @classmethod
    def load(
        cls,
        path: str | Path,
        *,
        joint_names: Sequence[str],
    ) -> "CoupledJointTransform":
        model_path = Path(path)
        data = json.loads(model_path.read_text(encoding="utf-8"))
        if data.get("schema_version") != 1 or data.get("units") != "degrees":
            raise ValueError(f"unsupported coupled-joint model: {model_path}")
        name_to_index = {str(name): index for index, name in enumerate(joint_names)}
        pairs: list[_Pair] = []
        for item in data.get("pairs", ()):
            names = tuple(str(name) for name in item.get("joint_names", ()))
            reference = np.asarray(
                item.get("motor_reference_absolute_deg", ()),
                dtype=np.float64,
            )
            if len(names) != 2 or reference.shape != (2,) or not np.all(np.isfinite(reference)):
                raise ValueError(f"invalid coupled-joint pair in {model_path}")
            if not set(names).issubset(name_to_index):
                continue
            pairs.append(
                _Pair(
                    indices=(name_to_index[names[0]], name_to_index[names[1]]),
                    reference_deg=reference,
                )
            )
        if not pairs:
            raise ValueError("coupled-joint model does not match the configured joints")
        return cls(
            powers=data["feature_powers"],
            forward_coefficients=data["forward_coefficients"],
            inverse_coefficients=data["inverse_coefficients"],
            pairs=pairs,
        )

    @property
    def transformed_indices(self) -> frozenset[int]:
        return frozenset(index for pair in self._pairs for index in pair.indices)

    @property
    def transformed_pairs(self) -> tuple[tuple[int, int], ...]:
        """返回共用同一个虚拟变换的物理电机索引对。"""
        return tuple(pair.indices for pair in self._pairs)

    def _features(self, values: np.ndarray) -> np.ndarray:
        first, second = (float(value) for value in values)
        return np.asarray(
            [first**first_power * second**second_power
             for first_power, second_power in self._powers],
            dtype=np.float64,
        )

    def _evaluate(self, coefficients: np.ndarray, values: np.ndarray) -> np.ndarray:
        return coefficients @ self._features(values)

    def _jacobian(self, coefficients: np.ndarray, values: np.ndarray) -> np.ndarray:
        first, second = (float(value) for value in values)
        columns = []
        for axis in (0, 1):
            derivative = []
            for first_power, second_power in self._powers:
                powers = (first_power, second_power)
                exponent = powers[axis]
                if exponent == 0:
                    derivative.append(0.0)
                    continue
                remaining = list(powers)
                remaining[axis] -= 1
                derivative.append(
                    exponent * first ** remaining[0] * second ** remaining[1]
                )
            columns.append(coefficients @ np.asarray(derivative, dtype=np.float64))
        jacobian = np.column_stack(columns)
        if not np.all(np.isfinite(jacobian)) or abs(float(np.linalg.det(jacobian))) < 1e-8:
            raise ValueError("coupled-joint model Jacobian is singular")
        return jacobian

    @staticmethod
    def _vector(values: Sequence[float], *, expected: int) -> np.ndarray:
        vector = np.asarray(values, dtype=np.float64).reshape(-1).copy()
        if vector.size != expected or not np.all(np.isfinite(vector)):
            raise ValueError(f"expected {expected} finite joint values")
        return vector

    def virtual_positions_to_motor(self, positions: Sequence[float]) -> np.ndarray:
        result = self._vector(positions, expected=len(positions))
        for pair in self._pairs:
            virtual_deg = np.degrees(result[list(pair.indices)])
            motor_deg = self._evaluate(self._inverse, virtual_deg) + pair.reference_deg
            result[list(pair.indices)] = np.radians(motor_deg)
        return result

    def motor_positions_to_virtual(self, positions: Sequence[float]) -> np.ndarray:
        result = self._vector(positions, expected=len(positions))
        for pair in self._pairs:
            motor_relative_deg = (
                np.degrees(result[list(pair.indices)]) - pair.reference_deg
            )
            virtual_deg = self._evaluate(self._forward, motor_relative_deg)
            result[list(pair.indices)] = np.radians(virtual_deg)
        return result

    def virtual_velocities_to_motor(
        self,
        virtual_positions: Sequence[float],
        virtual_velocities: Sequence[float],
    ) -> np.ndarray:
        positions = self._vector(virtual_positions, expected=len(virtual_positions))
        result = self._vector(virtual_velocities, expected=len(positions))
        for pair in self._pairs:
            virtual_deg = np.degrees(positions[list(pair.indices)])
            result[list(pair.indices)] = (
                self._jacobian(self._inverse, virtual_deg)
                @ result[list(pair.indices)]
            )
        return result

    def motor_velocities_to_virtual(
        self,
        motor_positions: Sequence[float],
        motor_velocities: Sequence[float],
    ) -> np.ndarray:
        positions = self._vector(motor_positions, expected=len(motor_positions))
        result = self._vector(motor_velocities, expected=len(positions))
        for pair in self._pairs:
            motor_relative_deg = (
                np.degrees(positions[list(pair.indices)]) - pair.reference_deg
            )
            result[list(pair.indices)] = (
                self._jacobian(self._forward, motor_relative_deg)
                @ result[list(pair.indices)]
            )
        return result

    def virtual_torques_to_motor(
        self,
        virtual_positions: Sequence[float],
        virtual_torques: Sequence[float],
    ) -> np.ndarray:
        positions = self._vector(virtual_positions, expected=len(virtual_positions))
        result = self._vector(virtual_torques, expected=len(positions))
        for pair in self._pairs:
            virtual_deg = np.degrees(positions[list(pair.indices)])
            jacobian = self._jacobian(self._inverse, virtual_deg)
            result[list(pair.indices)] = np.linalg.solve(
                jacobian.T,
                result[list(pair.indices)],
            )
        return result

    def motor_torques_to_virtual(
        self,
        motor_positions: Sequence[float],
        motor_torques: Sequence[float],
    ) -> np.ndarray:
        positions = self._vector(motor_positions, expected=len(motor_positions))
        result = self._vector(motor_torques, expected=len(positions))
        for pair in self._pairs:
            motor_relative_deg = (
                np.degrees(positions[list(pair.indices)]) - pair.reference_deg
            )
            jacobian = self._jacobian(self._forward, motor_relative_deg)
            result[list(pair.indices)] = np.linalg.solve(
                jacobian.T,
                result[list(pair.indices)],
            )
        return result

    def virtual_velocity_limits_to_motor(
        self,
        virtual_positions: Sequence[float],
        virtual_limits: Sequence[float],
    ) -> np.ndarray:
        positions = self._vector(virtual_positions, expected=len(virtual_positions))
        result = self._vector(virtual_limits, expected=len(positions))
        for pair in self._pairs:
            virtual_deg = np.degrees(positions[list(pair.indices)])
            result[list(pair.indices)] = (
                np.abs(self._jacobian(self._inverse, virtual_deg))
                @ result[list(pair.indices)]
            )
        if np.any(result <= 0.0) or not np.all(np.isfinite(result)):
            raise ValueError("transformed velocity limits must be finite and positive")
        return result
