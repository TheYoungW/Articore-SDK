"""普通关节命令的数据准备。"""
from __future__ import annotations

import time
from typing import Sequence

import numpy as np

from .state import MitCommand


class _JointCommandMixin:
    """只负责普通关节向量和 MIT 数据包，不执行安全状态机。"""

    def _transform_command_vectors(
        self,
        positions: Sequence[float],
        *,
        velocities: Sequence[float] | None = None,
        torques: Sequence[float] | None = None,
        velocity_limits: Sequence[float] | None = None,
    ) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        return (
            np.asarray(positions, dtype=np.float64),
            None if velocities is None else np.asarray(velocities, dtype=np.float64),
            None if torques is None else np.asarray(torques, dtype=np.float64),
            (
                None
                if velocity_limits is None
                else np.asarray(velocity_limits, dtype=np.float64)
            ),
        )

    def _transform_feedback_vectors(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        torques: Sequence[float],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return (
            np.asarray(positions, dtype=np.float64),
            np.asarray(velocities, dtype=np.float64),
            np.asarray(torques, dtype=np.float64),
        )

    def _resolved_mit_gains(
        self,
        values: np.ndarray | None,
        *,
        gain: str,
    ) -> np.ndarray:
        if values is not None:
            return np.asarray(values, dtype=np.float64).reshape(-1).copy()
        attribute = "mit_kp" if gain == "kp" else "mit_kd"
        return np.asarray(
            [getattr(joint, attribute) for joint in self.config.arm_joints],
            dtype=np.float64,
        )

    def _make_mit_command(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        kp: Sequence[float],
        kd: Sequence[float],
        feedforward_torques: Sequence[float],
    ) -> MitCommand:
        return MitCommand(
            positions=tuple(float(value) for value in positions),
            velocities=tuple(float(value) for value in velocities),
            kp=tuple(float(value) for value in kp),
            kd=tuple(float(value) for value in kd),
            feedforward_torques=tuple(
                float(value) for value in feedforward_torques
            ),
            timestamp=time.monotonic(),
        )

    @staticmethod
    def _compose_mit_motor_command(
        command: MitCommand,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return (
            np.asarray(command.positions, dtype=np.float64),
            np.asarray(command.velocities, dtype=np.float64),
            np.asarray(command.kp, dtype=np.float64),
            np.asarray(command.kd, dtype=np.float64),
            np.asarray(command.feedforward_torques, dtype=np.float64),
        )

    def _send_mit_command(self, command: MitCommand, *, strict: bool) -> None:
        position, velocity, kp, kd, torque = self._compose_mit_motor_command(command)
        with self._io_lock:
            self.robot.arm.send_mit(
                position,
                vel=velocity,
                kp=kp,
                kd=kd,
                tau=torque,
                strict=strict,
            )
