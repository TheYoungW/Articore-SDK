"""双臂重力补偿协调器。"""
from __future__ import annotations

import math
import time
from typing import Sequence

from ..sdk import ArxDCanDualArm
from .gravity_compensation import GravityCompensationMode


class DualArmGravityCompensationMode:
    """在同一循环中协调左右两套单臂重力补偿。"""

    def __init__(
        self,
        robot: ArxDCanDualArm,
        *,
        hz: float = 100.0,
        transition_seconds: float = 0.0,
        settle_seconds: float = 0.0,
        gravity_scale: float = 1.0,
        left_joint_scales: Sequence[float] | None = None,
        right_joint_scales: Sequence[float] | None = None,
        damping: float | Sequence[float] = 0.0,
    ) -> None:
        if not math.isfinite(hz) or hz <= 0.0:
            raise ValueError("hz must be finite and positive")
        self.robot = robot
        self.hz = float(hz)
        common = {
            "hz": self.hz,
            "transition_seconds": transition_seconds,
            "settle_seconds": settle_seconds,
            "gravity_scale": gravity_scale,
            "damping": damping,
        }
        self.left = GravityCompensationMode(
            robot.left,
            joint_scales=left_joint_scales,
            **common,
        )
        self.right = GravityCompensationMode(
            robot.right,
            joint_scales=right_joint_scales,
            **common,
        )
        self._active = False
        self._owns_connection = False

    @property
    def active(self) -> bool:
        """返回双臂重力补偿是否已启动。"""
        return self._active

    def start(self) -> None:
        """连接双臂并安全启动左右重力补偿。"""
        if self._active:
            raise RuntimeError("dual-arm gravity compensation is already active")
        self._owns_connection = not self.robot.connected
        if self._owns_connection:
            self.robot.connect()
        try:
            self.left.start()
            try:
                self.right.start()
            except Exception:
                self.left.stop()
                raise
        except Exception:
            if self._owns_connection:
                self.robot.close()
            raise
        self._active = True

    def run(self, *, seconds: float = 0.0) -> None:
        """持续刷新双臂重力补偿，直到超时或收到中断。"""
        if not self._active:
            raise RuntimeError("dual-arm gravity compensation is not active")
        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError("seconds must be finite and non-negative")
        deadline = None if seconds == 0.0 else time.monotonic() + seconds
        period = 1.0 / self.hz
        next_tick = time.monotonic()
        while deadline is None or time.monotonic() < deadline:
            self.left.step()
            self.right.step()
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def shutdown(self) -> None:
        """停止左右重力补偿，并关闭本对象建立的连接。"""
        errors: list[Exception] = []
        for mode in (self.left, self.right):
            try:
                mode.stop()
            except Exception as exc:
                errors.append(exc)
        self._active = False
        if self._owns_connection:
            try:
                self.robot.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError("dual-arm gravity-compensation shutdown failed") from errors[0]


__all__ = ["DualArmGravityCompensationMode"]
