"""Python 接口层的连接检查与原生安全参数校验。"""
from __future__ import annotations

import math

from .config import _MIT_GAIN_MAX


class _SafetyMixin:
    """不执行安全循环，只把故障动作交给 motor 原生运行时。"""

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("ARX-D-CAN arm is not connected")

    def _active_joint_names(self) -> list[str]:
        names = list(self.config.joint_names)
        if self.enable_gripper and self.config.gripper is not None:
            names.append(self.config.gripper.name)
        return names

    def _validate_safety_config(self) -> None:
        for joint in (*self.config.arm_joints, self.config.gripper):
            if joint is None:
                continue
            for name, value in (("Kp", joint.mit_kp), ("Kd", joint.mit_kd)):
                maximum = _MIT_GAIN_MAX[name]
                if not math.isfinite(value) or not 0.0 <= value <= maximum:
                    raise ValueError(
                        f"{joint.name}.MIT.{name.lower()} must be in [0, {maximum:g}]"
                    )
        positive_values = {
            "control_hz": self.config.control_hz,
            "command_timeout_s": self.config.command_timeout_s,
            "safe_hold_hz": self.config.safe_hold_hz,
            "feedback_check_hz": self.config.feedback_check_hz,
            "max_cached_feedback_age_s": self.config.max_cached_feedback_age_s,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.config.enable_grace_s < 0.0:
            raise ValueError("enable_grace_s must not be negative")
        if self.config.feedback_fault_threshold < 1:
            raise ValueError("feedback_fault_threshold must be at least 1")
        if self.config.safe_hold_failure_threshold < 1:
            raise ValueError("safe_hold_failure_threshold must be at least 1")
        if not 1 <= self.config.motor_communication_timeout_ms <= 0xFFFFFFFF // 20:
            raise ValueError(
                "motor_communication_timeout_ms must be in 1..=214748364"
            )

    def _require_operational(self) -> None:
        self._require_connected()
        runtime = self._single_safety_runtime
        if runtime is not None:
            self._sync_runtime_flags(runtime.health)
        if self._faulted:
            raise RuntimeError(
                f"ARX-D-CAN arm is faulted: {self._fault_reason}; call recover()"
            )

    def _trip_fault(self, reason: str) -> None:
        runtime = self._single_safety_runtime
        if runtime is not None:
            runtime.estop(reason)
            self._sync_runtime_flags(runtime.health)
            return
        try:
            self.robot.estop()
        finally:
            with self._state_lock:
                self._faulted = True
                self._fault_reason = reason
                self._safe_holding = False
                self._enabled = False
