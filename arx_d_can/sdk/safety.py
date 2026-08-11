"""连接保护、命令看门狗、安全保持与故障处理。"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from ..errors import CommandTimeoutError, CommunicationError, StaleFeedbackError
from .state import MitCommand


class _SafetyMixin:
    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("ARX-D-CAN arm is not connected")

    def _record_communication_error(
        self,
        error: CommunicationError,
        *,
        using_fallback: bool | None,
    ) -> None:
        with self._state_lock:
            self._last_communication_error = error
            if using_fallback is not None:
                self._using_fallback_state = using_fallback

    def _active_joint_names(self) -> list[str]:
        names = list(self.config.joint_names)
        if self.enable_gripper and self.config.gripper is not None:
            names.append(self.config.gripper.name)
        return names

    def _validate_safety_config(self) -> None:
        if self.config.gripper is not None:
            endpoints = (
                self.config.gripper_closed_value,
                self.config.gripper_open_value,
            )
            if not all(math.isfinite(value) for value in endpoints):
                raise ValueError("gripper endpoints must be finite")
            if math.isclose(*endpoints):
                raise ValueError("gripper open and closed values must differ")
        if (
            not math.isfinite(self.config.control_hz)
            or self.config.control_hz <= 0.0
        ):
            raise ValueError("control_hz must be finite and positive")
        if self.config.watchdog_enabled and self.config.command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be positive")
        if self.config.enable_grace_s < 0.0:
            raise ValueError("enable_grace_s must not be negative")
        if self.config.watchdog_poll_s <= 0.0:
            raise ValueError("watchdog_poll_s must be positive")
        action = self.config.watchdog_action.strip().lower()
        if action not in {"safe_hold", "disable"}:
            raise ValueError("watchdog_action must be 'safe_hold' or 'disable'")
        if self.config.safe_hold_hz <= 0.0:
            raise ValueError("safe_hold_hz must be positive")
        if (
            not math.isfinite(self.config.safe_hold_pv_velocity_limit)
            or self.config.safe_hold_pv_velocity_limit <= 0.0
        ):
            raise ValueError("safe_hold_pv_velocity_limit must be finite and positive")
        if (
            not math.isfinite(self.config.safe_hold_mit_kp)
            or self.config.safe_hold_mit_kp < 0.0
            or not math.isfinite(self.config.safe_hold_mit_kd)
            or self.config.safe_hold_mit_kd < 0.0
        ):
            raise ValueError("safe_hold MIT gains must be finite and non-negative")
        if self.config.safe_hold_failure_threshold < 1:
            raise ValueError("safe_hold_failure_threshold must be at least 1")
        if (
            not math.isfinite(self.config.feedback_check_hz)
            or self.config.feedback_check_hz <= 0.0
        ):
            raise ValueError("feedback_check_hz must be finite and positive")
        if self.config.feedback_fault_threshold < 1:
            raise ValueError("feedback_fault_threshold must be at least 1")
        if (
            not math.isfinite(self.config.max_cached_feedback_age_s)
            or self.config.max_cached_feedback_age_s <= 0.0
        ):
            raise ValueError("max_cached_feedback_age_s must be finite and positive")
        if not 1 <= self.config.motor_communication_timeout_ms <= 0xFFFFFFFF // 20:
            raise ValueError(
                "motor_communication_timeout_ms must be in 1..=214748364"
            )
        for joint in self.config.arm_joints:
            if (
                not math.isfinite(joint.coupled_motor_kd)
                or joint.coupled_motor_kd < 0.0
            ):
                raise ValueError(
                    f"{joint.name}.coupled_motor_kd must be finite and non-negative"
                )
            if (
                not math.isfinite(joint.coupled_velocity_filter_s)
                or joint.coupled_velocity_filter_s < 0.0
            ):
                raise ValueError(
                    f"{joint.name}.coupled_velocity_filter_s must be finite "
                    "and non-negative"
                )
            for name, value in (
                ("coupled_effort_limit", joint.coupled_effort_limit),
                ("coupled_torque_rise_rate", joint.coupled_torque_rise_rate),
                (
                    "coupled_hold_torque_rise_rate",
                    joint.coupled_hold_torque_rise_rate,
                ),
                ("coupled_torque_brake_rate", joint.coupled_torque_brake_rate),
            ):
                if value is not None and (
                    not math.isfinite(value) or value <= 0.0
                ):
                    raise ValueError(
                        f"{joint.name}.{name} must be finite and positive"
                    )

    def _require_operational(self) -> None:
        self._require_connected()
        if self._faulted:
            raise RuntimeError(
                f"ARX-D-CAN arm is faulted: {self._fault_reason}; "
                "call clear_fault() and enable() to recover"
            )

    def _record_successful_command(
        self,
        *,
        joint_positions: tuple[float, ...] | None = None,
        mit_command: MitCommand | None = None,
        gripper_position: float | None = None,
    ) -> None:
        with self._state_lock:
            if joint_positions is not None:
                self._last_joint_command = joint_positions
            if mit_command is not None:
                self._last_mit_command = mit_command
            if gripper_position is not None:
                self._last_gripper_command = gripper_position
            if self._enabled:
                self._watchdog_deadline = (
                    time.monotonic() + self.config.command_timeout_s
                )

    def _start_watchdog(self) -> None:
        with self._state_lock:
            safe_holding = self._safe_holding
        if not self.config.watchdog_enabled and not safe_holding:
            return
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            return
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="arx-d-can-command-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()

    def _stop_watchdog(self) -> None:
        self._watchdog_stop.set()
        thread = self._watchdog_thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=1.0)
        if thread is not threading.current_thread():
            self._watchdog_thread = None

    def _watchdog_loop(self) -> None:
        with self._state_lock:
            safe_holding = self._safe_holding
        if safe_holding:
            self._safe_hold_loop()
            return

        poll_s = max(0.005, self.config.watchdog_poll_s)
        while not self._watchdog_stop.wait(poll_s):
            with self._state_lock:
                deadline = self._watchdog_deadline
                enabled = self._enabled
            if enabled and deadline is not None and time.monotonic() > deadline:
                error = CommandTimeoutError(
                    f"command watchdog timed out after "
                    f"{self.config.command_timeout_s:.3f}s"
                )
                if self.config.watchdog_action.strip().lower() == "safe_hold":
                    if self._begin_safe_hold(
                        str(error),
                        expected_deadline=deadline,
                    ):
                        self._safe_hold_loop()
                else:
                    self._trip_fault(str(error))
                return

    def _begin_safe_hold(
        self,
        reason: str,
        *,
        expected_deadline: float | None = None,
    ) -> bool:
        called_from_watchdog = threading.current_thread() is self._watchdog_thread
        if not called_from_watchdog:
            self._stop_watchdog()
        with self._state_lock:
            if (
                not self._enabled
                or (
                    expected_deadline is not None
                    and (
                        self._watchdog_deadline != expected_deadline
                        or time.monotonic() <= expected_deadline
                    )
                )
            ):
                return False
            self._faulted = True
            self._safe_holding = True
            self._fault_reason = f"{reason}; holding last successful command"
            self._watchdog_deadline = None
        if not called_from_watchdog:
            self._start_watchdog()
        return True

    def _safe_hold_loop(self) -> None:
        period = 1.0 / self.config.safe_hold_hz
        while not self._watchdog_stop.is_set():
            with self._state_lock:
                if not self._safe_holding or not self._enabled:
                    return
                joint_target = self._last_joint_command
                mit_command = self._last_mit_command
                gripper_target = self._last_gripper_command
            try:
                if joint_target is not None:
                    if self._mode == "mit" and mit_command is not None:
                        self._send_mit_command(mit_command, strict=False)
                    else:
                        target, _, _, _ = self._transform_command_vectors(
                            joint_target
                        )
                        with self._io_lock:
                            self.robot.arm.send_pos_vel(target, strict=False)
                with self._io_lock:
                    if gripper_target is not None:
                        kp = self.config.gripper_force_control.hold_kp
                        kd = self.config.gripper_force_control.hold_kd
                        self.robot.gripper.send_mit(
                            np.array([gripper_target]),
                            kp=np.array([kp]),
                            kd=np.array([kd]),
                            strict=False,
                        )
            except Exception as exc:
                if isinstance(exc, CommunicationError):
                    self._record_communication_error(
                        exc,
                        using_fallback=(
                            False if isinstance(exc, StaleFeedbackError) else None
                        ),
                    )
                if isinstance(exc, StaleFeedbackError):
                    self._trip_fault(str(exc))
                    return
                with self._state_lock:
                    if self._safe_holding:
                        marker = "; hold retry failed:"
                        if marker not in (self._fault_reason or ""):
                            self._fault_reason = (
                                f"{self._fault_reason}{marker} {exc}"
                            )
            self._watchdog_stop.wait(period)

    def _resume_from_safe_hold(self) -> None:
        self._stop_watchdog()
        with self._state_lock:
            if not self._safe_holding:
                return
            self._faulted = False
            self._safe_holding = False
            self._fault_reason = None
            self._feedback_error_count = 0
            self._last_communication_error = None
            self._using_fallback_state = False
            self._watchdog_deadline = (
                time.monotonic() + self.config.command_timeout_s
                if self._enabled
                else None
            )
        self._start_watchdog()

    def _trip_fault(self, reason: str) -> None:
        with self._state_lock:
            if self._faulted and not self._safe_holding:
                return
            self._faulted = True
            self._safe_holding = False
            self._fault_reason = reason
            self._enabled = False
            self._watchdog_deadline = None
            self._watchdog_stop.set()
        try:
            self.robot.estop()
        except Exception as exc:
            with self._state_lock:
                self._fault_reason = f"{reason}; emergency disable error: {exc}"
                self._enabled = True
