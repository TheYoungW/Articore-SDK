"""Force-limited gripper closing state machine for MIT control."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math


class GripperControlState(str, Enum):
    IDLE = "idle"
    OPENING = "opening"
    CLOSING = "closing"
    HOLDING = "holding"
    OVERLOAD = "overload"


@dataclass(slots=True, frozen=True)
class GripperForceControlConfig:
    close_speed: float = 1.0
    contact_torque: float = 0.8
    overload_torque: float = 1.5
    motion_window_s: float = 0.2
    stall_movement: float = 0.01
    min_position_error: float = 0.05
    contact_hold_s: float = 0.2
    overload_hold_s: float = 0.05
    hold_offset: float = 0.08
    retreat_distance: float = 0.15
    max_step_interval_s: float = 0.05
    overload_retreat_interval_s: float = 0.1
    hold_kp: float = 2.0
    hold_kd: float = 0.5

    def __post_init__(self) -> None:
        values = (
            self.close_speed,
            self.contact_torque,
            self.overload_torque,
            self.motion_window_s,
            self.stall_movement,
            self.min_position_error,
            self.contact_hold_s,
            self.overload_hold_s,
            self.hold_offset,
            self.retreat_distance,
            self.max_step_interval_s,
            self.overload_retreat_interval_s,
            self.hold_kp,
            self.hold_kd,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("gripper force-control values must be finite")
        if any(value < 0.0 for value in values):
            raise ValueError("gripper force-control values must be non-negative")
        required_positive = {
            "close_speed": self.close_speed,
            "contact_torque": self.contact_torque,
            "motion_window_s": self.motion_window_s,
            "hold_offset": self.hold_offset,
            "retreat_distance": self.retreat_distance,
            "max_step_interval_s": self.max_step_interval_s,
            "overload_retreat_interval_s": self.overload_retreat_interval_s,
            "hold_kp": self.hold_kp,
        }
        for name, value in required_positive.items():
            if value == 0.0:
                raise ValueError(f"{name} must be greater than zero")
        if self.overload_torque <= self.contact_torque:
            raise ValueError("overload_torque must be greater than contact_torque")


@dataclass(slots=True, frozen=True)
class GripperMitCommand:
    position: float
    kp: float
    kd: float
    state: GripperControlState


class GripperForceController:
    def __init__(
        self,
        config: GripperForceControlConfig,
        *,
        open_value: float,
        closed_value: float,
        normal_kp: float,
        normal_kd: float,
    ) -> None:
        constructor_values = (open_value, closed_value, normal_kp, normal_kd)
        if not all(math.isfinite(value) for value in constructor_values):
            raise ValueError("gripper endpoints and normal gains must be finite")
        if normal_kp <= 0.0 or normal_kd < 0.0:
            raise ValueError("normal_kp must be positive and normal_kd non-negative")
        if config.hold_kp > normal_kp:
            raise ValueError("hold_kp must not exceed normal_kp")
        if math.isclose(open_value, closed_value):
            raise ValueError("gripper open and closed values must differ")
        self.config = config
        self.open_value = float(open_value)
        self.closed_value = float(closed_value)
        self.normal_kp = float(normal_kp)
        self.normal_kd = float(normal_kd)
        self._closing_direction = 1.0 if closed_value > open_value else -1.0
        self._samples: deque[tuple[float, float]] = deque()
        self.reset()

    @property
    def state(self) -> GripperControlState:
        return self._state

    def reset(self) -> None:
        self._state = GripperControlState.IDLE
        self._command_position: float | None = None
        self._last_time: float | None = None
        self._contact_started_at: float | None = None
        self._overload_started_at: float | None = None
        self._last_requested_position: float | None = None
        self._last_retreat_at: float | None = None
        self._samples.clear()

    def update(
        self,
        *,
        requested_position: float,
        actual_position: float,
        actual_torque: float,
        now: float,
    ) -> GripperMitCommand:
        requested = self._clamp(float(requested_position))
        actual = float(actual_position)
        torque = abs(float(actual_torque))
        timestamp = float(now)
        if not all(math.isfinite(value) for value in (requested, actual, torque, timestamp)):
            raise ValueError("gripper command and feedback values must be finite")

        if self._command_position is None:
            self._command_position = self._clamp(actual)
        dt = 0.0 if self._last_time is None else min(
            max(0.0, timestamp - self._last_time),
            self.config.max_step_interval_s,
        )
        self._last_time = timestamp
        self._record_motion(timestamp, actual)

        previous_requested = self._last_requested_position
        self._last_requested_position = requested
        opening_requested = (
            previous_requested is not None
            and self._is_more_open(requested, previous_requested)
        ) or self._is_more_open(requested, self._command_position)
        if opening_requested:
            self._clear_detection()
            self._state = GripperControlState.OPENING
            release_position = self._clamp(
                actual - self._closing_direction * self.config.hold_offset
            )
            self._command_position = (
                requested
                if self._is_more_open(requested, release_position)
                else release_position
            )
            return self._normal_command()

        if self._state is GripperControlState.OPENING:
            closing_requested = (
                previous_requested is not None
                and self._is_more_closed(requested, previous_requested)
            )
            if not closing_requested:
                return self._normal_command()

        if self._state is GripperControlState.OVERLOAD:
            if (
                torque >= self.config.overload_torque
                and self._last_retreat_at is not None
                and timestamp - self._last_retreat_at
                >= self.config.overload_retreat_interval_s
            ):
                self._command_position = self._clamp(
                    self._command_position
                    - self._closing_direction * self.config.retreat_distance
                )
                self._last_retreat_at = timestamp
            return self._hold_command()

        if self._state is GripperControlState.HOLDING:
            if self._sustained_overload(torque, timestamp):
                return self._enter_overload(actual)
            return self._hold_command()

        closing = self._is_more_closed(requested, actual) or self._is_more_closed(
            requested,
            self._command_position,
        )
        if not closing:
            self._clear_detection()
            self._state = GripperControlState.IDLE
            self._command_position = requested
            return self._normal_command()

        self._state = GripperControlState.CLOSING
        max_step = max(0.0, self.config.close_speed) * dt
        self._command_position = self._step_toward(
            self._command_position,
            requested,
            max_step,
        )

        if self._sustained_overload(torque, timestamp):
            return self._enter_overload(actual)

        contact = (
            torque >= self.config.contact_torque
            and self._motion_window_ready()
            and self._motion_span() <= self.config.stall_movement
            and abs(self._command_position - actual) >= self.config.min_position_error
        )
        if contact:
            if self._contact_started_at is None:
                self._contact_started_at = timestamp
            if timestamp - self._contact_started_at >= self.config.contact_hold_s:
                self._state = GripperControlState.HOLDING
                self._command_position = self._clamp(
                    actual + self._closing_direction * self.config.hold_offset
                )
                return self._hold_command()
        else:
            self._contact_started_at = None
        return self._normal_command()

    def _record_motion(self, now: float, position: float) -> None:
        self._samples.append((now, position))
        cutoff = now - max(0.0, self.config.motion_window_s)
        while len(self._samples) > 1 and self._samples[1][0] <= cutoff:
            self._samples.popleft()

    def _motion_window_ready(self) -> bool:
        return (
            len(self._samples) >= 2
            and self._samples[-1][0] - self._samples[0][0]
            >= self.config.motion_window_s
        )

    def _motion_span(self) -> float:
        if not self._samples:
            return 0.0
        positions = [position for _, position in self._samples]
        return max(positions) - min(positions)

    def _sustained_overload(self, torque: float, now: float) -> bool:
        if torque < self.config.overload_torque:
            self._overload_started_at = None
            return False
        if self._overload_started_at is None:
            self._overload_started_at = now
            return False
        return now - self._overload_started_at >= self.config.overload_hold_s

    def _enter_overload(self, actual: float) -> GripperMitCommand:
        self._state = GripperControlState.OVERLOAD
        self._command_position = self._clamp(
            actual - self._closing_direction * self.config.retreat_distance
        )
        self._last_retreat_at = self._last_time
        return self._hold_command()

    def _normal_command(self) -> GripperMitCommand:
        assert self._command_position is not None
        return GripperMitCommand(
            position=self._command_position,
            kp=self.normal_kp,
            kd=self.normal_kd,
            state=self._state,
        )

    def _hold_command(self) -> GripperMitCommand:
        assert self._command_position is not None
        return GripperMitCommand(
            position=self._command_position,
            kp=self.config.hold_kp,
            kd=self.config.hold_kd,
            state=self._state,
        )

    def _clear_detection(self) -> None:
        self._contact_started_at = None
        self._overload_started_at = None
        self._last_retreat_at = None

    def _is_more_closed(self, first: float, second: float) -> bool:
        return (first - second) * self._closing_direction > 1e-9

    def _is_more_open(self, first: float, second: float) -> bool:
        return (first - second) * self._closing_direction < -1e-9

    def _clamp(self, value: float) -> float:
        return min(max(value, min(self.open_value, self.closed_value)), max(self.open_value, self.closed_value))

    @staticmethod
    def _step_toward(current: float, target: float, max_step: float) -> float:
        delta = target - current
        if abs(delta) <= max_step:
            return target
        if max_step <= 0.0:
            return current
        return current + math.copysign(max_step, delta)
