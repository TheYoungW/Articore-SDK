"""Safe, synchronous gravity-compensation mode for :class:`ArxDCanArm`."""
from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
import time

import numpy as np

from ..sdk import ArxDCanArm


GravityProvider = Callable[[np.ndarray], np.ndarray]


@dataclass(slots=True, frozen=True)
class GravityCompensationSample:
    """One feedback and command sample from gravity-compensation mode."""

    elapsed_s: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    commanded_torques: tuple[float, ...]


def _gain_vector(
    value: float | Sequence[float],
    *,
    joint_count: int,
    name: str,
) -> np.ndarray:
    values = np.asarray(value, dtype=np.float64)
    if values.ndim == 0:
        values = np.full(joint_count, float(values), dtype=np.float64)
    else:
        values = values.reshape(-1)
    if len(values) != joint_count:
        raise ValueError(f"expected {joint_count} {name} values, got {len(values)}")
    if np.any(~np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError(f"{name} values must be finite and non-negative")
    return values


class GravityCompensationMode:
    """Run an arm in MIT gravity compensation with zero position stiffness.

    The mode starts from a current-position MIT hold, gradually replaces the
    configured Kp/Kd with gravity feedforward, and performs the inverse
    transition before disabling.  ``damping=0`` sends both Kp and Kd as zero;
    a small positive damping value keeps Kp at zero while adding velocity
    damping.
    """

    def __init__(
        self,
        arm: ArxDCanArm,
        *,
        hz: float = 100.0,
        transition_seconds: float = 3.0,
        settle_seconds: float = 0.5,
        gravity_scale: float = 1.0,
        joint_scales: Sequence[float] | None = None,
        damping: float | Sequence[float] = 0.0,
        max_velocity: float = math.radians(20.0),
        limit_margin: float = math.radians(5.0),
        torque_limit_ratio: float = 0.2,
        max_torques: Sequence[float] | None = None,
        stationary_velocity: float = math.radians(3.0),
        gravity_provider: GravityProvider | None = None,
    ) -> None:
        self.arm = arm
        self._joint_count = len(arm.joint_names)
        if self._joint_count == 0:
            raise ValueError("gravity compensation requires at least one arm joint")
        if not math.isfinite(hz) or hz <= 0.0:
            raise ValueError("hz must be finite and positive")
        if not math.isfinite(transition_seconds) or transition_seconds < 0.0:
            raise ValueError("transition_seconds must be finite and non-negative")
        if not math.isfinite(settle_seconds) or settle_seconds < 0.0:
            raise ValueError("settle_seconds must be finite and non-negative")
        if not math.isfinite(gravity_scale) or gravity_scale < 0.0:
            raise ValueError("gravity_scale must be finite and non-negative")
        if not math.isfinite(max_velocity) or max_velocity <= 0.0:
            raise ValueError("max_velocity must be finite and positive")
        if not math.isfinite(limit_margin) or limit_margin < 0.0:
            raise ValueError("limit_margin must be finite and non-negative")
        if (
            not math.isfinite(torque_limit_ratio)
            or torque_limit_ratio <= 0.0
            or torque_limit_ratio > 1.0
        ):
            raise ValueError("torque_limit_ratio must be in (0, 1]")
        if not math.isfinite(stationary_velocity) or stationary_velocity <= 0.0:
            raise ValueError("stationary_velocity must be finite and positive")

        self.hz = float(hz)
        self.transition_seconds = float(transition_seconds)
        self.settle_seconds = float(settle_seconds)
        self.gravity_scale = float(gravity_scale)
        self.max_velocity = float(max_velocity)
        self.limit_margin = float(limit_margin)
        self.stationary_velocity = float(stationary_velocity)
        self._period = 1.0 / self.hz
        self._zeros = np.zeros(self._joint_count, dtype=np.float64)
        self._damping = _gain_vector(
            damping,
            joint_count=self._joint_count,
            name="damping",
        )
        self._joint_scales = _gain_vector(
            1.0 if joint_scales is None else joint_scales,
            joint_count=self._joint_count,
            name="joint_scales",
        )
        self._default_kp = np.asarray(
            [joint.mit_kp for joint in arm.config.arm_joints],
            dtype=np.float64,
        )
        self._default_kd = np.asarray(
            [joint.mit_kd for joint in arm.config.arm_joints],
            dtype=np.float64,
        )
        if max_torques is None:
            ranges = np.asarray(
                [
                    joint.torque_range if joint.torque_range is not None else 10.0
                    for joint in arm.config.arm_joints
                ],
                dtype=np.float64,
            )
            self._max_torques = torque_limit_ratio * ranges
        else:
            self._max_torques = _gain_vector(
                max_torques,
                joint_count=self._joint_count,
                name="max_torques",
            )
            if np.any(self._max_torques <= 0.0):
                raise ValueError("max_torques values must be positive")

        self._gravity_provider = gravity_provider or self._build_gravity_provider()
        self._started = False
        self._owns_connection = False
        self._active_started = 0.0
        self._last_sample: GravityCompensationSample | None = None
        self._transition_safe = True

    @property
    def active(self) -> bool:
        return self._started

    @property
    def last_sample(self) -> GravityCompensationSample | None:
        return self._last_sample

    @property
    def max_torques(self) -> tuple[float, ...]:
        return tuple(float(value) for value in self._max_torques)

    def _build_gravity_provider(self) -> GravityProvider:
        if self.arm.config.urdf_path is None:
            raise ValueError("gravity compensation requires a URDF model")
        try:
            from ..dynamics import compute_generalized_gravity
            from ..kinematics import load_robot_model
        except (ImportError, ModuleNotFoundError) as exc:
            raise RuntimeError(
                "gravity compensation requires Pinocchio; install the "
                "'dynamics' extra. If a ROS environment shadows the active "
                "Python environment, run with PYTHONPATH unset."
            ) from exc

        model = load_robot_model(
            self.arm.config.urdf_path,
            controlled_joint_names=self.arm.joint_names,
        )
        data = model.createData()

        def compute(positions: np.ndarray) -> np.ndarray:
            return compute_generalized_gravity(model, positions, data)

        return compute

    def _gravity_torques(self, positions: np.ndarray) -> np.ndarray:
        raw = np.asarray(self._gravity_provider(positions), dtype=np.float64).reshape(-1)
        if len(raw) != self._joint_count:
            raise RuntimeError(
                f"gravity provider returned {len(raw)} torques, "
                f"expected {self._joint_count}"
            )
        torques = self.gravity_scale * self._joint_scales * raw
        if np.any(~np.isfinite(torques)):
            raise RuntimeError("gravity provider returned non-finite torques")
        exceeded = np.flatnonzero(np.abs(torques) > self._max_torques)
        if exceeded.size:
            details = ", ".join(
                f"{self.arm.joint_names[index]}={torques[index]:+.3f}Nm "
                f"(limit {self._max_torques[index]:.3f}Nm)"
                for index in exceeded
            )
            raise RuntimeError(f"gravity torque limit exceeded: {details}")
        return torques

    def _read_checked_state(self, *, require_stationary: bool = False):
        state = self.arm.read_state(request_feedback=True)
        if self.arm.faulted or self.arm.safe_holding:
            raise RuntimeError(
                self.arm.fault_reason or "arm entered fault-safe holding"
            )
        positions = np.asarray(state.arm.positions, dtype=np.float64)
        velocities = np.asarray(state.arm.velocities, dtype=np.float64)
        if len(positions) != self._joint_count or len(velocities) != self._joint_count:
            raise RuntimeError("arm feedback joint count changed")
        if np.any(~np.isfinite(positions)) or np.any(~np.isfinite(velocities)):
            raise RuntimeError("arm feedback contains non-finite values")
        velocity_limit = self.stationary_velocity if require_stationary else self.max_velocity
        if np.max(np.abs(velocities)) > velocity_limit:
            raise RuntimeError(
                f"joint velocity exceeded {math.degrees(velocity_limit):.3f}deg/s"
            )
        for index, joint in enumerate(self.arm.config.arm_joints):
            if (
                joint.lower_limit is not None
                and positions[index] < joint.lower_limit + self.limit_margin
            ):
                raise RuntimeError(f"{joint.name} approached its lower limit")
            if (
                joint.upper_limit is not None
                and positions[index] > joint.upper_limit - self.limit_margin
            ):
                raise RuntimeError(f"{joint.name} approached its upper limit")
        return state, positions, velocities

    def _send(
        self,
        positions: np.ndarray,
        torques: np.ndarray,
        *,
        kp: np.ndarray,
        kd: np.ndarray,
        require_enabled: bool = True,
    ) -> None:
        self.arm.send_joint_positions(
            positions,
            velocities=self._zeros,
            torques=torques,
            mit_kp=kp,
            mit_kd=kd,
            mode="mit",
            require_enabled=require_enabled,
        )

    def _run_transition(self, hold_position: np.ndarray, *, entering: bool) -> None:
        if self.transition_seconds <= 0.0:
            state, positions, _ = self._read_checked_state()
            gravity = self._gravity_torques(positions)
            alpha = 1.0 if entering else 0.0
            self._send(
                hold_position,
                alpha * gravity,
                kp=(1.0 - alpha) * self._default_kp,
                kd=(1.0 - alpha) * self._default_kd + alpha * self._damping,
            )
            return

        steps = max(1, math.ceil(self.transition_seconds * self.hz))
        started = time.monotonic()
        for index in range(steps + 1):
            state, positions, _ = self._read_checked_state()
            del state
            gravity = self._gravity_torques(positions)
            progress = index / steps
            gravity_alpha = progress if entering else 1.0 - progress
            self._send(
                hold_position,
                gravity_alpha * gravity,
                kp=(1.0 - gravity_alpha) * self._default_kp,
                kd=(
                    (1.0 - gravity_alpha) * self._default_kd
                    + gravity_alpha * self._damping
                ),
            )
            deadline = started + (index + 1) * self._period
            time.sleep(max(0.0, deadline - time.monotonic()))

    def start(self) -> GravityCompensationSample:
        """Connect, enable, and transition into gravity compensation."""
        if self._started:
            raise RuntimeError("gravity compensation is already active")
        self._owns_connection = not self.arm.connected
        try:
            if self._owns_connection:
                self.arm.connect()
            if self.arm.enabled:
                raise RuntimeError(
                    "arm must be disabled before entering gravity compensation"
                )
            state, initial_position, _ = self._read_checked_state(
                require_stationary=True
            )
            del state
            self._gravity_torques(initial_position)
            self.arm.configure("mit")
            self._send(
                initial_position,
                self._zeros,
                kp=self._default_kp,
                kd=self._default_kd,
                require_enabled=False,
            )
            self.arm.enable()

            settle_deadline = time.monotonic() + self.settle_seconds
            next_tick = time.monotonic()
            while time.monotonic() < settle_deadline:
                self._read_checked_state()
                self._send(
                    initial_position,
                    self._zeros,
                    kp=self._default_kp,
                    kd=self._default_kd,
                )
                next_tick += self._period
                time.sleep(max(0.0, next_tick - time.monotonic()))

            self._run_transition(initial_position, entering=True)
            self._started = True
            self._transition_safe = True
            self._active_started = time.monotonic()
            return self.step()
        except Exception:
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
            self._started = False
            raise

    def step(self) -> GravityCompensationSample:
        """Send one zero-stiffness gravity-compensation frame."""
        if not self._started:
            raise RuntimeError("gravity compensation is not active")
        try:
            _, positions, velocities = self._read_checked_state()
            torques = self._gravity_torques(positions)
            self._send(
                positions,
                torques,
                kp=self._zeros,
                kd=self._damping,
            )
        except Exception:
            self._transition_safe = False
            raise
        sample = GravityCompensationSample(
            elapsed_s=time.monotonic() - self._active_started,
            positions=tuple(float(value) for value in positions),
            velocities=tuple(float(value) for value in velocities),
            commanded_torques=tuple(float(value) for value in torques),
        )
        self._last_sample = sample
        return sample

    def run(
        self,
        *,
        seconds: float = 0.0,
        on_sample: Callable[[GravityCompensationSample], None] | None = None,
    ) -> None:
        """Refresh gravity compensation until timeout or ``KeyboardInterrupt``."""
        if not self._started:
            raise RuntimeError("gravity compensation is not active")
        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError("seconds must be finite and non-negative")
        deadline = None if seconds == 0.0 else time.monotonic() + seconds
        next_tick = time.monotonic()
        while deadline is None or time.monotonic() < deadline:
            sample = self.step()
            if on_sample is not None:
                on_sample(sample)
            next_tick += self._period
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def stop(self) -> None:
        """Restore the configured MIT gains, then disable the arm."""
        transition_error: Exception | None = None
        try:
            if (
                self._started
                and self.arm.connected
                and self.arm.enabled
                and not self.arm.faulted
                and not self.arm.safe_holding
                and self._transition_safe
            ):
                _, hold_position, _ = self._read_checked_state()
                self._run_transition(hold_position, entering=False)
        except Exception as exc:
            transition_error = exc
        finally:
            if self.arm.connected and self.arm.enabled:
                try:
                    self.arm.disable()
                except Exception as exc:
                    if transition_error is None:
                        transition_error = exc
            self._started = False
            self._transition_safe = True
        if transition_error is not None:
            raise RuntimeError(
                f"gravity-compensation shutdown failed: {transition_error}"
            ) from transition_error

    def shutdown(self) -> None:
        """Stop the mode and close a connection opened by this object."""
        error: Exception | None = None
        try:
            self.stop()
        except Exception as exc:
            error = exc
        if self._owns_connection and self.arm.connected:
            try:
                self.arm.close()
            except Exception as exc:
                if error is None:
                    error = exc
        if error is not None:
            raise error

    def __enter__(self) -> "GravityCompensationMode":
        self.start()
        return self

    def __exit__(self, *_args) -> None:
        self.shutdown()


__all__ = ["GravityCompensationMode", "GravityCompensationSample"]
