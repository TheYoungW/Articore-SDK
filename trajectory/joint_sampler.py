"""Dependency-light joint-position trajectory sampling."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True, slots=True)
class JointPositionPoint:
    """One time-stamped joint-position command."""

    time: float
    positions: np.ndarray


def _profile_scale(value: float, profile: str) -> float:
    t = min(1.0, max(0.0, float(value)))
    normalized = profile.strip().lower().replace("-", "_")
    if normalized == "linear":
        return t
    if normalized == "min_jerk":
        # Fifth-order time scaling: zero velocity and acceleration at both ends.
        return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5
    raise ValueError("profile must be 'min_jerk' or 'linear'")


def plan_joint_position_trajectory(
    start: Sequence[float],
    target: Sequence[float],
    *,
    duration: float,
    hz: float = 500.0,
    profile: str = "min_jerk",
) -> list[JointPositionPoint]:
    """Sample a joint-space position trajectory including both endpoints."""
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be a positive finite value")
    if not math.isfinite(hz) or hz <= 0.0:
        raise ValueError("hz must be a positive finite value")

    start_values = np.asarray(start, dtype=np.float64).reshape(-1)
    target_values = np.asarray(target, dtype=np.float64).reshape(-1)
    if start_values.size == 0 or start_values.shape != target_values.shape:
        raise ValueError("start and target must have the same non-zero length")
    if not np.all(np.isfinite(start_values)) or not np.all(np.isfinite(target_values)):
        raise ValueError("joint positions must be finite")

    intervals = max(1, int(math.ceil(duration * hz)))
    step_duration = duration / intervals
    delta = target_values - start_values
    points = []
    for index in range(intervals + 1):
        scale = _profile_scale(index / intervals, profile)
        points.append(
            JointPositionPoint(
                time=index * step_duration,
                positions=start_values + delta * scale,
            )
        )
    return points


__all__ = ["JointPositionPoint", "plan_joint_position_trajectory"]
