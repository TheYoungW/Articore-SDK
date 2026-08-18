"""Runtime-owned robot dynamics.

All built-in product calculations execute inside Motor Drive Layer.  This
module intentionally exposes only the thin native model facade; it contains no
Python rigid-body dynamics implementation and does not load product URDFs.
"""
from __future__ import annotations

from ..native_robotics import (
    JacobianReference,
    NativeArmModel,
    NativeIkResult,
    RobotSide,
    load_native_robot_model,
)


__all__ = [
    "JacobianReference",
    "NativeArmModel",
    "NativeIkResult",
    "RobotSide",
    "load_native_robot_model",
]
