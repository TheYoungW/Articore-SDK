"""ARX-D-CAN 机械臂控制器封装层。"""
from __future__ import annotations

from typing import Any


def __getattr__(name: str) -> Any:
    if name == "ArxDCanEndPose":
        from .arx_d_can_endpose_controller import ArxDCanEndPose

        return ArxDCanEndPose
    if name in {"GravityCompensationMode", "GravityCompensationSample"}:
        from .gravity_compensation import (
            GravityCompensationMode,
            GravityCompensationSample,
        )

        return {
            "GravityCompensationMode": GravityCompensationMode,
            "GravityCompensationSample": GravityCompensationSample,
        }[name]
    if name == "DualArmGravityCompensationMode":
        from .dual_gravity_compensation import DualArmGravityCompensationMode

        return DualArmGravityCompensationMode
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "ArxDCanEndPose",
    "DualArmGravityCompensationMode",
    "GravityCompensationMode",
    "GravityCompensationSample",
]
