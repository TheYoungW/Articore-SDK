"""ARX-D-CAN 机械臂控制器封装层。"""
from __future__ import annotations

from typing import Any


def __getattr__(name: str) -> Any:
    if name in {
        "DualArmGravityCompensationMode",
        "DualArmGravityCompensationSample",
    }:
        from .gravity_compensation import (
            DualArmGravityCompensationMode,
            DualArmGravityCompensationSample,
        )

        return {
            "DualArmGravityCompensationMode": DualArmGravityCompensationMode,
            "DualArmGravityCompensationSample": DualArmGravityCompensationSample,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "DualArmGravityCompensationMode",
    "DualArmGravityCompensationSample",
]
