"""ARX-D-CAN Python SDK.

The package keeps the motor-control path light: importing ``arx_d_can`` only
loads the USB2CAN SDK pieces. Kinematics, dynamics, trajectory planning, and
end-pose controllers are imported lazily because they require Pinocchio.
"""
from __future__ import annotations

import importlib
from typing import Any

from .actuator import ArxDCan, JointCfg, JointGroup, available_models, load_cfg
from .sdk import (
    ArxDCanArm,
    ArxDCanConfig,
    ArxDCanState,
    JointMotorConfig,
    JointState,
    MotorState,
    default_config,
)


def __getattr__(name: str) -> Any:
    if name == "ArxDCanEndPose":
        from .controllers import ArxDCanEndPose

        return ArxDCanEndPose
    if name in {"actuator", "controllers", "dynamics", "kinematics", "trajectory"}:
        return importlib.import_module(f"{__name__}.{name}")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "ArxDCan",
    "ArxDCanArm",
    "ArxDCanConfig",
    "ArxDCanEndPose",
    "ArxDCanState",
    "JointCfg",
    "JointGroup",
    "JointMotorConfig",
    "JointState",
    "MotorState",
    "actuator",
    "available_models",
    "controllers",
    "default_config",
    "dynamics",
    "kinematics",
    "load_cfg",
    "trajectory",
]
