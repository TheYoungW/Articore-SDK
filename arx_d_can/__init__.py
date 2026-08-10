"""ARX-D-CAN Python SDK。

本包保持电机控制路径的依赖精简：导入 ``arx_d_can`` 时只加载 USB2CAN SDK 组件。
运动学、动力学、轨迹规划和末端位姿控制器依赖 Pinocchio，因此采用延迟加载。
"""
from __future__ import annotations

import importlib
from typing import Any

from .actuator import ArxDCan, JointCfg, JointGroup, available_models, load_cfg
from .errors import (
    ArxDCanError,
    CommandTimeoutError,
    CommunicationError,
    FeedbackError,
    FeedbackTimeoutError,
    IncompleteFeedbackError,
    MotorFaultError,
    StaleFeedbackError,
    TransportError,
    UnexpectedMotorStateError,
)
from .diagnostics import MotorDiagnostic
from .sdk import (
    ArxDCanArm,
    ArxDCanConfig,
    ArxDCanState,
    CoupledControlStats,
    CoupledTorqueTelemetry,
    CoupledTorqueSaturation,
    CommunicationHealth,
    GripperState,
    JointMotorConfig,
    JointState,
    MitCommand,
    MotorState,
    default_config,
)


def __getattr__(name: str) -> Any:
    if name == "ArxDCanEndPose":
        from .controllers import ArxDCanEndPose

        return ArxDCanEndPose
    if name in {"GravityCompensationMode", "GravityCompensationSample"}:
        from .controllers import (
            GravityCompensationMode,
            GravityCompensationSample,
        )

        return {
            "GravityCompensationMode": GravityCompensationMode,
            "GravityCompensationSample": GravityCompensationSample,
        }[name]
    if name in {"actuator", "controllers", "dynamics", "kinematics", "trajectory"}:
        return importlib.import_module(f"{__name__}.{name}")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "ArxDCan",
    "ArxDCanError",
    "ArxDCanArm",
    "ArxDCanConfig",
    "ArxDCanEndPose",
    "ArxDCanState",
    "CoupledControlStats",
    "CoupledTorqueTelemetry",
    "CoupledTorqueSaturation",
    "CommandTimeoutError",
    "CommunicationError",
    "CommunicationHealth",
    "FeedbackError",
    "FeedbackTimeoutError",
    "GravityCompensationMode",
    "GravityCompensationSample",
    "GripperState",
    "JointCfg",
    "JointGroup",
    "JointMotorConfig",
    "JointState",
    "IncompleteFeedbackError",
    "MitCommand",
    "MotorState",
    "MotorFaultError",
    "MotorDiagnostic",
    "StaleFeedbackError",
    "TransportError",
    "UnexpectedMotorStateError",
    "actuator",
    "available_models",
    "controllers",
    "default_config",
    "dynamics",
    "kinematics",
    "load_cfg",
    "trajectory",
]
