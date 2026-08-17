"""ARX-D-CAN Python SDK。

本包保持电机控制路径的依赖精简：导入 ``arx_d_can`` 时只加载 USB2CAN SDK 组件。
运动学、动力学和轨迹规划依赖 Pinocchio，因此采用延迟加载。
"""
from __future__ import annotations

import importlib
from typing import Any

from .actuator import available_models, load_cfg
from .errors import (
    ArxDCanError,
    CommunicationError,
    FeedbackError,
    FeedbackTimeoutError,
    IncompleteFeedbackError,
    MotorFaultError,
    TransportError,
    UnexpectedMotorStateError,
)
from .sdk import (
    ArxDCanArm,
    ArxDCanConfig,
    ArxDCanDualArm,
    ArxDCanDualArmState,
    ArxDCanState,
    ConnectChannelResult,
    ConnectErrorCode,
    ConnectMotorResult,
    ConnectReport,
    DisableMotorResult,
    DisableReport,
    EnableMotorResult,
    EnableReport,
    GripperState,
    GripperControlState,
    GripperForceLevel,
    GripperHealth,
    JointMotorConfig,
    JointState,
    MotorDiagnostic,
    MotorState,
    RuntimeTransactionError,
    RuntimeTransportHealth,
    SafetyHealth,
    SafetyState,
    default_config,
)

# 异常实现由各层内部共享，但公共类型身份统一归属于包级 API。
for _public_error in (
    ArxDCanError,
    CommunicationError,
    FeedbackError,
    FeedbackTimeoutError,
    IncompleteFeedbackError,
    MotorFaultError,
    TransportError,
    UnexpectedMotorStateError,
):
    _public_error.__module__ = __name__
del _public_error


def __getattr__(name: str) -> Any:
    if name in {
        "DualArmGravityCompensationMode",
        "DualArmGravityCompensationSample",
    }:
        from .controllers import (
            DualArmGravityCompensationMode,
            DualArmGravityCompensationSample,
        )

        return {
            "DualArmGravityCompensationMode": DualArmGravityCompensationMode,
            "DualArmGravityCompensationSample": DualArmGravityCompensationSample,
        }[name]
    if name in {"actuator", "controllers", "dynamics", "kinematics", "trajectory"}:
        return importlib.import_module(f"{__name__}.{name}")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "ArxDCanError",
    "ArxDCanArm",
    "ArxDCanConfig",
    "ArxDCanDualArm",
    "ArxDCanDualArmState",
    "ArxDCanState",
    "CommunicationError",
    "ConnectChannelResult",
    "ConnectErrorCode",
    "ConnectMotorResult",
    "ConnectReport",
    "DisableMotorResult",
    "DisableReport",
    "EnableMotorResult",
    "EnableReport",
    "DualArmGravityCompensationMode",
    "DualArmGravityCompensationSample",
    "FeedbackError",
    "FeedbackTimeoutError",
    "GripperState",
    "GripperControlState",
    "GripperForceLevel",
    "GripperHealth",
    "JointMotorConfig",
    "JointState",
    "IncompleteFeedbackError",
    "MotorState",
    "RuntimeTransactionError",
    "RuntimeTransportHealth",
    "SafetyHealth",
    "SafetyState",
    "MotorFaultError",
    "MotorDiagnostic",
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
