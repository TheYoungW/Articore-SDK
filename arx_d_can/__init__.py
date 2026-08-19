"""ARX-D-CAN Python SDK。

内置产品的运动学和动力学由 Motor Drive Layer 私有模型执行，Python 只调用稳定 ABI。
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
    DualArmGripperState,
    ConnectChannelResult,
    ConnectErrorCode,
    ConnectMotorResult,
    ConnectReport,
    GravityCompensationPhase,
    GravityCompensationStatus,
    GravityProductBinding,
    GripperState,
    GripperControlState,
    GripperForceLevel,
    GripperHealth,
    JointMotorConfig,
    JointState,
    MotorDiagnostic,
    MotorPowerState,
    ProductPose,
    MotorState,
    RuntimeTransactionError,
    RuntimeTransportHealth,
    IkOptions,
    IkResult,
    JacobianReference,
    NativeRobotModel,
    RobotModelInfo,
    RobotPose,
    RobotSide,
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
    if name == "actuator":
        return importlib.import_module(f"{__name__}.{name}")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "ArxDCanError",
    "ArxDCanArm",
    "ArxDCanConfig",
    "ArxDCanDualArm",
    "ArxDCanDualArmState",
    "ArxDCanState",
    "DualArmGripperState",
    "CommunicationError",
    "ConnectChannelResult",
    "ConnectErrorCode",
    "ConnectMotorResult",
    "ConnectReport",
    "GravityCompensationPhase",
    "GravityCompensationStatus",
    "GravityProductBinding",
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
    "MotorPowerState",
    "ProductPose",
    "RuntimeTransactionError",
    "RuntimeTransportHealth",
    "IkOptions",
    "IkResult",
    "JacobianReference",
    "NativeRobotModel",
    "RobotModelInfo",
    "RobotPose",
    "RobotSide",
    "SafetyHealth",
    "SafetyState",
    "MotorFaultError",
    "MotorDiagnostic",
    "TransportError",
    "UnexpectedMotorStateError",
    "actuator",
    "available_models",
    "default_config",
    "load_cfg",
]
