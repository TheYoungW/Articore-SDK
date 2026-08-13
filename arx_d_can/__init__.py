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
    DisableMotorResult,
    DisableReport,
    EnableMotorResult,
    EnableReport,
    GripperState,
    GripperControlState,
    GripperForceLevel,
    GripperSafetyHealth,
    JointMotorConfig,
    JointState,
    MotorDiagnostic,
    MotorState,
    MissingEnableMotor,
    MissingDisableMotor,
    NativeDisableError,
    NativeEnableError,
    SafetyHealth,
    SafetyState,
    TransportHealth,
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
    "ArxDCan",
    "ArxDCanError",
    "ArxDCanArm",
    "ArxDCanConfig",
    "ArxDCanDualArm",
    "ArxDCanDualArmState",
    "ArxDCanEndPose",
    "ArxDCanState",
    "CommunicationError",
    "DisableMotorResult",
    "DisableReport",
    "EnableMotorResult",
    "EnableReport",
    "DualArmGravityCompensationMode",
    "DualArmGravityCompensationSample",
    "FeedbackError",
    "FeedbackTimeoutError",
    "GravityCompensationMode",
    "GravityCompensationSample",
    "GripperState",
    "GripperControlState",
    "GripperForceLevel",
    "GripperSafetyHealth",
    "JointCfg",
    "JointGroup",
    "JointMotorConfig",
    "JointState",
    "IncompleteFeedbackError",
    "MotorState",
    "MissingEnableMotor",
    "MissingDisableMotor",
    "NativeDisableError",
    "NativeEnableError",
    "SafetyHealth",
    "SafetyState",
    "MotorFaultError",
    "MotorDiagnostic",
    "TransportError",
    "TransportHealth",
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
