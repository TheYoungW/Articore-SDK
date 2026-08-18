"""ARX-D-CAN Python SDK。

内置产品的运动学和动力学由 Motor Drive Layer 私有模型执行。导入
``arx_d_can`` 时不加载系统 Pinocchio；自定义旧模型和旧笛卡尔轨迹接口采用延迟加载。
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
    if name in {"NativeArmModel", "NativeIkResult", "load_native_robot_model"}:
        from .native_robotics import (
            NativeArmModel,
            NativeIkResult,
            load_native_robot_model,
        )

        return {
            "NativeArmModel": NativeArmModel,
            "NativeIkResult": NativeIkResult,
            "load_native_robot_model": load_native_robot_model,
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
    "GravityCompensationPhase",
    "GravityCompensationStatus",
    "GravityProductBinding",
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
    "IkOptions",
    "IkResult",
    "JacobianReference",
    "NativeArmModel",
    "NativeIkResult",
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
    "controllers",
    "default_config",
    "dynamics",
    "kinematics",
    "load_cfg",
    "load_native_robot_model",
    "trajectory",
]
