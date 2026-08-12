"""ARX-D-CAN 高层 SDK 的公开导出。"""
from __future__ import annotations

import time

# 此处保留 ``load_cfg``，以兼容调用方以及通过 ``arx_d_can.sdk`` 检查配置加载过程的测试。
from ..actuator import load_cfg
from .arm import ArxDCanArm
from .config import ArxDCanConfig, JointMotorConfig, default_config
from .dual_arm import ArxDCanDualArm, ArxDCanDualArmState
from .diagnostics import MotorDiagnostic
from .native_safety import (
    EnableMotorResult,
    EnableReport,
    GripperControlState,
    GripperSafetyHealth,
    MissingEnableMotor,
    NativeEnableError,
    SafetyHealth,
    SafetyState,
    TransportHealth,
)
from .state import (
    ArxDCanState,
    GripperState,
    JointState,
    MitCommand,
    MotorState,
)

# 保留历史公开模块路径，确保对象 repr 和序列化数据兼容。
for _public_type in (
    ArxDCanArm,
    ArxDCanConfig,
    ArxDCanDualArm,
    ArxDCanDualArmState,
    ArxDCanState,
    EnableMotorResult,
    EnableReport,
    GripperControlState,
    GripperSafetyHealth,
    GripperState,
    JointMotorConfig,
    JointState,
    MitCommand,
    MotorDiagnostic,
    MotorState,
    MissingEnableMotor,
    NativeEnableError,
    SafetyHealth,
    SafetyState,
    TransportHealth,
):
    _public_type.__module__ = __name__
del _public_type


__all__ = [
    "ArxDCanArm",
    "ArxDCanConfig",
    "ArxDCanDualArm",
    "ArxDCanDualArmState",
    "ArxDCanState",
    "EnableMotorResult",
    "EnableReport",
    "GripperState",
    "GripperControlState",
    "GripperSafetyHealth",
    "JointMotorConfig",
    "JointState",
    "MitCommand",
    "MotorDiagnostic",
    "MotorState",
    "MissingEnableMotor",
    "NativeEnableError",
    "SafetyHealth",
    "SafetyState",
    "TransportHealth",
    "default_config",
]
