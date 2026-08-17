"""ARX-D-CAN 高层 SDK 的公开导出。"""
from __future__ import annotations

import time

from motor_drive_layer import (
    ConnectChannelResult,
    ConnectErrorCode,
    ConnectMotorResult,
    ConnectReport,
    DisableMotorResult,
    DisableReport,
    EnableMotorResult,
    EnableReport,
    GripperControlState,
    GripperHealth,
    RuntimeTransactionError,
    RuntimeTransportHealth,
    SafetyHealth,
    SafetyState,
)

# 此处保留 ``load_cfg``，以兼容调用方以及通过 ``arx_d_can.sdk`` 检查配置加载过程的测试。
from ..actuator import load_cfg
from .arm import ArxDCanArm
from .config import (
    ArxDCanConfig,
    JointMotorConfig,
    default_config,
)
from .dual_arm import ArxDCanDualArm, ArxDCanDualArmState
from .diagnostics import MotorDiagnostic
from .gripper import GripperForceLevel
from .state import (
    ArxDCanState,
    GripperState,
    JointState,
    MotorState,
)

# 保留历史公开模块路径，确保对象 repr 和序列化数据兼容。
for _public_type in (
    ArxDCanArm,
    ArxDCanConfig,
    ArxDCanDualArm,
    ArxDCanDualArmState,
    ArxDCanState,
    GripperForceLevel,
    GripperState,
    JointMotorConfig,
    JointState,
    MotorDiagnostic,
    MotorState,
):
    _public_type.__module__ = __name__
del _public_type


__all__ = [
    "ArxDCanArm",
    "ArxDCanConfig",
    "ArxDCanDualArm",
    "ArxDCanDualArmState",
    "ArxDCanState",
    "ConnectChannelResult",
    "ConnectErrorCode",
    "ConnectMotorResult",
    "ConnectReport",
    "DisableMotorResult",
    "DisableReport",
    "EnableMotorResult",
    "EnableReport",
    "GripperState",
    "GripperControlState",
    "GripperForceLevel",
    "GripperHealth",
    "JointMotorConfig",
    "JointState",
    "MotorDiagnostic",
    "MotorState",
    "RuntimeTransactionError",
    "RuntimeTransportHealth",
    "SafetyHealth",
    "SafetyState",
    "default_config",
]
