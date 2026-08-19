"""ARX-D-CAN 高层 SDK 的公开导出。"""
from __future__ import annotations

import time

from arx_d_can._motor_abi import (
    ConnectChannelResult,
    ConnectErrorCode,
    ConnectMotorResult,
    ConnectReport,
    GravityCompensationPhase,
    GravityCompensationStatus,
    GravityProductBinding,
    GripperControlState,
    GripperHealth,
    RuntimeTransactionError,
    RuntimeTransportHealth,
    IkOptions,
    IkResult,
    JacobianReference,
    NativeRobotModel,
    MotorPowerState,
    ProductPose,
    RobotModelInfo,
    RobotPose,
    RobotSide,
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
    DualArmGripperState,
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
    DualArmGripperState,
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
    "DualArmGripperState",
    "ConnectChannelResult",
    "ConnectErrorCode",
    "ConnectMotorResult",
    "ConnectReport",
    "GravityCompensationPhase",
    "GravityCompensationStatus",
    "GravityProductBinding",
    "GripperState",
    "GripperControlState",
    "GripperForceLevel",
    "GripperHealth",
    "JointMotorConfig",
    "JointState",
    "MotorDiagnostic",
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
    "default_config",
]
