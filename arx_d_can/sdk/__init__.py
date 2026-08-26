"""Yunyi 双臂高层接口。"""
from __future__ import annotations

from .dual_arm import ArxDCanDualArm, ArxDCanDualArmState, JointLimit
from .state import ArxDCanState, DualArmGripperState, JointState

for _public_type in (
    ArxDCanDualArm,
    ArxDCanDualArmState,
    ArxDCanState,
    DualArmGripperState,
    JointState,
    JointLimit,
):
    _public_type.__module__ = __name__
del _public_type

__all__ = [
    "ArxDCanDualArm",
    "ArxDCanDualArmState",
    "ArxDCanState",
    "DualArmGripperState",
    "JointState",
    "JointLimit",
]
