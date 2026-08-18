"""Robot kinematics.

Built-in products use the private Articore Runtime model. The historical
Python Pinocchio surface remains available as an optional migration path.
"""
from __future__ import annotations

from ..native_robotics import (
    JacobianReference,
    NativeArmModel,
    NativeIkResult,
    RobotSide,
    load_native_robot_model,
)


def _pinocchio_required(*_args, **_kwargs):
    raise RuntimeError(
        "this legacy custom-model API requires the optional 'legacy-models' "
        "dependency; built-in products should use load_native_robot_model()"
    )


try:
    from .robot_model import (
        load_robot_model,
        get_end_effector_frame,
        get_joint_count,
        _resolve_urdf,
        get_joint_names,
        get_joint_limits,
        get_end_effector_frame_id,
        get_all_frame_names,
        pad_q_for_model,
    )
    from .forward_kinematics import compute_fk, joint_to_pose
    from .inverse_kinematics import (
        compute_ik,
        solve_ik_with_retry,
        solve_ik,
        pos_rot_to_se3,
        IKResult,
        IKSolverParams,
    )
except ModuleNotFoundError as exc:
    if exc.name != "pinocchio":
        raise
    load_robot_model = get_end_effector_frame = get_joint_count = _pinocchio_required
    _resolve_urdf = get_joint_names = get_joint_limits = _pinocchio_required
    get_end_effector_frame_id = get_all_frame_names = pad_q_for_model = _pinocchio_required
    compute_fk = joint_to_pose = compute_ik = _pinocchio_required
    solve_ik_with_retry = solve_ik = pos_rot_to_se3 = _pinocchio_required
    IKResult = NativeIkResult
    IKSolverParams = None


__all__ = [
    "NativeArmModel", "NativeIkResult", "RobotSide", "JacobianReference",
    "load_native_robot_model", "load_robot_model", "get_end_effector_frame",
    "get_joint_count", "_resolve_urdf", "get_joint_names", "get_joint_limits",
    "get_end_effector_frame_id", "get_all_frame_names", "pad_q_for_model",
    "compute_fk", "joint_to_pose", "compute_ik", "solve_ik_with_retry",
    "solve_ik", "pos_rot_to_se3", "IKResult", "IKSolverParams",
]
