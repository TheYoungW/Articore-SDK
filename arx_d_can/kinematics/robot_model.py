"""reBot-DevArm 机器人模型加载模块 — 基于 Pinocchio。

默认的 urdf_path 和 end_effector_frame 来自 models.yaml 选中的默认机型。
"""

from pathlib import Path
from typing import List, Sequence, Tuple

import numpy as np
import pinocchio as pin

from ..actuator import load_cfg

_cfg_dir = Path(__file__).resolve().parents[1] / "config"
_project_root = _cfg_dir.parent

_hw_cfg_cache: dict | None = None


def _hw_config() -> dict:
    """Load kinematics fields (urdf_path, end_effector_frame) from the hardware YAML."""
    global _hw_cfg_cache
    if _hw_cfg_cache is not None:
        return _hw_cfg_cache

    _hw_cfg_cache = load_cfg()
    return _hw_cfg_cache


def _resolve_urdf(urdf_path: str | None = None) -> Tuple[str, str]:
    if urdf_path is None:
        urdf_path = _hw_config().get("urdf_path", "")

    if not urdf_path:
        raise ValueError("urdf_path is empty. Set it in the hardware config file.")

    if not Path(urdf_path).is_absolute():
        urdf_path = str(_project_root / urdf_path)

    pkg_dir = str(Path(urdf_path).resolve().parent)
    if pkg_dir.endswith("/urdf") or pkg_dir.endswith("\\urdf"):
        pkg_dir = str(Path(pkg_dir).parent)
    return urdf_path, pkg_dir


def load_robot_model(
    urdf_path: str | None = None,
    controlled_joint_names: Sequence[str] | None = None,
) -> pin.Model:
    """Load a URDF, optionally reducing it to the configured controlled joints.

    A multi-arm robot must keep one authoritative URDF.  Callers controlling
    only one arm pass that arm's joint names; every other movable joint is
    locked at the neutral configuration while the original frame tree and
    transforms are preserved.
    """
    path, _ = _resolve_urdf(urdf_path)
    model = pin.buildModelFromUrdf(path)
    if controlled_joint_names is None:
        return model

    requested = tuple(str(name) for name in controlled_joint_names)
    if not requested:
        raise ValueError("controlled_joint_names must not be empty")
    if len(requested) != len(set(requested)):
        raise ValueError("controlled_joint_names contains duplicate names")

    movable_joint_ids = {
        str(model.names[joint_id]): joint_id
        for joint_id in range(1, model.njoints)
        if model.joints[joint_id].nq > 0
    }
    unknown = set(requested).difference(movable_joint_ids)
    if unknown:
        raise ValueError(
            "controlled joints not found in URDF: " + ", ".join(sorted(unknown))
        )

    requested_set = set(requested)
    locked_joint_ids = [
        joint_id
        for name, joint_id in movable_joint_ids.items()
        if name not in requested_set
    ]
    reduced = pin.buildReducedModel(
        model,
        locked_joint_ids,
        pin.neutral(model),
    )
    reduced_names = tuple(get_joint_names(reduced))
    if reduced_names != requested:
        raise ValueError(
            "configured joint order does not match URDF model order: "
            f"configured={requested}, urdf={reduced_names}"
        )
    if reduced.nq != len(requested):
        raise ValueError(
            "each controlled motor joint must have exactly one position variable"
        )
    return reduced


def get_end_effector_frame() -> str:
    return _hw_config().get("end_effector_frame", "gripper_end")


def get_joint_count() -> int:
    model = load_robot_model()
    return model.nq


def get_joint_names(model: pin.Model) -> List[str]:
    return [n for n, j in zip(model.names[1:], model.joints[1:]) if j.idx_q >= 0]


def get_joint_limits(model: pin.Model) -> List[Tuple[float, float]]:
    limits = []
    for name, joint in zip(model.names[1:], model.joints[1:]):
        if joint.idx_q < 0:
            continue
        lo = float(model.lowerPositionLimit[joint.idx_q])
        hi = float(model.upperPositionLimit[joint.idx_q])
        limits.append((-np.inf, np.inf) if np.isinf(lo) and np.isinf(hi) else (lo, hi))
    return limits


def get_end_effector_frame_id(
    model: pin.Model,
    frame_name: str | None = None,
) -> int:
    return model.getFrameId(frame_name or get_end_effector_frame())


def get_all_frame_names(model: pin.Model) -> List[str]:
    return [f.name for f in model.frames]


def pad_q_for_model(model: pin.Model, q: np.ndarray, controlled_joints: int | None = None) -> np.ndarray:
    nq = model.nq
    n_ctrl = controlled_joints if controlled_joints is not None else nq
    padded = np.zeros(nq)
    padded[:min(q.shape[0], n_ctrl)] = q[:min(q.shape[0], n_ctrl)]
    return padded
