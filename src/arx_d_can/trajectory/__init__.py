"""ARX-D-CAN trajectory planning APIs.

Joint-position sampling stays dependency-light. Cartesian and CLIK APIs are
loaded lazily because they require Pinocchio.
"""
from __future__ import annotations

import importlib
from typing import Any

from .joint_sampler import JointPositionPoint, plan_joint_position_trajectory


_LAZY_EXPORTS = {
    "TrajProfile": ("sampler", "TrajProfile"),
    "TrajPlanParams": ("sampler", "TrajPlanParams"),
    "CartesianPoint": ("sampler", "CartesianPoint"),
    "CartesianTrajectory": ("sampler", "CartesianTrajectory"),
    "CartesianTrajectoryResult": ("sampler", "CartesianTrajectoryResult"),
    "plan_cartesian_geodesic_trajectory": (
        "sampler",
        "plan_cartesian_geodesic_trajectory",
    ),
    "IKParams": ("clik_tracker", "IKParams"),
    "CLIKParams": ("clik_tracker", "IKParams"),
    "JointTrajectoryPoint": ("clik_tracker", "JointTrajectoryPoint"),
    "track_trajectory": ("clik_tracker", "track_trajectory"),
    "TrajStats": ("trajectory_planner", "TrajStats"),
    "plan_joint_space_trajectory": (
        "trajectory_planner",
        "plan_joint_space_trajectory",
    ),
    "compute_traj_stats": ("trajectory_planner", "compute_traj_stats"),
}


def __getattr__(name: str) -> Any:
    try:
        module_name, attribute = _LAZY_EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from exc
    module = importlib.import_module(f"{__name__}.{module_name}")
    value = getattr(module, attribute)
    globals()[name] = value
    return value


__all__ = [
    "JointPositionPoint",
    "plan_joint_position_trajectory",
    *_LAZY_EXPORTS,
]
