"""Robot dynamics with a Runtime-owned model and optional legacy helpers."""
from __future__ import annotations

from ..native_robotics import NativeArmModel, load_native_robot_model


def _pinocchio_required(*_args, **_kwargs):
    raise RuntimeError(
        "this legacy custom-model API requires the optional 'dynamics' "
        "dependency; built-in products should use NativeArmModel methods"
    )


try:
    from .robot_model import (
        load_dynamics_model, get_default_gravity, set_gravity, get_gravity,
        neutral_configuration, random_configuration,
    )
    from .inertia import (
        compute_mass_matrix, compute_coriolis_matrix, compute_gravity_vector,
        compute_nle, compute_all_terms,
    )
    from .forward_dynamics import compute_forward_dynamics, forward_dynamics_from_nle
    from .inverse_dynamics import (
        compute_inverse_dynamics, compute_generalized_gravity, compute_static_torque,
    )
    from .derivatives import (
        compute_mass_matrix_derivatives, compute_coriolis_derivatives,
        compute_rnea_derivatives, compute_generalized_gravity_derivatives,
    )
    from .energy import compute_kinetic_energy, compute_potential_energy, compute_total_energy
    from .centroidal import (
        compute_center_of_mass, compute_com_velocity, compute_centroidal_matrix,
        compute_centroidal_momentum,
    )
except ModuleNotFoundError as exc:
    if exc.name != "pinocchio":
        raise
    load_dynamics_model = get_default_gravity = set_gravity = get_gravity = _pinocchio_required
    neutral_configuration = random_configuration = _pinocchio_required
    compute_mass_matrix = compute_coriolis_matrix = compute_gravity_vector = _pinocchio_required
    compute_nle = compute_all_terms = compute_forward_dynamics = _pinocchio_required
    forward_dynamics_from_nle = compute_inverse_dynamics = _pinocchio_required
    compute_generalized_gravity = compute_static_torque = _pinocchio_required
    compute_mass_matrix_derivatives = compute_coriolis_derivatives = _pinocchio_required
    compute_rnea_derivatives = compute_generalized_gravity_derivatives = _pinocchio_required
    compute_kinetic_energy = compute_potential_energy = compute_total_energy = _pinocchio_required
    compute_center_of_mass = compute_com_velocity = compute_centroidal_matrix = _pinocchio_required
    compute_centroidal_momentum = _pinocchio_required


__all__ = [
    "NativeArmModel", "load_native_robot_model", "load_dynamics_model",
    "get_default_gravity", "set_gravity", "get_gravity", "neutral_configuration",
    "random_configuration", "compute_mass_matrix", "compute_coriolis_matrix",
    "compute_gravity_vector", "compute_nle", "compute_all_terms",
    "compute_forward_dynamics", "forward_dynamics_from_nle",
    "compute_inverse_dynamics", "compute_generalized_gravity", "compute_static_torque",
    "compute_mass_matrix_derivatives", "compute_coriolis_derivatives",
    "compute_rnea_derivatives", "compute_generalized_gravity_derivatives",
    "compute_kinetic_energy", "compute_potential_energy", "compute_total_energy",
    "compute_center_of_mass", "compute_com_velocity", "compute_centroidal_matrix",
    "compute_centroidal_momentum",
]
