"""Dynamics 动力学库 — 基于 Pinocchio 的刚体动力学参数解算。"""

from .robot_model import (
    load_dynamics_model,
    get_default_gravity,
    set_gravity,
    get_gravity,
    neutral_configuration,
    random_configuration,
)
from .inertia import (
    compute_mass_matrix,
    compute_coriolis_matrix,
    compute_gravity_vector,
    compute_nle,
    compute_all_terms,
)
from .forward_dynamics import (
    compute_forward_dynamics,
    forward_dynamics_from_nle,
)
from .inverse_dynamics import (
    compute_inverse_dynamics,
    compute_generalized_gravity,
    compute_static_torque,
)
from .derivatives import (
    compute_mass_matrix_derivatives,
    compute_coriolis_derivatives,
    compute_rnea_derivatives,
    compute_generalized_gravity_derivatives,
)
from .energy import (
    compute_kinetic_energy,
    compute_potential_energy,
    compute_total_energy,
)
from .centroidal import (
    compute_center_of_mass,
    compute_com_velocity,
    compute_centroidal_matrix,
    compute_centroidal_momentum,
)

__all__ = [
    # 机器人模型
    "load_dynamics_model",
    "get_default_gravity",
    "set_gravity",
    "get_gravity",
    "neutral_configuration",
    "random_configuration",
    # 惯性与非线性项
    "compute_mass_matrix",
    "compute_coriolis_matrix",
    "compute_gravity_vector",
    "compute_nle",
    "compute_all_terms",
    # 正动力学
    "compute_forward_dynamics",
    "forward_dynamics_from_nle",
    # 逆动力学
    "compute_inverse_dynamics",
    "compute_generalized_gravity",
    "compute_static_torque",
    # 导数
    "compute_mass_matrix_derivatives",
    "compute_coriolis_derivatives",
    "compute_rnea_derivatives",
    "compute_generalized_gravity_derivatives",
    # 能量
    "compute_kinetic_energy",
    "compute_potential_energy",
    "compute_total_energy",
    # 质心动力学
    "compute_center_of_mass",
    "compute_com_velocity",
    "compute_centroidal_matrix",
    "compute_centroidal_momentum",
]
