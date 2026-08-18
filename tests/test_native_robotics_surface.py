from __future__ import annotations

import numpy as np

from arx_d_can import dynamics


def test_dynamics_namespace_exposes_only_native_model_facade() -> None:
    assert set(dynamics.__all__) == {
        "JacobianReference",
        "NativeArmModel",
        "NativeIkResult",
        "RobotSide",
        "load_native_robot_model",
    }
    assert not hasattr(dynamics, "compute_mass_matrix")
    assert not hasattr(dynamics, "load_dynamics_model")


def test_native_model_executes_rigid_body_dynamics_in_runtime() -> None:
    with dynamics.load_native_robot_model("yunyi_v1_0_right") as model:
        q = np.zeros(model.dof)
        dq = np.zeros(model.dof)
        torque = np.ones(model.dof)

        position, rotation, homogeneous = model.fk(q)
        outputs = (
            position,
            rotation,
            homogeneous,
            model.jacobian(q),
            model.gravity(q),
            model.mass_matrix(q),
            model.coriolis_matrix(q, dq),
            model.nonlinear_effects(q, dq),
            model.rnea(q, dq, dq),
            model.aba(q, dq, torque),
        )

        assert position.shape == (3,)
        assert rotation.shape == (3, 3)
        assert homogeneous.shape == (4, 4)
        assert all(np.all(np.isfinite(value)) for value in outputs)
