from __future__ import annotations

import math

from arx_d_can import NativeRobotModel, RobotSide


def test_native_model_executes_rigid_body_dynamics_in_runtime() -> None:
    with NativeRobotModel(side=RobotSide.RIGHT) as model:
        q = [0.0] * model.info.dof
        dq = [0.0] * model.info.dof
        torque = [1.0] * model.info.dof

        pose = model.fk(q)
        outputs = (
            pose.position,
            pose.rotation,
            pose.homogeneous,
            model.jacobian(q),
            model.gravity(q),
            model.mass_matrix(q),
            model.coriolis_matrix(q, dq),
            model.nonlinear_effects(q, dq),
            model.rnea(q, dq, dq),
            model.aba(q, dq, torque),
        )

        assert len(pose.position) == 3
        assert len(pose.rotation) == 3
        assert len(pose.homogeneous) == 4
        assert all(
            math.isfinite(value)
            for output in outputs
            for row in output
            for value in (row if isinstance(row, tuple) else (row,))
        )
