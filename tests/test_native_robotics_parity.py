from __future__ import annotations

import numpy as np
import pinocchio as pin
import pytest
from motor_drive_layer import RuntimeCallError

from arx_d_can.kinematics.robot_model import load_robot_model
from arx_d_can.native_robotics import JacobianReference, NativeArmModel


@pytest.mark.parametrize(
    ("product", "prefix"),
    [("yunyi_v1_0_left", "l"), ("yunyi_v1_0_right", "r")],
)
def test_native_model_matches_migration_pinocchio_model(product: str, prefix: str) -> None:
    legacy = load_robot_model(
        "models/yunyi_v1_0.urdf",
        controlled_joint_names=[f"{prefix}-joint{i}" for i in range(1, 8)]
    )
    data = legacy.createData()
    frame_id = legacy.getFrameId(f"{prefix}-link7")
    rng = np.random.default_rng(41 if prefix == "l" else 42)

    with NativeArmModel(product) as native:
        assert native.joint_names == tuple(f"{prefix}-joint{i}" for i in range(1, 8))
        for _ in range(20):
            q = rng.uniform(legacy.lowerPositionLimit * 0.7, legacy.upperPositionLimit * 0.7)
            dq = rng.normal(0.0, 0.5, 7)
            ddq = rng.normal(0.0, 1.0, 7)
            torque = rng.normal(0.0, 2.0, 7)

            pin.forwardKinematics(legacy, data, q)
            pin.updateFramePlacements(legacy, data)
            transform = data.oMf[frame_id]
            position, rotation, homogeneous = native.fk(q)
            np.testing.assert_allclose(position, transform.translation, atol=2e-15)
            np.testing.assert_allclose(rotation, transform.rotation, atol=2e-15)
            np.testing.assert_allclose(homogeneous, transform.homogeneous, atol=2e-15)

            pin.computeJointJacobians(legacy, data, q)
            expected_jacobian = pin.getFrameJacobian(legacy, data, frame_id, pin.LOCAL)
            np.testing.assert_allclose(
                native.jacobian(q, JacobianReference.LOCAL),
                expected_jacobian,
                atol=2e-14,
            )
            for native_reference, pin_reference in (
                (JacobianReference.WORLD, pin.WORLD),
                (
                    JacobianReference.LOCAL_WORLD_ALIGNED,
                    pin.LOCAL_WORLD_ALIGNED,
                ),
            ):
                np.testing.assert_allclose(
                    native.jacobian(q, native_reference),
                    pin.getFrameJacobian(
                        legacy,
                        data,
                        frame_id,
                        pin_reference,
                    ),
                    atol=2e-14,
                )
            np.testing.assert_allclose(
                native.gravity(q),
                pin.computeGeneralizedGravity(legacy, data, q),
                atol=2e-14,
            )
            np.testing.assert_allclose(
                native.mass_matrix(q), pin.crba(legacy, data, q), atol=2e-14
            )
            np.testing.assert_allclose(
                native.coriolis_matrix(q, dq),
                pin.computeCoriolisMatrix(legacy, data, q, dq),
                atol=2e-14,
            )
            np.testing.assert_allclose(
                native.nonlinear_effects(q, dq),
                pin.nonLinearEffects(legacy, data, q, dq),
                atol=2e-14,
            )
            np.testing.assert_allclose(
                native.rnea(q, dq, ddq),
                pin.rnea(legacy, data, q, dq, ddq),
                atol=2e-14,
            )
            np.testing.assert_allclose(
                native.aba(q, dq, torque),
                pin.aba(legacy, data, q, dq, torque),
                atol=2e-11,
            )


@pytest.mark.parametrize("product", ["yunyi_v1_0_left", "yunyi_v1_0_right"])
def test_native_ik_reaches_native_fk_target(product: str) -> None:
    with NativeArmModel(product) as model:
        target_q = 0.25 * (model.lower_position_limits + model.upper_position_limits)
        position, rotation, _ = model.fk(target_q)
        result = model.ik(position, rotation, np.zeros(7), random_seed=7)
        assert result.success
        assert result.error < 1e-4
        actual_position, actual_rotation, _ = model.fk(result.q)
        np.testing.assert_allclose(actual_position, position, atol=1e-4)
        np.testing.assert_allclose(actual_rotation, rotation, atol=2e-4)


def test_native_model_rejects_wrong_dimensions_and_non_finite_values() -> None:
    with NativeArmModel("yunyi_v1_0_left") as model:
        with pytest.raises(ValueError, match="exactly 7"):
            model.gravity([0.0] * 6)
        with pytest.raises(ValueError, match="finite"):
            model.fk([0.0] * 6 + [float("nan")])


def test_native_ik_rejects_invalid_rotation_matrix() -> None:
    with NativeArmModel("yunyi_v1_0_right") as model:
        with pytest.raises(RuntimeCallError, match="rotation"):
            model.ik(
                [0.0, 0.0, 0.0],
                [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.0]],
                [0.0] * 7,
            )
