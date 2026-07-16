import numpy as np
import pytest

from arx_d_can.trajectory import plan_joint_position_trajectory


def test_min_jerk_joint_trajectory_samples_500_hz_and_keeps_endpoints():
    start = np.zeros(6)
    target = np.array([0.0, -1.0, -1.0, 0.0, 0.0, 0.0])

    points = plan_joint_position_trajectory(
        start,
        target,
        duration=6.0,
        hz=500.0,
    )

    assert len(points) == 3001
    assert points[0].time == 0.0
    assert points[-1].time == pytest.approx(6.0)
    np.testing.assert_array_equal(points[0].positions, start)
    np.testing.assert_allclose(points[-1].positions, target, atol=1e-12)
    np.testing.assert_allclose(points[1500].positions, target * 0.5, atol=1e-12)


@pytest.mark.parametrize("duration,hz", [(0.0, 500.0), (1.0, 0.0)])
def test_joint_trajectory_rejects_non_positive_timing(duration, hz):
    with pytest.raises(ValueError):
        plan_joint_position_trajectory([0.0], [1.0], duration=duration, hz=hz)


def test_joint_trajectory_rejects_mismatched_joint_counts():
    with pytest.raises(ValueError, match="same non-zero length"):
        plan_joint_position_trajectory([0.0], [0.0, 1.0], duration=1.0)
