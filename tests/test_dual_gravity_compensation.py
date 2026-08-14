from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from arx_d_can.controllers.gravity_compensation import DualArmGravityCompensationMode
from arx_d_can.sdk import ArxDCanConfig, JointMotorConfig


def _joint(name: str) -> JointMotorConfig:
    return JointMotorConfig(
        name=name,
        motor_id=1,
        feedback_id=0x11,
        model="4340P",
        mit_kp=10.0,
        mit_kd=1.0,
        pv_vel_kp=0.01,
        pv_vel_ki=0.001,
        pv_pos_kp=50.0,
        pv_pos_ki=0.5,
        pv_vlim=2.0,
        lower_limit=-1.0,
        upper_limit=1.0,
    )


class _Arm:
    def __init__(self, name: str, mode: str = "mit") -> None:
        self.joint_names = (name,)
        self._mode = mode
        self.config = ArxDCanConfig(
            arm_control_mode=mode,
            feedback_check_hz=100.0,
            soft_limit_margin=0.1,
            arm_joints=(_joint(name),),
        )


class _Robot:
    def __init__(self, mode: str = "mit") -> None:
        self.left = _Arm("left_joint", mode)
        self.right = _Arm("right_joint", mode)
        self.connected = False
        self.enabled = False
        self.calls: list[tuple[str, object]] = []
        self.commands: list[dict[str, object]] = []
        self.safety_health = SimpleNamespace(
            safe_holding=False,
            fault_reason=None,
        )
        self.left_position = 0.1
        self.right_position = -0.2

    @property
    def _effective_control_hz(self) -> float:
        return 400.0 if self.connected else 500.0

    @staticmethod
    def _side(position: float):
        return SimpleNamespace(
            arm=SimpleNamespace(
                positions=(position,),
                velocities=(0.0,),
            )
        )

    def _state(self):
        return SimpleNamespace(
            left=self._side(self.left_position),
            right=self._side(self.right_position),
        )

    def connect(self) -> None:
        self.connected = True
        self.calls.append(("connect", None))

    def read_state(self):
        self.calls.append(("read_state", None))
        return self._state()

    def read_cached_state(self):
        self.calls.append(("read_cached_state", None))
        return self._state()

    def enable(self, **kwargs) -> None:
        self.enabled = True
        self.calls.append(("enable", kwargs))

    def _submit_joint_positions(self, **kwargs) -> None:
        assert self.enabled
        self.commands.append(
            {
                key: np.asarray(value, dtype=float).copy()
                for key, value in kwargs.items()
            }
        )

    def disable(self) -> None:
        self.enabled = False
        self.calls.append(("disable", None))

    def close(self) -> None:
        self.connected = False
        self.enabled = False
        self.calls.append(("close", None))


def _mode(
    robot: _Robot,
    *,
    hz: float | None = None,
) -> DualArmGravityCompensationMode:
    return DualArmGravityCompensationMode(
        robot,
        hz=hz,
        transition_seconds=0.0,
        left_gravity_provider=lambda _positions: np.array([1.0]),
        right_gravity_provider=lambda _positions: np.array([-2.0]),
    )


def test_dual_gravity_uses_one_atomic_runtime_submission() -> None:
    robot = _Robot()
    gravity = _mode(robot)

    sample = gravity.start()

    assert gravity._update_hz == pytest.approx(400.0)
    enable_kwargs = next(value for name, value in robot.calls if name == "enable")
    assert enable_kwargs == {}
    assert len(robot.commands) == 1
    assert bool(robot.commands[0]["enforce_position_limits"])
    np.testing.assert_allclose(robot.commands[0]["left_torques"], [1.0])
    np.testing.assert_allclose(robot.commands[0]["right_torques"], [-2.0])
    assert sample.left.commanded_torques == pytest.approx((1.0,))
    assert sample.right.commanded_torques == pytest.approx((-2.0,))

    gravity.step()
    assert len(robot.commands) == 2

    gravity.stop()
    assert not robot.connected
    assert not robot.enabled
    assert robot.calls[-1] == ("close", None)


def test_dual_gravity_does_not_cap_user_call_frequency() -> None:
    robot = _Robot()
    gravity = _mode(robot, hz=1000.0)

    gravity.start()
    try:
        assert gravity._update_hz == pytest.approx(1000.0)
    finally:
        gravity.stop()


def test_dual_gravity_clips_each_feedback_hold_target() -> None:
    robot = _Robot()
    robot.left_position = 1.1
    robot.right_position = -1.2
    gravity = _mode(robot)

    sample = gravity.start()
    try:
        np.testing.assert_allclose(robot.commands[-1]["left"], [0.9])
        np.testing.assert_allclose(robot.commands[-1]["right"], [-0.9])
        assert sample.left.positions == pytest.approx((1.1,))
        assert sample.right.positions == pytest.approx((-1.2,))
        assert sample.left.clipped_joints == ("left_joint",)
        assert sample.right.clipped_joints == ("right_joint",)
    finally:
        gravity.stop()
    np.testing.assert_allclose(robot.commands[-1]["left"], [0.9])
    np.testing.assert_allclose(robot.commands[-1]["right"], [-0.9])


def test_dual_gravity_requires_mit_mode() -> None:
    robot = _Robot(mode="pv")
    gravity = _mode(robot)

    with pytest.raises(RuntimeError, match="control_mode='mit'"):
        gravity.start()

    assert not robot.connected
