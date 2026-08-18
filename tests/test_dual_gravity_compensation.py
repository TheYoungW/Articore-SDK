from __future__ import annotations

from types import SimpleNamespace

import pytest
from motor_drive_layer import (
    GravityCompensationPhase,
    GravityCompensationStatus,
)

from arx_d_can.controllers.gravity_compensation import DualArmGravityCompensationMode


class _Arm:
    def __init__(self, name: str, mode: str = "mit") -> None:
        motor = object()
        joint = SimpleNamespace(name=name)
        self.joint_names = (name,)
        self._mode = mode
        self.config = SimpleNamespace(
            control_hz=500.0,
            arm_joints=(joint,),
        )
        self.robot = SimpleNamespace(_motor_map={name: motor})


class _Robot:
    def __init__(self, mode: str = "mit") -> None:
        self.left = _Arm("left-joint1", mode)
        self.right = _Arm("right-joint1", mode)
        self.connected = False
        self.enabled = False
        self.calls: list[tuple[str, object]] = []
        self.safety_health = SimpleNamespace(
            safe_holding=False,
            fault_reason=None,
        )
        self.left_position = 0.1
        self.right_position = -0.2
        self._status = GravityCompensationStatus(
            phase=GravityCompensationPhase.INACTIVE,
            active=False,
            transition_progress=0.0,
            control_cycles=0,
            joints=(),
            gravity_feedforward_torque=(),
        )

    @property
    def _effective_control_hz(self) -> float:
        return 400.0 if self.connected else 500.0

    @property
    def gravity_compensation_status(self) -> GravityCompensationStatus:
        return self._status

    @staticmethod
    def _side(position: float):
        return SimpleNamespace(
            arm=SimpleNamespace(
                positions=(position,),
                velocities=(0.0,),
            )
        )

    def connect(self) -> None:
        self.connected = True
        self.calls.append(("connect", None))

    def read_cached_state(self):
        self.calls.append(("read_cached_state", None))
        return SimpleNamespace(
            left=self._side(self.left_position),
            right=self._side(self.right_position),
        )

    def enable(self) -> None:
        self.enabled = True
        self.calls.append(("enable", None))

    def start_gravity_compensation(self, *, transition_ms: int) -> None:
        self.calls.append(("gravity_start", transition_ms))
        motors = (
            self.left.robot._motor_map["left-joint1"],
            self.right.robot._motor_map["right-joint1"],
        )
        self._status = GravityCompensationStatus(
            phase=GravityCompensationPhase.ACTIVE,
            active=True,
            transition_progress=1.0,
            control_cycles=3,
            joints=motors,
            gravity_feedforward_torque=(1.0, -2.0),
        )

    def stop_gravity_compensation(self) -> None:
        self.calls.append(("gravity_stop", None))
        self._status = GravityCompensationStatus(
            phase=GravityCompensationPhase.INACTIVE,
            active=False,
            transition_progress=0.0,
            control_cycles=0,
            joints=(),
            gravity_feedforward_torque=(),
        )

    def _submit_joint_positions(self, **_kwargs) -> None:
        raise AssertionError("Python compatibility layer must not submit MIT commands")

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
    )


def test_dual_gravity_delegates_control_to_native_runtime() -> None:
    robot = _Robot()
    gravity = _mode(robot)

    sample = gravity.start()

    assert gravity._update_hz == pytest.approx(400.0)
    assert ("gravity_start", 1) in robot.calls
    assert sample.left.commanded_torques == pytest.approx((1.0,))
    assert sample.right.commanded_torques == pytest.approx((-2.0,))

    gravity.step()
    assert [name for name, _ in robot.calls].count("gravity_start") == 1

    gravity.stop()
    assert not robot.connected
    assert not robot.enabled
    assert ("gravity_stop", None) in robot.calls
    assert robot.calls[-1] == ("close", None)


def test_dual_gravity_does_not_cap_diagnostic_poll_frequency() -> None:
    robot = _Robot()
    gravity = _mode(robot, hz=1000.0)

    gravity.start()
    try:
        assert gravity._update_hz == pytest.approx(1000.0)
    finally:
        gravity.stop()


def test_dual_gravity_rejects_removed_python_model_overrides() -> None:
    robot = _Robot()

    with pytest.raises(ValueError, match="Python gravity provider"):
        DualArmGravityCompensationMode(
            robot,
            left_gravity_provider=lambda _q: (1.0,),
        )
    with pytest.raises(ValueError, match="gravity_scale"):
        DualArmGravityCompensationMode(robot, gravity_scale=0.5)
    with pytest.raises(ValueError, match="拖拽阻尼"):
        DualArmGravityCompensationMode(robot, damping=0.1)


def test_dual_gravity_requires_mit_mode() -> None:
    robot = _Robot(mode="pv")
    gravity = _mode(robot)

    with pytest.raises(RuntimeError, match="control_mode='mit'"):
        gravity.start()

    assert not robot.connected
