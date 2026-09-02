from __future__ import annotations

from dataclasses import replace

import pytest

from arx_d_can import ArxDCanDualArm
from arx_d_can._dds.models import (
    ProductArmState,
    ProductGripperState,
    ProductState,
    RuntimeControlMode,
    SafetyState,
)


class FakeTransport:
    def __init__(self, mode: RuntimeControlMode = RuntimeControlMode.MIT) -> None:
        self.control_mode = mode
        self.has_grippers = True
        self.connected = False
        self.closed = False
        self.calls: list[tuple] = []
        arm = ProductArmState(
            positions=tuple(float(i) for i in range(7)),
            velocities=(0.1,) * 7,
            torques=(0.2,) * 7,
            enabled=(True,) * 7,
            mos_temperatures=(30.0,) * 7,
            rotor_temperatures=(31.0,) * 7,
        )
        self.state = ProductState(
            has_grippers=True,
            left=arm,
            right=replace(arm, positions=tuple(float(i + 7) for i in range(7))),
            left_gripper=ProductGripperState(False, 0.0, 0),
            right_gripper=ProductGripperState(False, 0.0, 0),
            motion_arrived=True,
            timestamp_ns=123,
            sequence=456,
        )
        self.health = type("Health", (), {"state": SafetyState.READY})()

    def connect(self) -> None:
        self.calls.append(("connect",))
        self.connected = True

    def disconnect(self) -> None:
        self.calls.append(("disconnect",))
        self.connected = False
        self.closed = True

    def enable(self, motors=None) -> bool:
        self.calls.append(("enable", motors))
        return True

    def disable(self, motors=None) -> bool:
        self.calls.append(("disable", motors))
        return True

    def configure_mode(self, mode) -> None:
        self.calls.append(("configure_mode", mode))
        self.control_mode = mode

    def set_joint_pv(self, positions, velocity) -> None:
        self.calls.append(("pv", tuple(positions), velocity))

    def set_joint_mit_fast(self, positions, velocity) -> None:
        self.calls.append(("mit_fast", tuple(positions), velocity))

    def set_joint_mit(self, positions, velocities, kp, kd, torques) -> None:
        self.calls.append(
            ("mit", tuple(positions), tuple(velocities), tuple(kp), tuple(kd), tuple(torques))
        )

    def get_fps(self) -> float:
        return 500.0


def test_constructor_forwards_dds_target_configuration(monkeypatch) -> None:
    captured = {}

    def create(**kwargs):
        captured.update(kwargs)
        return FakeTransport(kwargs["control_mode"])

    monkeypatch.setattr("arx_d_can.sdk.dual_arm.DdsRuntimeClient", create)
    robot = ArxDCanDualArm(
        robot_id="robot-2",
        domain_id=9,
        client_id="controller-a",
        robot_ip="192.168.1.185",
        network_interfaces=("eth0", "eth1"),
        control_mode="pv",
        discovery_timeout=3.0,
    )

    assert robot.control_mode == "pv"
    assert captured["robot_id"] == "robot-2"
    assert captured["domain_id"] == 9
    assert captured["client_id"] == "controller-a"
    assert captured["robot_ip"] == "192.168.1.185"
    assert captured["network_interfaces"] == ("eth0", "eth1")
    assert captured["control_mode"] is RuntimeControlMode.PV


def test_lifecycle_delegates_to_control_lease_session() -> None:
    transport = FakeTransport()
    robot = ArxDCanDualArm(_transport=transport)

    robot.connect()
    assert robot.connected
    assert robot.enable()
    assert robot.disable()
    robot.disconnect()

    assert transport.calls == [
        ("connect",),
        ("enable", None),
        ("disable", None),
        ("disconnect",),
    ]


def test_pv_and_mit_frames_keep_left_then_right_wire_order() -> None:
    pv_transport = FakeTransport(RuntimeControlMode.PV)
    pv = ArxDCanDualArm(control_mode="pv", _transport=pv_transport)
    pv.set_joint_pv(left=range(7), right=range(7, 14), velocity=40)
    assert pv_transport.calls[-1] == ("pv", tuple(float(i) for i in range(14)), 40)

    mit_transport = FakeTransport(RuntimeControlMode.MIT)
    mit = ArxDCanDualArm(_transport=mit_transport)
    mit.set_joint_mit(
        left_positions=(0.0,) * 7,
        right_positions=(1.0,) * 7,
        left_velocities=(2.0,) * 7,
        right_velocities=(3.0,) * 7,
        kp=10.0,
        kd=1.0,
        left_feedforward_torques=(4.0,) * 7,
        right_feedforward_torques=(5.0,) * 7,
    )
    call = mit_transport.calls[-1]
    assert call[0] == "mit"
    assert call[1] == (0.0,) * 7 + (1.0,) * 7
    assert call[2] == (2.0,) * 7 + (3.0,) * 7
    assert call[3] == (10.0,) * 14
    assert call[4] == (1.0,) * 14
    assert call[5] == (4.0,) * 7 + (5.0,) * 7


def test_cached_dds_state_is_exposed_without_client_side_control_logic() -> None:
    transport = FakeTransport()
    robot = ArxDCanDualArm(_transport=transport)
    state = robot.read_state()

    assert state.left.positions == pytest.approx(range(7))
    assert state.right.positions == pytest.approx(range(7, 14))
    assert state.left.arm.enabled == (True,) * 7
    assert state.left.gripper is None
    assert state.timestamp_ns == 123
    assert state.sequence == 456
    assert robot.get_fps() == 500.0


def test_unsupported_dds_v1_partial_power_and_waypoint_array_are_explicit() -> None:
    transport = FakeTransport(RuntimeControlMode.PV)
    robot = ArxDCanDualArm(control_mode="pv", _transport=transport)

    with pytest.raises(ValueError, match="whole-robot"):
        # The real DDS transport rejects this before sending.
        from arx_d_can._dds.client import DdsRuntimeClient

        bare = DdsRuntimeClient.__new__(DdsRuntimeClient)
        bare.enable(("l-joint1",))
    with pytest.raises(ValueError, match="waypoint arrays"):
        robot.move_linear(side="left", poses=((0.0,) * 6, (1.0,) * 6))
