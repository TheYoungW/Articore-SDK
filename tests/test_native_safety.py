from __future__ import annotations

import ctypes
import pytest

from arx_d_can.sdk.native_safety import (
    GripperControlState,
    NativeSafetyRuntime,
    SafetyState,
    _SafetyHealth,
)


class FakeRuntimeLibrary:
    def articore_runtime_get_health(self, _runtime, output) -> int:
        health = ctypes.cast(output, ctypes.POINTER(_SafetyHealth)).contents
        health.state = 4
        health.safe_holding = 1
        health.disable_confirmed = 0
        health.last_successful_command_age_ns = 25_000_000
        health.last_fresh_feedback_age_ns = 2_000_000
        health.consecutive_send_failures = 1
        health.consecutive_feedback_failures = 3
        health.left_transport.connected = 1
        health.left_transport.healthy = 1
        health.left_transport.last_feedback_age_ns = 2_000_000
        health.left_transport.tx_frames = 100
        health.left_transport.rx_frames = 99
        health.left_transport.last_tx_age_ns = 1_000_000
        health.left_transport.last_rx_age_ns = 2_000_000
        health.right_transport.connected = 0
        health.right_transport.healthy = 0
        health.right_transport.consecutive_feedback_failures = 3
        health.right_transport.last_feedback_age_ns = (1 << 64) - 1
        health.right_transport.send_errors = 2
        health.right_transport.receive_errors = 4
        health.right_transport.last_tx_age_ns = (1 << 64) - 1
        health.right_transport.last_rx_age_ns = (1 << 64) - 1
        health.right_transport.last_error = b"device disconnected"
        health.gripper_count = 1
        health.grippers[0].available = 1
        health.grippers[0].side = 0
        health.grippers[0].control_state = 4
        health.grippers[0].opening = 375.0
        health.grippers[0].motor_position = 1.25
        health.grippers[0].torque = 0.9
        health.grippers[0].contact_detected = 1
        health.grippers[0].stalled = 1
        health.grippers[0].has_hold_target = 1
        health.grippers[0].hold_target = 1.3
        health.grippers[0].feedback_age_ns = 3_000_000
        health.grippers[0].name = b"left/l-gripper"
        health.fault_reason = b"consecutive feedback failures"
        return 0

    def articore_runtime_last_error(self):
        return b"ok"


def test_native_health_is_exposed_as_immutable_python_values() -> None:
    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = FakeRuntimeLibrary()
    runtime._ptr = 1

    health = runtime.health

    assert health.state is SafetyState.SAFE_HOLD
    assert health.safe_holding
    assert not health.disable_confirmed
    assert health.last_successful_command_age_s == 0.025
    assert health.left_transport.tx_frames == 100
    assert health.left_transport.last_rx_age_s == 0.002
    assert health.right_transport.connected is False
    assert health.right_transport.last_feedback_age_s is None
    assert health.right_transport.last_tx_age_s is None
    assert health.right_transport.last_error == "device disconnected"
    assert health.fault_reason == "consecutive feedback failures"
    assert health.left_gripper is not None
    assert health.left_gripper.control_state is GripperControlState.HOLDING
    assert health.left_gripper.opening == 375.0
    assert health.left_gripper.contact_detected
    assert health.left_gripper.stalled
    assert health.left_gripper.hold_target == pytest.approx(1.3)
    assert health.left_gripper.feedback_age_s == 0.003
    assert health.right_gripper is None
