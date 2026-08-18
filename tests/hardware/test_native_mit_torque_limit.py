"""Safe real-hardware acceptance for Runtime ABI 2.6 MIT torque limiting.

The test lowers only left/right J7's Runtime-local torque limit so limiter
activation can be observed far below the physical motor effort limit. It does
not change YAML, URDF, motor registers, or firmware.
"""
from __future__ import annotations

from dataclasses import replace
import math
import os
import statistics
import time
from types import MethodType

import pytest

from arx_d_can import ArxDCanDualArm, SafetyState


TEST_TORQUE_LIMIT = 0.5
TEST_APPLIED_LIMIT = TEST_TORQUE_LIMIT
# Yunyi J7 maps its configured 7 N.m range to the 4310 native 10-unit range.
# 0.34 N.m therefore becomes about 0.486 native units. A small position error
# adds a P term so the complete requested result exceeds the 0.5 test limit,
# while every individual submitted field remains within its valid range.
PULSE_TORQUE = 0.34
PULSE_POSITION_OFFSET = 0.01
PUBLISH_HZ = 100.0
HOLD_SECONDS = 2.0


def _feedback_stats(robot: ArxDCanDualArm):
    result = {}
    for side_name, arm in (("left", robot.left), ("right", robot.right)):
        for name, stats in arm.robot.get_feedback_stats(
            joint_names=arm._active_joint_names()
        ).items():
            result[f"{side_name}/{name}"] = stats
    return result


def _install_safe_j7_test_limits(robot: ArxDCanDualArm) -> None:
    original = robot._runtime_joint_configs

    def configured_with_safe_j7_limits(self):
        configs = list(original())
        assert len(configs) == 14
        for index in (6, 13):
            configs[index] = replace(
                configs[index], torque_limit=TEST_TORQUE_LIMIT
            )
        return tuple(configs)

    robot._runtime_joint_configs = MethodType(  # type: ignore[method-assign]
        configured_with_safe_j7_limits, robot
    )


def test_native_mit_torque_limit_on_every_repeated_cycle() -> None:
    if os.environ.get("ARX_D_CAN_RUN_HARDWARE_TEST") != "1":
        pytest.skip("set ARX_D_CAN_RUN_HARDWARE_TEST=1 to use real hardware")

    robot = ArxDCanDualArm(
        control_mode="mit",
        transport="socketcanfd",
        left_channel="can-left",
        right_channel="can-right",
    )
    _install_safe_j7_test_limits(robot)
    disable_report = None
    connected = False
    try:
        robot.connect()
        connected = True
        runtime = robot._safety_runtime
        assert runtime is not None
        assert runtime.control_hz == 500
        robot.enable()

        initial = robot.read_cached_state()
        left_target = tuple(initial.left.arm.positions)
        right_target = tuple(initial.right.arm.positions)
        zeros = (0.0,) * 7
        def j7_pulse(values: tuple[float, ...]):
            direction = -1.0 if values[6] > 0.0 else 1.0
            target = values[:6] + (values[6] + direction * PULSE_POSITION_OFFSET,)
            torque = (0.0,) * 6 + (direction * PULSE_TORQUE,)
            return target, torque

        left_pulse_target, left_pulse = j7_pulse(left_target)
        right_pulse_target, right_pulse = j7_pulse(right_target)
        pulse_kp = (0.0,) * 6 + (28.0,)
        pulse_kd = zeros

        # Establish an ordinary hold before measuring the deliberately low
        # J7 test limit.
        robot.submit_raw_mit(
            left_positions=left_target,
            right_positions=right_target,
            left_feedforward_torques=zeros,
            right_feedforward_torques=zeros,
        )
        time.sleep(0.03)
        activation_before = (
            runtime.mit_torque_limit_stats.torque_limit_activation_count
        )

        # One Python submission is intentionally left in the mailbox. The
        # activation counter must continue increasing as the 500 Hz worker
        # repeats and re-evaluates it against new native feedback.
        robot.submit_raw_mit(
            left_positions=left_pulse_target,
            right_positions=right_pulse_target,
            kp=pulse_kp,
            kd=pulse_kd,
            left_feedforward_torques=left_pulse,
            right_feedforward_torques=right_pulse,
        )
        time.sleep(0.006)
        first = runtime.mit_torque_limit_stats
        time.sleep(0.006)
        second = runtime.mit_torque_limit_stats

        expected_mask = (1 << 6) | (1 << 13)
        assert first.torque_limited_joint_mask & expected_mask == expected_mask
        assert second.torque_limited_joint_mask & expected_mask == expected_mask
        assert first.torque_limit_activation_count > activation_before
        assert second.torque_limit_activation_count > first.torque_limit_activation_count
        assert len(second.joints) == 14
        for index in (6, 13):
            joint = second.joints[index]
            assert joint.limited
            assert abs(joint.requested_resultant_torque) > TEST_APPLIED_LIMIT
            assert 0.0 < joint.applied_scale < 1.0
            assert abs(joint.applied_resultant_torque) <= TEST_APPLIED_LIMIT + 1e-4

        # Return to zero feedforward immediately, then prove a 100 Hz Python
        # publisher can feed a 500 Hz native worker while all 16 feedback
        # streams remain healthy.
        before = _feedback_stats(robot)
        started = time.perf_counter()
        tick = 0
        submissions = 0
        while True:
            scheduled = started + tick / PUBLISH_HZ
            remaining = scheduled - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
            robot.submit_raw_mit(
                left_positions=left_target,
                right_positions=right_target,
                left_feedforward_torques=zeros,
                right_feedforward_torques=zeros,
            )
            submissions += 1
            elapsed = time.perf_counter() - started
            if elapsed >= HOLD_SECONDS:
                break
            tick += 1

        after = _feedback_stats(robot)
        rates = {
            name: (after[name].update_count - stats.update_count) / elapsed
            for name, stats in before.items()
        }
        health = robot.safety_health
        final = robot.read_cached_state()
        maximum_motion = max(
            *(abs(actual - initial) for actual, initial in zip(
                final.left.arm.positions, left_target
            )),
            *(abs(actual - initial) for actual, initial in zip(
                final.right.arm.positions, right_target
            )),
        )
        assert submissions / elapsed >= PUBLISH_HZ * 0.95
        assert len(rates) == 16
        assert min(rates.values()) >= 450.0
        assert health.state is SafetyState.RUNNING
        assert not health.motor_faults
        assert health.left_transport.send_errors == 0
        assert health.left_transport.receive_errors == 0
        assert health.right_transport.send_errors == 0
        assert health.right_transport.receive_errors == 0
        assert maximum_motion < math.radians(5.0)
        left_j7 = second.joints[6]
        right_j7 = second.joints[13]
        print(
            "\nRuntime MIT limiter: "
            f"activation_delta={second.torque_limit_activation_count - activation_before}, "
            f"mask=0x{second.torque_limited_joint_mask:X}"
        )
        print(
            "J7 resultant/scale/applied: "
            f"left={left_j7.requested_resultant_torque:.5f}/"
            f"{left_j7.applied_scale:.5f}/"
            f"{left_j7.applied_resultant_torque:.5f}, "
            f"right={right_j7.requested_resultant_torque:.5f}/"
            f"{right_j7.applied_scale:.5f}/"
            f"{right_j7.applied_resultant_torque:.5f}"
        )
        print(
            "100 Hz publisher / 500 Hz Runtime feedback: "
            f"min={min(rates.values()):.2f}, "
            f"median={statistics.median(rates.values()):.2f}, "
            f"max={max(rates.values()):.2f} Hz"
        )
        print(f"maximum observed hold motion: {math.degrees(maximum_motion):.3f} deg")
    finally:
        if connected:
            try:
                robot.disable()
                disable_report = robot.last_disable_report
                if disable_report is not None:
                    print(
                        "\ndisable transaction: "
                        f"expected={disable_report.expected_count}, "
                        f"disabled={disable_report.disabled_count}, "
                        f"missing={disable_report.missing_count}, "
                        f"failures={disable_report.failure_count}"
                    )
            finally:
                robot.close()

    assert disable_report is not None
    assert disable_report.success
    assert disable_report.barrier_confirmed
    assert disable_report.expected_count == 16
    assert disable_report.disabled_count == 16
    assert disable_report.missing_count == 0
    assert disable_report.failure_count == 0
    print("disable confirmed: 16/16")
