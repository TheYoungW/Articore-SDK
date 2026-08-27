#!/usr/bin/env python3
"""Soak the complete Yunyi product while holding its current PV pose."""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time

from arx_d_can import ArxDCanDualArm, SafetyState


SAMPLE_HZ = 500.0


def require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    if health.state in {
        SafetyState.FAULT,
        SafetyState.SAFE_STOP,
        SafetyState.SAFE_HOLD,
        SafetyState.DEGRADED,
    }:
        raise RuntimeError(
            f"unsafe Runtime state={health.state.name}: "
            f"{health.last_operation_error or health.fault_reason or health.safety_reason}"
        )


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[max(index, 0)]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=300.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0.0:
        raise ValueError("--seconds must be positive and finite")

    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    enabled = False
    report: dict[str, object] = {
        "duration_requested_s": args.seconds,
        "sample_target_hz": SAMPLE_HZ,
    }
    try:
        robot.connect()
        require_healthy(robot)
        initial = robot.read_state()
        report["initial_left_rad"] = list(initial.left.arm.positions)
        report["initial_right_rad"] = list(initial.right.arm.positions)
        report["initial_left_gripper"] = (
            None if initial.left.gripper is None else initial.left.gripper.opening
        )
        report["initial_right_gripper"] = (
            None if initial.right.gripper is None else initial.right.gripper.opening
        )

        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")
        enabled = True
        robot.set_joint_pv(
            left=initial.left.arm.positions,
            right=initial.right.arm.positions,
            velocity=10.0,
        )

        position_min = list(initial.left.arm.positions + initial.right.arm.positions)
        position_max = list(position_min)
        feedback_ages_ms: list[float] = []
        fps_samples: list[float] = []
        sequence_start: int | None = None
        sequence_end = 0
        sequence_changes = 0
        previous_sequence: int | None = None
        samples = 0
        late_cycles = 0
        maximum_lateness_s = 0.0
        period = 1.0 / SAMPLE_HZ
        started = time.monotonic()
        deadline = started
        next_health = started
        next_progress = started + 30.0

        while True:
            now = time.monotonic()
            if now - started >= args.seconds:
                break
            state = robot.read_state()
            sampled_at_ns = time.monotonic_ns()
            if sequence_start is None:
                sequence_start = state.sequence
            sequence_end = state.sequence
            if previous_sequence != state.sequence:
                sequence_changes += 1
            previous_sequence = state.sequence
            feedback_ages_ms.append(
                max(0.0, (sampled_at_ns - state.timestamp_ns) / 1_000_000.0)
            )
            positions = state.left.arm.positions + state.right.arm.positions
            for index, position in enumerate(positions):
                position_min[index] = min(position_min[index], position)
                position_max[index] = max(position_max[index], position)
            samples += 1

            if now >= next_health:
                require_healthy(robot)
                fps_samples.append(robot.get_fps())
                next_health += 1.0
            if now >= next_progress:
                print(
                    json.dumps(
                        {
                            "elapsed_s": now - started,
                            "state": robot.get_health().state.name,
                            "fps": fps_samples[-1],
                            "maximum_feedback_age_ms": max(feedback_ages_ms),
                            "samples": samples,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                next_progress += 30.0

            deadline += period
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                late_cycles += 1
                maximum_lateness_s = max(maximum_lateness_s, -remaining)

        elapsed = time.monotonic() - started
        final = robot.read_state()
        require_healthy(robot)
        report.update(
            {
                "elapsed_s": elapsed,
                "samples": samples,
                "sample_hz": samples / elapsed,
                "late_cycles": late_cycles,
                "maximum_lateness_ms": maximum_lateness_s * 1000.0,
                "sequence_start": sequence_start,
                "sequence_end": sequence_end,
                "sequence_delta": sequence_end - (sequence_start or 0),
                "sequence_changes_observed": sequence_changes,
                "worst_motor_feedback_age_ms": {
                    "mean": statistics.fmean(feedback_ages_ms),
                    "p99": percentile(feedback_ages_ms, 0.99),
                    "maximum": max(feedback_ages_ms),
                },
                "can_feedback_fps": {
                    "mean": statistics.fmean(fps_samples),
                    "minimum": min(fps_samples),
                    "maximum": max(fps_samples),
                },
                "joint_position_peak_to_peak_rad": [
                    high - low for low, high in zip(position_min, position_max)
                ],
                "final_health": robot.get_health().state.name,
                "final_left_enabled": list(final.left.arm.enabled),
                "final_right_enabled": list(final.right.arm.enabled),
                "final_left_gripper_enabled": (
                    None if final.left.gripper is None else final.left.gripper.enabled
                ),
                "final_right_gripper_enabled": (
                    None if final.right.gripper is None else final.right.gripper.enabled
                ),
            }
        )
        print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    finally:
        if enabled:
            try:
                robot.disable()
            except Exception:
                pass
        robot.disconnect()


if __name__ == "__main__":
    main()
