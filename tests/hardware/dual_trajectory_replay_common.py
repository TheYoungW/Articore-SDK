#!/usr/bin/env python3
"""Shared implementation for developer-only real-hardware trajectory replay."""
from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path
import sys
from typing import Literal

from arx_d_can import ArxDCanDualArm, SafetyState
from arx_d_can.examples.control import example_14_replay_trajectory as replay_example
from arx_d_can.service_tools.dual_trajectory_recording import (
    DualArmTrajectorySample,
    load_trajectory,
    replay,
)


Mode = Literal["pv", "mit"]
PROJECT_ROOT = Path(__file__).resolve().parents[2]
TRAJECTORY_PATH = (
    PROJECT_ROOT / "trajectories" / "dual_clipped_random_100.json"
)
JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 8))
EXPECTED_SAMPLE_COUNT = 200
DISCONTINUITY_RATE_RAD_S = 10.0
REPLAY_VELOCITY_PERCENT = 50.0
START_TIMEOUT_S = 30.0
POSITION_TOLERANCE_RAD = math.radians(1.0)
VELOCITY_TOLERANCE_RAD_S = math.radians(2.0)
HARDWARE_TIME_SCALE = 3.0


def _segment_peak_rate(
    timestamps: list[float],
    samples: list[DualArmTrajectorySample],
    index: int,
) -> float:
    duration = timestamps[index] - timestamps[index - 1]
    previous = samples[index - 1]
    current = samples[index]
    return max(
        *(
            abs(end - start) / duration
            for start, end in zip(
                previous.left_positions,
                current.left_positions,
            )
        ),
        *(
            abs(end - start) / duration
            for start, end in zip(
                previous.right_positions,
                current.right_positions,
            )
        ),
    )


def _split_discontinuities(
    timestamps: list[float],
    samples: list[DualArmTrajectorySample],
) -> tuple[
    list[tuple[list[float], list[DualArmTrajectorySample]]],
    list[tuple[int, float]],
]:
    boundaries = [0]
    discontinuities: list[tuple[int, float]] = []
    for index in range(1, len(samples)):
        peak_rate = _segment_peak_rate(timestamps, samples, index)
        if peak_rate > DISCONTINUITY_RATE_RAD_S:
            boundaries.append(index)
            discontinuities.append((index, peak_rate))
    boundaries.append(len(samples))
    segments = [
        (timestamps[start:end], samples[start:end])
        for start, end in zip(boundaries, boundaries[1:])
    ]
    return segments, discontinuities


def _load_test_trajectory() -> tuple[
    list[tuple[list[float], list[DualArmTrajectorySample]]],
    list[tuple[int, float]],
]:
    if not TRAJECTORY_PATH.is_file():
        raise FileNotFoundError(f"trajectory does not exist: {TRAJECTORY_PATH}")
    timestamps, samples = load_trajectory(
        TRAJECTORY_PATH,
        expected_left_joint_names=JOINT_NAMES,
        expected_right_joint_names=JOINT_NAMES,
    )
    # The product trajectory ABI covers the fixed 14 arm joints. This
    # diagnostic intentionally excludes recorded gripper motion instead of
    # reintroducing a Python real-time gripper replay loop.
    samples = [
        replace(sample, left_gripper=None, right_gripper=None)
        for sample in samples
    ]
    if len(samples) != EXPECTED_SAMPLE_COUNT:
        raise RuntimeError(
            f"expected {EXPECTED_SAMPLE_COUNT} trajectory samples, got {len(samples)}"
        )
    segments, discontinuities = _split_discontinuities(timestamps, samples)
    if [len(segment_samples) for _, segment_samples in segments] != [100, 100]:
        raise RuntimeError(
            "expected the appended trajectory to split into two safe 100-point "
            f"segments, got {[len(values) for _, values in segments]}"
        )
    scaled_segments = [
        (
            [
                segment_timestamps[0]
                + (value - segment_timestamps[0]) * HARDWARE_TIME_SCALE
                for value in segment_timestamps
            ],
            segment_samples,
        )
        for segment_timestamps, segment_samples in segments
    ]
    return scaled_segments, discontinuities


def _require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    if health.state in {
        SafetyState.DEGRADED,
        SafetyState.SAFE_HOLD,
        SafetyState.SAFE_STOP,
        SafetyState.FAULT,
        SafetyState.PARTIALLY_ENABLED,
    }:
        raise RuntimeError(
            f"unsafe Runtime state={health.state.name}: "
            f"{health.last_operation_error or health.fault_reason or health.safety_reason}"
        )


def _run_hardware(
    mode: Mode,
    segments: list[tuple[list[float], list[DualArmTrajectorySample]]],
) -> None:
    robot = ArxDCanDualArm(control_mode=mode, with_grippers=True)
    run_error: BaseException | None = None
    cleanup_errors: list[str] = []
    try:
        robot.connect()
        _require_healthy(robot)
        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")

        for number, (timestamps, samples) in enumerate(segments, start=1):
            print(
                f"[{mode.upper()}] moving to segment {number} start "
                f"({len(samples)} samples)",
                flush=True,
            )
            replay_example._move_to_start(
                robot,
                samples[0],
                velocity=REPLAY_VELOCITY_PERCENT,
                timeout=START_TIMEOUT_S,
                position_tolerance=POSITION_TOLERANCE_RAD,
                velocity_tolerance=VELOCITY_TOLERANCE_RAD_S,
            )
            _require_healthy(robot)
            print(f"[{mode.upper()}] replaying segment {number}", flush=True)
            replay(
                robot,
                timestamps=timestamps,
                samples=samples,
                velocity=REPLAY_VELOCITY_PERCENT,
            )
            _require_healthy(robot)
        print(f"[{mode.upper()}] trajectory replay completed", flush=True)
    except BaseException as error:
        run_error = error
    finally:
        try:
            if robot.connected:
                robot.disable()
                print("whole product disable confirmed", flush=True)
        except Exception as error:
            cleanup_errors.append(f"disable failed: {error}")
        try:
            robot.disconnect()
            print("Runtime disconnected", flush=True)
        except Exception as error:
            cleanup_errors.append(f"disconnect failed: {error}")

    if run_error is not None:
        for error in cleanup_errors:
            print(error, file=sys.stderr, flush=True)
        raise run_error.with_traceback(run_error.__traceback__)
    if cleanup_errors:
        raise RuntimeError("; ".join(cleanup_errors))


def main(mode: Mode) -> int:
    parser = argparse.ArgumentParser(
        description=(
            f"Developer-only {mode.upper()} real-hardware replay of "
            f"{TRAJECTORY_PATH.name}."
        )
    )
    parser.add_argument(
        "--i-understand-robot-will-move",
        action="store_true",
        help="required acknowledgement before connecting and enabling the robot",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate and summarize the trajectory without opening the Runtime",
    )
    args = parser.parse_args()

    segments, discontinuities = _load_test_trajectory()
    duration = sum(values[-1] - values[0] for values, _ in segments)
    print(f"trajectory: {TRAJECTORY_PATH}")
    print(f"mode: {mode.upper()}")
    print(f"segments: {[len(samples) for _, samples in segments]}")
    print(f"replay duration: {duration:.3f}s plus safe segment transitions")
    print(f"hardware timestamp scale: {HARDWARE_TIME_SCALE:g}x")
    for index, peak_rate in discontinuities:
        print(
            f"split before sample {index}: implied peak rate "
            f"{peak_rate:.3f} rad/s"
        )

    if args.dry_run:
        print("dry-run complete; Runtime was not opened")
        return 0
    if not args.i_understand_robot_will_move:
        parser.error("--i-understand-robot-will-move is required")

    _run_hardware(mode, segments)
    return 0


__all__ = ["main"]
