#!/usr/bin/env python3
"""Real Yunyi dual-arm product-pose and 500 Hz cache-read validation."""
from __future__ import annotations

import json
import argparse
import math
import statistics
import time

from arx_d_can import ArxDCanDualArm, SafetyState


RATE_HZ = 500.0
SPEED_PERCENT = 10.0
POSITION_TOLERANCE = 0.05
SOAK_SECONDS = 60.0
SETTLE_SECONDS = 2.0


def target(joint4_degrees: float) -> tuple[float, ...]:
    values = [0.0] * 7
    values[3] = math.radians(joint4_degrees)
    return tuple(values)


def set_joint_positions(
    robot: ArxDCanDualArm,
    mode: str,
    left: tuple[float, ...],
    right: tuple[float, ...],
) -> None:
    if mode == "mit":
        robot.set_joint_mit(left=left, right=right)
    else:
        robot.set_joint_pv(left=left, right=right, velocity=SPEED_PERCENT)


def require_motion_health(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    if health.state in {
        SafetyState.FAULT,
        SafetyState.SAFE_STOP,
        SafetyState.SAFE_HOLD,
    }:
        raise RuntimeError(
            f"unsafe Runtime state={health.state.name}: "
            f"{health.fault_reason or health.safety_reason}"
        )


def wait_for_targets(
    robot: ArxDCanDualArm,
    left_positions: tuple[float, ...],
    right_positions: tuple[float, ...],
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        require_motion_health(robot)
        state = robot.read_state()
        errors = [
            *(abs(actual - expected) for actual, expected in zip(
                state.left.arm.positions, left_positions
            )),
            *(abs(actual - expected) for actual, expected in zip(
                state.right.arm.positions, right_positions
            )),
        ]
        if max(errors) <= POSITION_TOLERANCE:
            return
        time.sleep(0.02)
    raise TimeoutError(f"joint target was not reached within {timeout:.1f}s")


def sample_while_moving(
    robot: ArxDCanDualArm,
    moving_side: str,
    positions: tuple[float, ...],
    timeout: float,
) -> dict[str, object]:
    period = 1.0 / RATE_HZ
    started = time.perf_counter()
    deadline = started
    call_times: list[float] = []
    sequences = {"left": [], "right": []}
    poses = {"left": [], "right": []}
    joint4 = {"left": [], "right": []}
    late_cycles = 0

    while time.perf_counter() - started < timeout:
        cycle_started = time.perf_counter()
        left = robot.get_pose_sample("left")
        right = robot.get_pose_sample("right")
        cycle_finished = time.perf_counter()
        call_times.append(cycle_finished - cycle_started)
        for name, sample in (("left", left), ("right", right)):
            sequences[name].append(sample.sequence)
            poses[name].append(sample.values)

        state = robot.read_state()
        joint4["left"].append(state.left.arm.positions[3])
        joint4["right"].append(state.right.arm.positions[3])
        require_motion_health(robot)
        moving_arm = state.left.arm if moving_side == "left" else state.right.arm
        if abs(moving_arm.positions[3] - positions[3]) <= POSITION_TOLERANCE:
            break

        deadline += period
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        else:
            late_cycles += 1

    elapsed = time.perf_counter() - started
    result: dict[str, object] = {
        "samples": len(call_times),
        "elapsed_s": elapsed,
        "pair_read_hz": len(call_times) / elapsed,
        "late_cycles": late_cycles,
        "mean_pair_call_us": statistics.fmean(call_times) * 1e6,
        "max_pair_call_us": max(call_times) * 1e6,
    }
    for name in ("left", "right"):
        first = poses[name][0]
        last = poses[name][-1]
        result[name] = {
            "sequence_start": sequences[name][0],
            "sequence_end": sequences[name][-1],
            "sequence_updates": len(set(sequences[name])),
            "joint4_start_deg": math.degrees(joint4[name][0]),
            "joint4_end_deg": math.degrees(joint4[name][-1]),
            "pose_delta_norm": math.sqrt(
                sum((end - begin) ** 2 for begin, end in zip(first, last))
            ),
            "pose_start": first,
            "pose_end": last,
        }
    return result


def sample_fixed_rate(robot: ArxDCanDualArm, duration: float) -> dict[str, object]:
    period = 1.0 / RATE_HZ
    started = time.perf_counter()
    deadline = started
    samples = 0
    late_cycles = 0
    call_times: list[float] = []
    first_sequence: dict[str, int] = {}
    last_sequence: dict[str, int] = {}
    sequence_updates = {"left": 0, "right": 0}
    previous_sequence: dict[str, int] = {}
    while time.perf_counter() - started < duration:
        cycle_started = time.perf_counter()
        values = {
            "left": robot.get_pose_sample("left"),
            "right": robot.get_pose_sample("right"),
        }
        call_times.append(time.perf_counter() - cycle_started)
        for name, sample in values.items():
            first_sequence.setdefault(name, sample.sequence)
            if previous_sequence.get(name) != sample.sequence:
                sequence_updates[name] += 1
            previous_sequence[name] = sample.sequence
            last_sequence[name] = sample.sequence
        samples += 1
        if samples % 50 == 0:
            require_motion_health(robot)
        deadline += period
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        else:
            late_cycles += 1
    elapsed = time.perf_counter() - started
    return {
        "samples": samples,
        "elapsed_s": elapsed,
        "pair_read_hz": samples / elapsed,
        "late_cycles": late_cycles,
        "mean_pair_call_us": statistics.fmean(call_times) * 1e6,
        "max_pair_call_us": max(call_times) * 1e6,
        "left_sequence_start": first_sequence["left"],
        "left_sequence_end": last_sequence["left"],
        "left_sequence_updates": sequence_updates["left"],
        "right_sequence_start": first_sequence["right"],
        "right_sequence_end": last_sequence["right"],
        "right_sequence_updates": sequence_updates["right"],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("mit", "pv"), default="mit")
    parser.add_argument("--soak-seconds", type=float, default=SOAK_SECONDS)
    args = parser.parse_args()
    robot = ArxDCanDualArm(control_mode=args.mode, with_grippers=False)
    report: dict[str, object] = {}
    try:
        robot.connect()
        report["control_mode"] = args.mode.upper()
        report["connected_health"] = robot.get_health().state.name
        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")

        initial = robot.read_state()
        # The left J4 on this rig reaches a repeatable mechanical/calibration
        # boundary near +0.146 rad rather than logical zero. Keep the complete
        # left arm at its current pose and exercise only the verified right J4.
        left_hold = tuple(initial.left.arm.positions)
        zero = target(0.0)
        set_joint_positions(robot, args.mode, left_hold, zero)
        wait_for_targets(robot, left_hold, zero, timeout=12.0)

        ninety = target(90.0)
        set_joint_positions(robot, args.mode, left_hold, ninety)
        report["motion_0_to_90"] = sample_while_moving(
            robot, "right", ninety, timeout=10.0
        )
        wait_for_targets(robot, left_hold, ninety, timeout=3.0)
        time.sleep(SETTLE_SECONDS)
        settled_ninety = robot.read_state().right.arm.positions[3]
        report["settled_90"] = {
            "joint4_deg": math.degrees(settled_ninety),
            "error_deg": math.degrees(settled_ninety - ninety[3]),
            "settle_seconds": SETTLE_SECONDS,
        }

        set_joint_positions(robot, args.mode, left_hold, zero)
        wait_for_targets(robot, left_hold, zero, timeout=12.0)
        time.sleep(SETTLE_SECONDS)
        settled_zero = robot.read_state().right.arm.positions[3]
        report["settled_zero"] = {
            "joint4_deg": math.degrees(settled_zero),
            "error_deg": math.degrees(settled_zero),
            "settle_seconds": SETTLE_SECONDS,
        }
        if args.soak_seconds > 0.0:
            report["hold_soak"] = sample_fixed_rate(robot, args.soak_seconds)
        report["final_health"] = robot.get_health().state.name
        report["final_left_pose"] = robot.get_pose("left")
        report["final_right_pose"] = robot.get_pose("right")
        if not robot.disable():
            raise RuntimeError("whole-product disable was not confirmed")
        report["disabled"] = not robot.enabled
    finally:
        robot.disconnect()
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
