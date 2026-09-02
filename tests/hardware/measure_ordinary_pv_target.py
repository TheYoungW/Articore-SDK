#!/usr/bin/env python3
"""真机诊断：记录一次完整双臂普通 PV 终点命令的14关节响应。"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

from arx_d_can import ArxDCanDualArm, SafetyState
from arx_d_can.examples.common import joint_degrees, speed_percent


JOINT_NAMES = tuple(
    [f"left/J{index}" for index in range(1, 8)]
    + [f"right/J{index}" for index in range(1, 8)]
)
UNSAFE_STATES = {
    SafetyState.FAULT,
    SafetyState.SAFE_HOLD,
    SafetyState.SAFE_STOP,
    SafetyState.DEGRADED,
}


def _require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    invalid = tuple(
        item.role
        for item in health.motor_feedback
        if not (
            item.has_feedback
            and item.fresh
            and item.has_state
            and item.values_finite
        )
    )
    if health.state in UNSAFE_STATES or invalid or health.feedback_issue_count:
        detail = (
            health.last_operation_error
            or health.safety_reason
            or health.fault_reason
            or f"invalid feedback: {invalid}"
        )
        raise RuntimeError(f"unsafe Runtime state={health.state.name}: {detail}")


def _metrics(
    times: list[float],
    positions: list[tuple[float, ...]],
    velocities: list[tuple[float, ...]],
    start: tuple[float, ...],
    target: tuple[float, ...],
    tolerance: float,
) -> list[dict[str, float | str | None]]:
    result = []
    for joint, name in enumerate(JOINT_NAMES):
        q = [row[joint] for row in positions]
        dq = [row[joint] for row in velocities]
        peak_index = max(range(len(dq)), key=lambda index: abs(dq[index]))
        first_tolerance = next(
            (
                elapsed
                for elapsed, value in zip(times, q, strict=True)
                if abs(value - target[joint]) <= tolerance
            ),
            None,
        )
        direction = 1.0 if target[joint] >= start[joint] else -1.0
        overshoot = max(
            0.0,
            max(direction * (value - target[joint]) for value in q),
        )
        result.append(
            {
                "joint": name,
                "start_deg": math.degrees(start[joint]),
                "target_deg": math.degrees(target[joint]),
                "travel_deg": math.degrees(target[joint] - start[joint]),
                "peak_abs_velocity_deg_s": math.degrees(abs(dq[peak_index])),
                "peak_velocity_time_s": times[peak_index],
                "first_within_0_5deg_s": first_tolerance,
                "overshoot_deg": math.degrees(overshoot),
                "final_error_deg": math.degrees(q[-1] - target[joint]),
                "position_peak_to_peak_deg": math.degrees(max(q) - min(q)),
                "velocity_rms_deg_s": math.degrees(
                    math.sqrt(statistics.fmean(value * value for value in dq))
                ),
            }
        )
    return result


def run(args: argparse.Namespace) -> dict[str, object]:
    left_target = joint_degrees(args.left)
    right_target = joint_degrees(args.right)
    target = left_target + right_target
    period = 1.0 / args.sample_hz
    tolerance = math.radians(args.position_tolerance_deg)
    velocity_tolerance = math.radians(args.velocity_tolerance_deg_s)
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    enabled = False
    cleanup_errors: list[str] = []
    times: list[float] = []
    sequences: list[int] = []
    positions: list[tuple[float, ...]] = []
    velocities: list[tuple[float, ...]] = []
    try:
        robot.connect()
        _require_healthy(robot)
        limits = tuple(robot.get_joint_limits().values())
        for index, (value, limit) in enumerate(zip(target, limits, strict=True)):
            if not limit.min_angle_rad <= value <= limit.max_angle_rad:
                raise ValueError(f"{JOINT_NAMES[index]} target is outside product limits")
        initial = robot.read_state()
        start = tuple(initial.left.arm.positions) + tuple(initial.right.arm.positions)
        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")
        enabled = True
        _require_healthy(robot)
        submitted = time.perf_counter()
        robot.set_joint_pv(
            left=left_target,
            right=right_target,
            velocity=args.velocity,
        )
        next_tick = submitted
        stable_since: float | None = None
        settled_elapsed: float | None = None
        timed_out = False
        missed_periods = 0
        while True:
            remaining = next_tick - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
            captured = time.perf_counter()
            if captured - next_tick >= period:
                missed_periods += 1
            state = robot.read_state()
            elapsed = captured - submitted
            q = tuple(state.left.arm.positions) + tuple(state.right.arm.positions)
            dq = tuple(state.left.arm.velocities) + tuple(state.right.arm.velocities)
            times.append(elapsed)
            sequences.append(int(state.sequence))
            positions.append(q)
            velocities.append(dq)
            if len(times) % 50 == 0:
                _require_healthy(robot)
            if settled_elapsed is None:
                if max(
                    abs(a - b) for a, b in zip(q, target, strict=True)
                ) <= tolerance and max(
                    abs(value) for value in dq
                ) <= velocity_tolerance:
                    stable_since = captured if stable_since is None else stable_since
                    if captured - stable_since >= args.stable_seconds:
                        settled_elapsed = elapsed
                else:
                    stable_since = None
            if settled_elapsed is not None and elapsed - settled_elapsed >= args.hold_seconds:
                break
            if elapsed >= args.timeout:
                timed_out = True
                break
            next_tick += period

        span = times[-1] - times[0]
        metrics = _metrics(times, positions, velocities, start, target, tolerance)
        return {
            "motor_speed_percent": args.velocity,
            "sample_target_hz": args.sample_hz,
            "samples": len(times),
            "sample_hz": (len(times) - 1) / span,
            "unique_feedback_hz": (len(set(sequences)) - 1) / span,
            "missed_periods": missed_periods,
            "settling_time_s": settled_elapsed,
            "timed_out": timed_out,
            "hold_seconds": args.hold_seconds,
            "start_deg": [math.degrees(value) for value in start],
            "target_deg": [math.degrees(value) for value in target],
            "joints": metrics,
            "fastest_joint": max(
                metrics, key=lambda item: float(item["peak_abs_velocity_deg_s"])
            )["joint"],
            "health": robot.get_health().state.name,
            "_samples": (times, sequences, positions, velocities),
        }
    finally:
        if enabled:
            try:
                robot.disable()
            except Exception as error:
                cleanup_errors.append(f"disable: {error}")
        try:
            robot.disconnect()
        except Exception as error:
            cleanup_errors.append(f"disconnect: {error}")
        if cleanup_errors:
            print("清理错误：" + "; ".join(cleanup_errors), flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", default="0,0,0,90,0,0,0")
    parser.add_argument("--right", default="0,0,0,90,0,0,0")
    parser.add_argument("--velocity", type=speed_percent, default=50.0)
    parser.add_argument("--sample-hz", type=float, default=500.0)
    parser.add_argument("--position-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--velocity-tolerance-deg-s", type=float, default=1.0)
    parser.add_argument("--stable-seconds", type=float, default=0.25)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/ordinary_pv_target/ordinary_pv_target.json"),
    )
    parser.add_argument("--yes", action="store_true")
    return parser


def main(args: argparse.Namespace) -> None:
    if not args.yes:
        input(
            "将使能双臂并执行指定普通 PV 目标；确认工作空间安全、急停可用后按回车..."
        )
    report = run(args)
    times, sequences, positions, velocities = report.pop("_samples")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["elapsed_s", "sequence"]
            + [f"{name}_position_rad" for name in JOINT_NAMES]
            + [f"{name}_velocity_rad_s" for name in JOINT_NAMES]
        )
        for elapsed, sequence, q, dq in zip(
            times, sequences, positions, velocities, strict=True
        ):
            writer.writerow((elapsed, sequence, *q, *dq))
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    print(f"JSON: {args.output}", flush=True)
    print(f"CSV: {csv_path}", flush=True)


if __name__ == "__main__":
    main(build_parser().parse_args())
