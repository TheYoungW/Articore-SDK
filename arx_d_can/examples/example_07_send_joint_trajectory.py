#!/usr/bin/env python3
"""Example 07: execute a time-parameterized joint trajectory."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import time

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import (
    add_connection_arguments,
    arm_kwargs,
    parse_joint_positions_degrees,
)
from arx_d_can.trajectory import JointPositionPoint, plan_joint_position_trajectory


@dataclass(frozen=True, slots=True)
class ExecutionStats:
    commands: int
    elapsed_s: float
    achieved_hz: float
    missed_deadlines: int
    max_lateness_ms: float


def _format_degrees(positions) -> str:
    return "[" + ", ".join(f"{math.degrees(value):+.2f}°" for value in positions) + "]"


def execute_trajectory(
    arm,
    points: list[JointPositionPoint],
    *,
    report_seconds: float = 1.0,
) -> ExecutionStats:
    if len(points) < 2:
        raise ValueError("trajectory must contain at least two points")

    nominal_period = points[1].time - points[0].time
    started = time.perf_counter()
    next_report = started
    missed_deadlines = 0
    max_lateness = 0.0

    for point in points:
        deadline = started + point.time
        remaining = deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        sent_at = time.perf_counter()
        lateness = max(0.0, sent_at - deadline)
        max_lateness = max(max_lateness, lateness)
        if lateness > nominal_period:
            missed_deadlines += 1

        arm.send_joint_positions(point.positions)
        if sent_at >= next_report:
            state = arm.read_state(request_feedback=False)
            print(
                f"trajectory {point.time:.3f}/{points[-1].time:.3f}s "
                f"actual={_format_degrees(state.arm.positions)}",
                flush=True,
            )
            next_report = sent_at + max(0.1, report_seconds)

    elapsed = time.perf_counter() - started
    intervals = len(points) - 1
    return ExecutionStats(
        commands=len(points),
        elapsed_s=elapsed,
        achieved_hz=intervals / elapsed if elapsed > 0.0 else 0.0,
        missed_deadlines=missed_deadlines,
        max_lateness_ms=max_lateness * 1000.0,
    )


def hold_target(arm, target, *, seconds: float, hz: float) -> None:
    period = 1.0 / hz
    started = time.perf_counter()
    cycle = 0
    while time.perf_counter() - started < seconds:
        arm.send_joint_positions(target)
        cycle += 1
        remaining = started + cycle * period - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)


def print_stats(label: str, stats: ExecutionStats) -> None:
    print(
        f"{label}: commands={stats.commands} elapsed={stats.elapsed_s:.3f}s "
        f"achieved={stats.achieved_hz:.1f}Hz missed={stats.missed_deadlines} "
        f"max_lateness={stats.max_lateness_ms:.3f}ms"
    )


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(**arm_kwargs(args))
    target = parse_joint_positions_degrees(
        args.positions,
        expected_count=len(arm.joint_names),
    )
    zero_position = (0.0,) * len(arm.joint_names)
    try:
        arm.connect()
        arm.configure()
        initial = arm.read_state(request_feedback=True).arm.positions
        print(f"initial={_format_degrees(initial)} target={_format_degrees(target)}")
        arm.enable()

        outbound = plan_joint_position_trajectory(
            initial,
            target,
            duration=args.duration,
            hz=args.hz,
            profile=args.profile,
        )
        print_stats("outbound", execute_trajectory(arm, outbound))
        hold_target(arm, target, seconds=args.hold_seconds, hz=args.hz)
        reached = arm.read_state(request_feedback=True)
        error_deg = max(
            abs(math.degrees(actual - expected))
            for actual, expected in zip(reached.arm.positions, target)
        )
        print(
            f"reached={_format_degrees(reached.arm.positions)} "
            f"max_error={error_deg:.3f}°"
        )

        if args.return_zero:
            inbound = plan_joint_position_trajectory(
                reached.arm.positions,
                zero_position,
                duration=args.return_seconds,
                hz=args.hz,
                profile=args.profile,
            )
            print_stats("return", execute_trajectory(arm, inbound))
            hold_target(
                arm,
                zero_position,
                seconds=args.zero_hold_seconds,
                hz=args.hz,
            )
            returned = arm.read_state(request_feedback=True)
            print(f"returned={_format_degrees(returned.arm.positions)}")
    finally:
        arm.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Execute a smooth ARX-D-CAN joint trajectory at 500 Hz by default."
    )
    parser.add_argument("positions", help="Comma-separated joint targets in degrees")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--hz", type=float, default=500.0)
    parser.add_argument("--profile", choices=("min_jerk", "linear"), default="min_jerk")
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--return-zero", action="store_true")
    parser.add_argument("--return-seconds", type=float, default=6.0)
    parser.add_argument("--zero-hold-seconds", type=float, default=2.0)
    add_connection_arguments(parser)
    main(parser.parse_args())
