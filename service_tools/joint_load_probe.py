#!/usr/bin/env python3
"""Example 08: move one joint through a slow trajectory and log tracking load."""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import time

try:
    from arx_d_can.examples.common import (
        add_connection_arguments,
        interpolate_joint_positions,
        make_arm,
        parse_joint_positions,
    )
except ModuleNotFoundError:
    from teleop.adapters.arm.arx_d_can.examples.common import (
        add_connection_arguments,
        interpolate_joint_positions,
        make_arm,
        parse_joint_positions,
    )


DEFAULT_PREPOSE = "0,-1.0471975512,-1.0471975512,0,0,0"


@dataclass(slots=True, frozen=True)
class Sample:
    elapsed_s: float
    target_rad: float
    actual_rad: float
    error_rad: float
    velocity_rad_s: float
    torque: float


def sine_target(*, center: float, amplitude: float, elapsed: float, period: float) -> float:
    if period <= 0.0:
        raise ValueError("period must be positive")
    return center + amplitude * math.sin(2.0 * math.pi * elapsed / period)


def parse_optional_prepose(text: str) -> tuple[float, ...] | None:
    if not text.strip():
        return None
    return parse_joint_positions(text)


def return_zero_target(*, disabled: bool) -> tuple[float, ...] | None:
    if disabled:
        return None
    return (0.0,) * 6


def next_tick_delay(*, next_tick: float, period: float, now: float) -> tuple[float, float]:
    next_tick += period
    return next_tick, max(0.0, next_tick - now)


def summarize_samples(samples: list[Sample]) -> dict[str, float]:
    if not samples:
        return {
            "samples": 0,
            "max_abs_error_rad": 0.0,
            "rms_error_rad": 0.0,
            "peak_abs_torque": 0.0,
            "mean_abs_torque": 0.0,
            "peak_abs_velocity_rad_s": 0.0,
        }
    return {
        "samples": len(samples),
        "max_abs_error_rad": max(abs(sample.error_rad) for sample in samples),
        "rms_error_rad": math.sqrt(
            sum(sample.error_rad * sample.error_rad for sample in samples) / len(samples)
        ),
        "peak_abs_torque": max(abs(sample.torque) for sample in samples),
        "mean_abs_torque": sum(abs(sample.torque) for sample in samples) / len(samples),
        "peak_abs_velocity_rad_s": max(abs(sample.velocity_rad_s) for sample in samples),
    }


def write_csv(path: Path, samples: list[Sample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "elapsed_s",
            "target_rad",
            "actual_rad",
            "error_rad",
            "velocity_rad_s",
            "torque",
        ])
        for sample in samples:
            writer.writerow([
                f"{sample.elapsed_s:.6f}",
                f"{sample.target_rad:.9f}",
                f"{sample.actual_rad:.9f}",
                f"{sample.error_rad:.9f}",
                f"{sample.velocity_rad_s:.9f}",
                f"{sample.torque:.9f}",
            ])


def run_probe(args: argparse.Namespace) -> list[Sample]:
    joint_index = args.joint - 1
    if not 0 <= joint_index < 6:
        raise SystemExit("--joint must be in 1..6")
    amplitude = math.radians(args.amplitude_deg)
    duration = max(0.0, args.cycles * args.period)
    period = 1.0 / max(1.0, args.hz)
    samples: list[Sample] = []
    arm = make_arm(args)
    try:
        arm.connect()
        arm.configure()
        arm.enable()
        initial_state = arm.read_state(request_feedback=True)
        prepose = parse_optional_prepose(args.prepose)
        base = list(initial_state.arm.positions if prepose is None else prepose)
        center = base[joint_index] if args.center_rad is None else float(args.center_rad)
        print(
            f"joint{args.joint} load probe: center={center:+.6f} rad, "
            f"amplitude={amplitude:+.6f} rad, period={args.period:.3f}s, cycles={args.cycles}"
        )
        if prepose is not None:
            print("moving to prepose:", " ".join(f"{value:+.6f}" for value in prepose))
            interpolate_joint_positions(
                arm,
                initial_state.arm.positions,
                prepose,
                seconds=args.prepose_seconds,
                hz=args.hz,
            )

        if args.move_seconds > 0.0:
            current_state = arm.read_state(request_feedback=True)
            start = list(current_state.arm.positions)
            start[joint_index] = center
            interpolate_joint_positions(arm, current_state.arm.positions, start, seconds=args.move_seconds, hz=args.hz)

        start_time = time.monotonic()
        next_tick = start_time
        while True:
            elapsed = time.monotonic() - start_time
            if elapsed > duration:
                break
            target = sine_target(center=center, amplitude=amplitude, elapsed=elapsed, period=args.period)
            command = list(base)
            command[joint_index] = target
            arm.send_joint_positions(command)
            state = arm.read_state(request_feedback=True)
            actual = float(state.arm.positions[joint_index])
            velocity = float(state.arm.velocities[joint_index]) if state.arm.velocities else 0.0
            torque = float(state.arm.torques[joint_index]) if state.arm.torques else 0.0
            samples.append(Sample(elapsed, target, actual, actual - target, velocity, torque))
            next_tick, delay = next_tick_delay(
                next_tick=next_tick,
                period=period,
                now=time.monotonic(),
            )
            if delay > 0.0:
                time.sleep(delay)

        if args.return_center:
            current = arm.read_state(request_feedback=True).arm.positions
            target = list(current)
            target[joint_index] = center
            interpolate_joint_positions(arm, current, target, seconds=args.return_seconds, hz=args.hz)

        zero_target = return_zero_target(disabled=args.no_return_zero)
        if zero_target is not None:
            current = arm.read_state(request_feedback=True).arm.positions
            print("returning all arm joints to zero")
            interpolate_joint_positions(
                arm,
                current,
                zero_target,
                seconds=args.return_zero_seconds,
                hz=args.hz,
            )
    finally:
        arm.close()
    return samples


def main(args: argparse.Namespace) -> None:
    samples = run_probe(args)
    summary = summarize_samples(samples)
    if args.csv:
        path = Path(args.csv)
        write_csv(path, samples)
        print(f"csv: {path}")
    print("summary:")
    for key, value in summary.items():
        if key == "samples":
            print(f"  {key}: {int(value)}")
        else:
            print(f"  {key}: {value:.6f}")
    print("tip: compare peak_abs_torque and max_abs_error_rad between PV parameter sets.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Probe one ARX-D-CAN joint load with a slow sine trajectory.")
    parser.add_argument("--joint", type=int, default=4, help="1-based joint index to move")
    parser.add_argument("--amplitude-deg", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=4.0, help="Sine period in seconds")
    parser.add_argument("--cycles", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--center-rad", type=float, default=None, help="Default: current joint position")
    parser.add_argument(
        "--prepose",
        default=DEFAULT_PREPOSE,
        help="Optional six-joint pose before probing; empty string disables",
    )
    parser.add_argument("--prepose-seconds", type=float, default=4.0)
    parser.add_argument("--move-seconds", type=float, default=2.0, help="Move to center before probing")
    parser.add_argument("--return-center", action="store_true")
    parser.add_argument("--return-seconds", type=float, default=2.0)
    parser.add_argument("--no-return-zero", action="store_true")
    parser.add_argument("--return-zero-seconds", type=float, default=6.0)
    parser.add_argument("--csv", default="", help="Optional CSV output path")
    add_connection_arguments(parser)
    main(parser.parse_args())
