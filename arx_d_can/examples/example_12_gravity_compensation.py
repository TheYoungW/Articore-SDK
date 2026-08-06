#!/usr/bin/env python3
"""Example 12: run safe MIT gravity compensation with zero position stiffness."""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanArm, GravityCompensationMode
from arx_d_can.examples.common import add_connection_arguments


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return parsed


def non_negative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return parsed


def non_negative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return parsed


def parse_joint_values(
    text: str,
    *,
    expected_count: int,
    name: str,
    allow_negative: bool = False,
) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != expected_count:
        raise ValueError(
            f"expected {expected_count} comma-separated {name} values, got {len(values)}"
        )
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{name} values must be finite")
    if not allow_negative and any(value < 0.0 for value in values):
        raise ValueError(f"{name} values must be finite and non-negative")
    return values


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        transport=getattr(args, "transport", None),
        control_mode="mit",
    )
    joint_scales = (
        None
        if args.joint_scales is None
        else parse_joint_values(
            args.joint_scales,
            expected_count=len(arm.joint_names),
            name="joint scale",
            allow_negative=True,
        )
    )
    mode = GravityCompensationMode(
        arm,
        hz=args.hz,
        transition_seconds=args.transition_seconds,
        settle_seconds=args.settle_seconds,
        gravity_scale=args.gravity_scale,
        joint_scales=joint_scales,
        damping=args.damping,
    )
    last_report = -1.0
    report_period = 1.0 / args.report_hz

    def report(sample) -> None:
        nonlocal last_report
        if sample.elapsed_s - last_report < report_period:
            return
        last_report = sample.elapsed_s
        max_velocity = math.degrees(
            max(abs(value) for value in sample.velocities)
        )
        max_torque = max(abs(value) for value in sample.commanded_torques)
        positions = " ".join(
            f"{math.degrees(value):+.1f}" for value in sample.positions
        )
        print(
            f"t={sample.elapsed_s:5.1f}s "
            f"max_velocity={max_velocity:5.2f}deg/s "
            f"max_torque={max_torque:5.2f}Nm "
            f"positions(deg)={positions}",
            flush=True,
        )

    try:
        print(
            "Gravity compensation will enable every arm joint and make the arm "
            "backdrivable. Support the arm and keep clear of joint limits.",
            flush=True,
        )
        for remaining in range(args.countdown, 0, -1):
            print(f"enabling in {remaining}...", flush=True)
            time.sleep(1.0)
        mode.start()
        print(
            f"gravity compensation active: Kp=0, Kd={args.damping:g}; "
            + (
                f"running for {args.seconds:g}s"
                if args.seconds > 0.0
                else "press Ctrl+C to stop"
            ),
            flush=True,
        )
        mode.run(seconds=args.seconds, on_sample=report)
        print("duration complete; restoring stiffness and disabling", flush=True)
    except KeyboardInterrupt:
        print("stop requested; restoring stiffness and disabling", flush=True)
    finally:
        mode.shutdown()
        print("all arm motors disabled", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seconds",
        type=non_negative_float,
        default=0.0,
        help="Active duration; 0 runs until Ctrl+C (default: 0)",
    )
    parser.add_argument(
        "--hz",
        type=positive_float,
        default=100.0,
        help="Command and feedback rate (default: 100)",
    )
    parser.add_argument(
        "--report-hz",
        type=positive_float,
        default=1.0,
        help="Status output rate (default: 1)",
    )
    parser.add_argument(
        "--transition-seconds",
        type=non_negative_float,
        default=0.0,
        help="Optional time to ramp stiffness and gravity torque (default: 0)",
    )
    parser.add_argument(
        "--settle-seconds",
        type=non_negative_float,
        default=0.0,
        help="Optional current-position hold before gravity torque (default: 0)",
    )
    parser.add_argument(
        "--gravity-scale",
        type=non_negative_float,
        default=1.0,
        help="Global multiplier for model gravity torques (default: 1)",
    )
    parser.add_argument(
        "--joint-scales",
        help=(
            "Comma-separated signed per-joint gravity multipliers; "
            "default: all 1"
        ),
    )
    parser.add_argument(
        "--damping",
        type=non_negative_float,
        default=0.0,
        help="Final MIT Kd; Kp is always 0 (default: 0, pure torque)",
    )
    parser.add_argument(
        "--countdown",
        type=non_negative_int,
        default=3,
        help="Seconds before enabling (default: 3)",
    )
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
