#!/usr/bin/env python3
"""Service tool: safely write and verify Damiao motor zero positions."""
from __future__ import annotations

import argparse
import time

try:
    from arx_d_can.examples.common import add_connection_arguments, make_arm
except ModuleNotFoundError:
    from teleop.adapters.arm.arx_d_can.examples.common import (
        add_connection_arguments,
        make_arm,
    )


VERIFY_SAMPLES = 3


def named_positions(state) -> dict[str, float]:
    positions = dict(zip(state.arm.names, state.arm.positions))
    if state.gripper is not None:
        positions[state.gripper.name] = state.gripper.position
    return positions


def read_complete_state(arm, *, attempts: int, interval: float):
    last_error: RuntimeError | None = None
    for attempt in range(1, attempts + 1):
        try:
            return arm.read_state(request_feedback=True)
        except RuntimeError as error:
            last_error = error
            if attempt < attempts:
                time.sleep(interval)
    raise RuntimeError(
        f"incomplete motor feedback after {attempts} attempts; zero was NOT written: "
        f"{last_error}"
    ) from last_error


def require_stationary(
    arm,
    *,
    duration_s: float,
    sample_hz: float,
    max_velocity: float,
    max_movement: float,
    include_gripper: bool,
):
    period = 1.0 / max(1.0, sample_hz)
    deadline = time.monotonic() + max(0.1, duration_s)
    first = read_complete_state(arm, attempts=10, interval=0.05)
    latest = first
    peak_velocity = max(abs(value) for value in first.arm.velocities)
    if include_gripper:
        if first.gripper is None:
            raise RuntimeError("gripper feedback is unavailable")
        peak_velocity = max(peak_velocity, abs(first.gripper.velocity))
    while time.monotonic() < deadline:
        time.sleep(period)
        latest = read_complete_state(arm, attempts=3, interval=0.02)
        peak_velocity = max(
            peak_velocity,
            *(abs(value) for value in latest.arm.velocities),
        )
        if include_gripper:
            if latest.gripper is None:
                raise RuntimeError("gripper feedback is unavailable")
            peak_velocity = max(peak_velocity, abs(latest.gripper.velocity))
    movement = max(
        abs(end - start)
        for start, end in zip(first.arm.positions, latest.arm.positions)
    )
    if include_gripper:
        movement = max(
            movement,
            abs(latest.gripper.position - first.gripper.position),
        )
    if peak_velocity > max_velocity or movement > max_movement:
        raise RuntimeError(
            "arm is not stationary; zero was NOT written: "
            f"peak_velocity={peak_velocity:.6f} rad/s (limit {max_velocity:.6f}), "
            f"movement={movement:.6f} rad (limit {max_movement:.6f})"
        )
    return latest


def main(args: argparse.Namespace) -> None:
    arm = make_arm(args, enable_gripper=args.include_gripper)
    try:
        arm.connect()
        state = require_stationary(
            arm,
            duration_s=args.stationary_seconds,
            sample_hz=args.stationary_hz,
            max_velocity=args.max_velocity,
            max_movement=args.max_movement,
            include_gripper=args.include_gripper,
        )
        print("verified stationary arm position:")
        for name, position in zip(state.arm.names, state.arm.positions):
            print(f"  {name}: {position:+.6f} rad")
        if state.gripper is not None:
            print(f"  {state.gripper.name}: {state.gripper.position:+.6f} rad")

        before_positions = named_positions(state)
        joint_names = list(state.arm.names)
        if args.include_gripper:
            if state.gripper is None:
                raise RuntimeError("gripper feedback is unavailable")
            joint_names.append(state.gripper.name)
        for name in joint_names:
            if abs(before_positions[name]) <= args.verify_tolerance:
                print(
                    f"warning: {name} was already near zero; position feedback "
                    "alone cannot prove that the zero command was executed"
                )
        completed = arm.set_zero(
            joint_names=joint_names,
            verify_tolerance=args.verify_tolerance,
            verify_velocity=args.max_velocity,
            verify_samples=VERIFY_SAMPLES,
        )
        verified = read_complete_state(arm, attempts=10, interval=0.05)
        after_positions = named_positions(verified)
        print(f"zero position written and verified with {VERIFY_SAMPLES} fresh samples:")
        for name in completed:
            print(
                f"  {name}: {before_positions[name]:+.6f} -> "
                f"{after_positions[name]:+.6f} rad"
            )
    finally:
        arm.close()


def build_parser(
    *,
    description: str = "Write current stationary ARX-D-CAN motor positions as zero.",
) -> argparse.ArgumentParser:
    """Build the shared safe-zero command-line interface."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--include-gripper",
        action="store_true",
        help="Also zero the gripper; default only zeros the configured arm joints",
    )
    parser.add_argument("--stationary-seconds", type=float, default=1.0)
    parser.add_argument("--stationary-hz", type=float, default=20.0)
    parser.add_argument("--max-velocity", type=float, default=0.05)
    parser.add_argument("--max-movement", type=float, default=0.01)
    parser.add_argument("--verify-tolerance", type=float, default=0.02)
    add_connection_arguments(parser)
    return parser


def cli() -> None:
    main(build_parser().parse_args())


if __name__ == "__main__":
    cli()
