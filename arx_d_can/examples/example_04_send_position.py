#!/usr/bin/env python3
"""Example 04: send one arm-joint target in PV or MIT mode."""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments


def parse_positions_degrees(text: str, *, expected_count: int = 6) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != expected_count:
        raise ValueError(
            f"expected {expected_count} comma-separated joint positions, got {len(values)}"
        )
    return tuple(math.radians(value) for value in values)


def parse_torques(text: str, *, expected_count: int = 6) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != expected_count:
        raise ValueError(
            f"expected {expected_count} comma-separated joint torques, got {len(values)}"
        )
    if any(not math.isfinite(value) for value in values):
        raise ValueError("joint torques must be finite")
    return values


def parse_velocities_degrees(
    text: str,
    *,
    require_positive: bool = False,
    expected_count: int = 6,
) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != expected_count:
        raise ValueError(
            f"expected {expected_count} comma-separated joint velocities, got {len(values)}"
        )
    if any(not math.isfinite(value) for value in values):
        raise ValueError("joint velocities must be finite")
    if require_positive and any(value <= 0.0 for value in values):
        raise ValueError("PV velocity limits must be positive")
    return tuple(math.radians(value) for value in values)


def hold_target(
    arm,
    target: tuple[float, ...],
    *,
    velocities: tuple[float, ...] | None = None,
    velocity_limits: tuple[float, ...] | None = None,
    torques: tuple[float, ...] | None = None,
    seconds: float,
    hz: float,
) -> None:
    period = 1.0 / max(1.0, hz)
    deadline = None if seconds <= 0.0 else time.monotonic() + seconds
    while deadline is None or time.monotonic() < deadline:
        arm.send_joint_positions(
            target,
            velocities=velocities,
            velocity_limits=velocity_limits,
            torques=torques,
        )
        time.sleep(period)


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        control_mode=args.mode,
    )
    # The fallback keeps simple third-party test doubles compatible. Real SDK
    # instances always expose joint_names from the selected model profile.
    joint_count = len(getattr(arm, "joint_names", ())) or 6
    target = parse_positions_degrees(args.positions, expected_count=joint_count)
    velocities = (
        None
        if args.velocities is None
        else parse_velocities_degrees(args.velocities, expected_count=joint_count)
    )
    velocity_limits = (
        None
        if args.velocity_limits is None
        else parse_velocities_degrees(
            args.velocity_limits,
            require_positive=True,
            expected_count=joint_count,
        )
    )
    torques = (
        None
        if args.torques is None
        else parse_torques(args.torques, expected_count=joint_count)
    )
    if args.mode != "mit" and velocities is not None:
        raise ValueError("--velocities can only be used with --mode mit")
    if args.mode != "mit" and torques is not None:
        raise ValueError("--torques can only be used with --mode mit")
    if args.mode != "pv" and velocity_limits is not None:
        raise ValueError("--velocity-limits can only be used with --mode pv")
    try:
        arm.connect()
        arm.configure()
        arm.enable()
        arm.send_joint_positions(
            target,
            velocities=velocities,
            velocity_limits=velocity_limits,
            torques=torques,
        )
        print(f"control mode: {args.mode.upper()}", flush=True)
        print(
            "sent(deg):",
            " ".join(f"{math.degrees(value):+.3f}" for value in target),
            flush=True,
        )
        if torques is not None:
            print(
                "feedforward torque(Nm):",
                " ".join(f"{value:+.3f}" for value in torques),
                flush=True,
            )
        if velocities is not None:
            print(f"MIT target velocities(deg/s): {args.velocities}", flush=True)
        if velocity_limits is not None:
            print(f"PV velocity limits(deg/s): {args.velocity_limits}", flush=True)

        if args.hold_seconds <= 0.0:
            print(
                "holding target continuously; support the arm before pressing Ctrl+C, "
                "because exit disables all motors",
                flush=True,
            )
        else:
            print(
                f"holding target for {args.hold_seconds:.3f}s; "
                "motors will be disabled afterwards",
                flush=True,
            )
        hold_target(
            arm,
            target,
            velocities=velocities,
            velocity_limits=velocity_limits,
            torques=torques,
            seconds=args.hold_seconds,
            hz=args.hz,
        )
        print("hold finished; all motors will now be disabled")
    except KeyboardInterrupt:
        print("stop requested; disabling all motors")
    finally:
        arm.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send a requested ARX-D-CAN joint position.")
    parser.add_argument(
        "--positions",
        default="0,-57.30,-57.30,0,34.38,0",
        help=(
            "Six comma-separated joint positions in degrees; default: "
            "0,-57.30,-57.30,0,34.38,0"
        ),
    )
    parser.add_argument(
        "--mode",
        choices=("pv", "mit"),
        default="pv",
        help="Motor control mode: pv (default) or mit",
    )
    parser.add_argument(
        "--velocities",
        help=(
            "Six comma-separated MIT target velocities in deg/s; "
            "MIT only, default: all zero"
        ),
    )
    parser.add_argument(
        "--velocity-limits",
        help=(
            "Six comma-separated positive PV maximum velocities in deg/s; "
            "PV only, default: values from YAML"
        ),
    )
    parser.add_argument(
        "--torques",
        help=(
            "Six comma-separated MIT feedforward torques in N*m; "
            "MIT only, default: all zero"
        ),
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="Keep refreshing the final target; 0 holds forever (default)",
    )
    parser.add_argument("--hz", type=float, default=100.0, help="Target refresh frequency")
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
