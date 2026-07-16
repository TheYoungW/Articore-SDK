#!/usr/bin/env python3
"""Example 04: send one six-joint target in PV or MIT mode."""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanArm


def parse_positions_degrees(text: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != 6:
        raise ValueError(f"expected 6 comma-separated joint positions, got {len(values)}")
    return tuple(math.radians(value) for value in values)


def parse_torques(text: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != 6:
        raise ValueError(f"expected 6 comma-separated joint torques, got {len(values)}")
    if any(not math.isfinite(value) for value in values):
        raise ValueError("joint torques must be finite")
    return values


def hold_target(
    arm,
    target: tuple[float, ...],
    *,
    torques: tuple[float, ...] | None = None,
    seconds: float,
    hz: float,
) -> None:
    period = 1.0 / max(1.0, hz)
    deadline = None if seconds <= 0.0 else time.monotonic() + seconds
    while deadline is None or time.monotonic() < deadline:
        arm.send_joint_positions(target, torques=torques)
        time.sleep(period)


def main(args: argparse.Namespace) -> None:
    target = parse_positions_degrees(args.positions)
    torques = None if args.torques is None else parse_torques(args.torques)
    if args.mode != "mit" and torques is not None:
        raise ValueError("--torques can only be used with --mode mit")
    arm = ArxDCanArm(
        port=args.port,
        baud=args.baud,
        control_mode=args.mode,
    )
    try:
        arm.connect()
        arm.configure()
        arm.enable()
        arm.send_joint_positions(target, torques=torques)
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
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB2CAN serial port")
    parser.add_argument("--baud", type=int, default=1_000_000, help="USB2CAN serial baudrate")
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
