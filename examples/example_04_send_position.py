#!/usr/bin/env python3
"""Example 04: directly send one six-joint position target."""
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


def hold_target(arm, target: tuple[float, ...], *, seconds: float, hz: float) -> None:
    period = 1.0 / max(1.0, hz)
    deadline = None if seconds <= 0.0 else time.monotonic() + seconds
    while deadline is None or time.monotonic() < deadline:
        arm.send_joint_positions(target)
        time.sleep(period)


def main(args: argparse.Namespace) -> None:
    target = parse_positions_degrees(args.positions)
    arm = ArxDCanArm(port=args.port, baud=args.baud)
    try:
        arm.connect()
        arm.configure()
        arm.enable()
        arm.send_joint_positions(target)
        print(
            "sent(deg):",
            " ".join(f"{math.degrees(value):+.3f}" for value in target),
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
            seconds=args.hold_seconds,
            hz=args.hz,
        )
        print("hold finished; all motors will now be disabled")
    except KeyboardInterrupt:
        print("stop requested; disabling all motors")
    finally:
        arm.close()


if __name__ == "__main__":
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
        "--hold-seconds",
        type=float,
        default=0.0,
        help="Keep refreshing the final target; 0 holds forever (default)",
    )
    parser.add_argument("--hz", type=float, default=100.0, help="Target refresh frequency")
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB2CAN serial port")
    parser.add_argument("--baud", type=int, default=1_000_000, help="USB2CAN serial baudrate")
    main(parser.parse_args())
