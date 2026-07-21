#!/usr/bin/env python3
"""Example 05: open and close the gripper."""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments


def send_gripper_for_seconds(
    arm,
    value: float,
    *,
    seconds: float,
    hz: float = 100.0,
    raw: bool = False,
) -> None:
    period = 1.0 / max(1.0, hz)
    deadline = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < deadline:
        if raw:
            arm.set_gripper_motor_value(value)
        else:
            arm.set_gripper(value)
        time.sleep(period)


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        enable_gripper=True,
    )
    try:
        arm.connect()
        arm.configure()
        arm.enable()
        mode = "raw motor value" if args.raw else "0..1000 mapped value"
        print(f"gripper command mode: {mode}")
        print("opening gripper")
        send_gripper_for_seconds(
            arm,
            args.open_value,
            seconds=args.open_seconds,
            hz=args.hz,
            raw=args.raw,
        )
        print("closing gripper")
        send_gripper_for_seconds(
            arm,
            args.closed_value,
            seconds=args.close_seconds,
            hz=args.hz,
            raw=args.raw,
        )
        print("gripper test finished; all motors will now be disabled")
    finally:
        arm.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Open and close the ARX-D-CAN gripper.")
    parser.add_argument("--open-value", type=float, default=1000.0)
    parser.add_argument("--closed-value", type=float, default=0.0)
    parser.add_argument("--open-seconds", type=float, default=2.0)
    parser.add_argument("--close-seconds", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--raw", action="store_true", help="Send raw gripper motor target values directly")
    add_connection_arguments(parser)
    main(parser.parse_args())
