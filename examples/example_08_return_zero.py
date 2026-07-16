#!/usr/bin/env python3
"""Example 08: directly send every ARX-D-CAN motor to zero."""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanArm


ZERO_ARM_POSITION = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def hold_zero(
    arm: ArxDCanArm,
    *,
    include_gripper: bool,
    seconds: float,
    hz: float,
) -> None:
    period = 1.0 / max(1.0, hz)
    deadline = None if seconds <= 0.0 else time.monotonic() + seconds
    while deadline is None or time.monotonic() < deadline:
        arm.send_joint_positions(ZERO_ARM_POSITION)
        if include_gripper:
            arm.set_gripper_motor_value(0.0)
        time.sleep(period)


def main(args: argparse.Namespace) -> None:
    include_gripper = not args.arm_only
    arm = ArxDCanArm(
        port=args.port,
        baud=args.baud,
        enable_gripper=include_gripper,
    )
    try:
        arm.connect()
        arm.configure()
        arm.enable()
        print("sending all enabled motors directly to zero", flush=True)
        hold_zero(
            arm,
            include_gripper=include_gripper,
            seconds=args.hold_seconds,
            hz=args.hz,
        )
        print("hold finished; all motors will now be disabled")
    except KeyboardInterrupt:
        print("stop requested; disabling all motors")
    finally:
        arm.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Directly send all ARX-D-CAN motors to 0 degrees without changing "
            "their zero calibration. The gripper closes to motor zero by default."
        )
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="Keep refreshing zero; 0 holds forever (default)",
    )
    parser.add_argument("--hz", type=float, default=100.0, help="Target refresh frequency")
    parser.add_argument(
        "--arm-only",
        action="store_true",
        help="Return only the six arm joints; do not connect to or move the gripper",
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB2CAN serial port")
    parser.add_argument("--baud", type=int, default=1_000_000, help="USB2CAN serial baudrate")
    main(parser.parse_args())
