#!/usr/bin/env python3
"""Example 03: clear all active ARX-D-CAN motor faults safely."""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanArm


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        port=args.port,
        baud=args.baud,
        enable_gripper=args.include_gripper,
    )
    try:
        arm.connect()
        names = arm.clear_motor_faults()
        state = arm.read_state(request_feedback=True)
        positions = " ".join(
            f"{name}={math.degrees(position):+.3f}deg"
            for name, position in zip(state.arm.names, state.arm.positions)
        )
        print("cleared:", " ".join(names))
        print("arm_pos:", positions)
        if state.gripper is not None:
            print(
                f"{state.gripper.name}="
                f"{math.degrees(state.gripper.position):+.3f}deg"
            )
        print("all cleared motors remain disabled")
    finally:
        arm.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Clear all active ARX-D-CAN motor faults without enabling or moving "
            "the arm. Remove any obstruction and support the arm first."
        )
    )
    parser.add_argument(
        "--include-gripper",
        action="store_true",
        help="Also clear the gripper motor fault",
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB2CAN serial port")
    parser.add_argument("--baud", type=int, default=1_000_000, help="USB2CAN serial baudrate")
    main(parser.parse_args())
