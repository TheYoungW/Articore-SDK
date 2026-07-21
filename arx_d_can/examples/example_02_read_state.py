#!/usr/bin/env python3
"""Example 02: read one or multiple state samples without enabling the arm."""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments, arm_kwargs


def print_state(arm, *, sample_index: int | None = None) -> None:
    state = arm.read_state()
    prefix = "" if sample_index is None else f"[{sample_index:04d}] "
    arm_positions = " ".join(
        f"{name}={math.degrees(pos):+.3f}"
        for name, pos in zip(state.arm.names, state.arm.positions)
    )
    print(f"{prefix}arm_pos(deg): {arm_positions}")
    arm_velocities = " ".join(
        f"{name}={math.degrees(vel):+.3f}"
        for name, vel in zip(state.arm.names, state.arm.velocities)
    )
    print(f"{prefix}arm_vel(deg/s): {arm_velocities}")
    if state.gripper is not None:
        print(
            f"{prefix}{state.gripper.name}: "
            f"pos={math.degrees(state.gripper.position):+.3f} deg "
            f"vel={math.degrees(state.gripper.velocity):+.3f} deg/s "
            f"tau={state.gripper.torque:+.6f}"
        )


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(**arm_kwargs(args))
    try:
        arm.connect()
        if not args.watch:
            print_state(arm)
            return

        sample_index = 0
        period = 1.0 / max(1.0, args.hz)
        while args.count <= 0 or sample_index < args.count:
            sample_index += 1
            print_state(arm, sample_index=sample_index)
            time.sleep(period)
    finally:
        arm.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read ARX-D-CAN state without enabling the arm.")
    parser.add_argument("--watch", action="store_true", help="Continuously print state")
    parser.add_argument("--hz", type=float, default=10.0, help="Print frequency in watch mode")
    parser.add_argument("--count", type=int, default=0, help="Stop after N samples in watch mode; 0 means forever")
    add_connection_arguments(parser)
    main(parser.parse_args())
