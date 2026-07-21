#!/usr/bin/env python3
"""Example 08: directly send every ARX-D-CAN motor to zero."""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments


def hold_zero(
    arm: ArxDCanArm,
    *,
    zero_position: tuple[float, ...],
    include_gripper: bool,
    velocity_limits: tuple[float, ...] | None,
    seconds: float,
    hz: float,
) -> None:
    period = 1.0 / max(1.0, hz)
    deadline = None if seconds <= 0.0 else time.monotonic() + seconds
    next_feedback = time.monotonic()
    next_progress = next_feedback
    reached = False
    while deadline is None or time.monotonic() < deadline:
        arm.send_joint_positions(
            zero_position,
            velocity_limits=velocity_limits,
        )
        if include_gripper:
            arm.set_gripper_motor_value(0.0)

        now = time.monotonic()
        if now >= next_feedback:
            state = arm.read_state(request_feedback=True)
            max_error = max(abs(value) for value in state.arm.positions)
            if state.gripper is not None:
                max_error = max(max_error, abs(state.gripper.position))
            if not reached and max_error <= math.radians(1.0):
                reached = True
                print("zero target reached within 1.0 deg", flush=True)
            if now >= next_progress:
                positions = " ".join(
                    f"{name}={math.degrees(value):+.2f}deg"
                    for name, value in zip(state.arm.names, state.arm.positions)
                )
                print(f"current: {positions}", flush=True)
                next_progress = now + 1.0
            next_feedback = now + 0.1

        time.sleep(max(0.0, period))


def main(args: argparse.Namespace) -> None:
    include_gripper = not args.arm_only
    arm = ArxDCanArm(
        model=args.arm_model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        enable_gripper=include_gripper,
    )
    zero_position = (0.0,) * len(arm.joint_names)
    velocity_limits = None
    if args.velocity_limit is not None:
        if not math.isfinite(args.velocity_limit) or args.velocity_limit <= 0.0:
            raise ValueError("--velocity-limit must be finite and positive")
        velocity_limits = (math.radians(args.velocity_limit),) * len(arm.joint_names)
    try:
        arm.connect()
        arm.configure()
        arm.enable()
        print("all requested motors confirmed ENABLED; sending zero target", flush=True)
        if velocity_limits is not None:
            print(f"PV velocity limit: {args.velocity_limit:g} deg/s", flush=True)
        if args.hold_seconds <= 0.0:
            print("holding continuously; press Ctrl+C to disable all motors", flush=True)
        hold_zero(
            arm,
            zero_position=zero_position,
            include_gripper=include_gripper,
            velocity_limits=velocity_limits,
            seconds=args.hold_seconds,
            hz=args.hz,
        )
        print("hold finished; all motors will now be disabled")
    except KeyboardInterrupt:
        print("stop requested; disabling all motors")
    except Exception as exc:
        print(f"return-to-zero aborted: {exc}", flush=True)
        raise
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
        "--velocity-limit",
        type=float,
        default=None,
        help=(
            "PV maximum velocity applied to all arm joints in deg/s; "
            "default: per-joint values from YAML"
        ),
    )
    parser.add_argument(
        "--arm-only",
        action="store_true",
        help="Return only the arm joints; do not connect to or move the gripper",
    )
    add_connection_arguments(parser)
    main(parser.parse_args())
