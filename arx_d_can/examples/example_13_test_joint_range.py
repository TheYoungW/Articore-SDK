#!/usr/bin/env python3
"""Move selected joints one at a time without leaving their URDF limits."""
from __future__ import annotations

import argparse
from dataclasses import replace
import math
import time
from typing import Sequence

from arx_d_can import ArxDCanArm, ArxDCanConfig, JointMotorConfig, default_config
from arx_d_can.examples.common import add_connection_arguments
from arx_d_can.trajectory import plan_joint_position_trajectory


DEFAULT_JOINTS = tuple(f"right_leg_joint{index}" for index in range(1, 5))


def parse_joint_names(text: str) -> tuple[str, ...]:
    names = tuple(value.strip() for value in text.split(",") if value.strip())
    if not names:
        raise argparse.ArgumentTypeError("at least one joint name is required")
    if len(set(names)) != len(names):
        raise argparse.ArgumentTypeError("joint names must be unique")
    return names


def positive_finite(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive and finite")
    return parsed


def range_percent(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or not 0.0 < parsed < 100.0:
        raise argparse.ArgumentTypeError("range percent must be between 0 and 100")
    return parsed


def nonnegative_finite(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be non-negative and finite")
    return parsed


def select_joint_config(
    config: ArxDCanConfig,
    joint_names: Sequence[str],
) -> ArxDCanConfig:
    by_name = {joint.name: joint for joint in config.arm_joints}
    missing = [name for name in joint_names if name not in by_name]
    if missing:
        raise ValueError("unknown arm joints: " + ", ".join(missing))
    return replace(
        config,
        arm_joints=tuple(by_name[name] for name in joint_names),
        gripper=None,
        gripper_force_control_enabled=False,
    )


def calculate_sweep_targets(
    joint: JointMotorConfig,
    start: float,
    *,
    range_fraction: float,
) -> tuple[float, float]:
    lower = joint.lower_limit
    upper = joint.upper_limit
    if lower is None or upper is None:
        raise ValueError(f"{joint.name} does not have finite URDF position limits")
    if not lower <= start <= upper:
        raise ValueError(
            f"{joint.name} starts outside its URDF range: "
            f"{math.degrees(start):+.3f}deg not in "
            f"[{math.degrees(lower):+.3f}, {math.degrees(upper):+.3f}]deg"
        )
    if not math.isfinite(range_fraction) or not 0.0 < range_fraction < 1.0:
        raise ValueError("range fraction must be between 0 and 1")
    neutral = min(upper, max(lower, 0.0))
    lower_target = neutral + (lower - neutral) * range_fraction
    upper_target = neutral + (upper - neutral) * range_fraction
    if lower_target >= upper_target:
        raise ValueError(f"{joint.name} has no usable sweep range around its start position")
    return lower_target, upper_target


def format_positions(names: Sequence[str], positions: Sequence[float]) -> str:
    return " ".join(
        f"{name}={math.degrees(position):+.3f}deg"
        for name, position in zip(names, positions)
    )


def execute_move(
    arm: ArxDCanArm,
    start: Sequence[float],
    target: Sequence[float],
    *,
    duration: float,
    hz: float,
    label: str,
) -> tuple[float, ...]:
    points = plan_joint_position_trajectory(
        start,
        target,
        duration=duration,
        hz=hz,
        profile="min_jerk",
    )
    started = time.perf_counter()
    next_report = started
    for point in points:
        deadline = started + point.time
        remaining = deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        arm.send_joint_positions(point.positions)
        now = time.perf_counter()
        if now >= next_report:
            state = arm.read_state(request_feedback=False).arm
            print(
                f"{label} t={point.time:.1f}/{duration:.1f}s "
                f"{format_positions(state.names, state.positions)}",
                flush=True,
            )
            next_report = now + 0.5
    return tuple(float(value) for value in target)


def hold_target(
    arm: ArxDCanArm,
    target: Sequence[float],
    *,
    seconds: float,
    hz: float,
) -> None:
    started = time.perf_counter()
    cycle = 0
    while time.perf_counter() - started < seconds:
        arm.send_joint_positions(target)
        cycle += 1
        remaining = started + cycle / hz - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)


def run(args: argparse.Namespace) -> int:
    model = args.arm_model
    if model is None and args.config_path is None:
        model = "corina_v2"
    config = default_config(
        model=model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        transport=args.transport,
        control_hz=args.hz,
        arm_control_mode="posvel",
    )
    config = select_joint_config(config, args.joints)
    arm = ArxDCanArm(config=config)
    sweep_fraction = args.range_percent / 100.0
    tolerance = math.radians(args.tolerance_deg)

    try:
        arm.connect()
        initial = arm.read_state(request_feedback=True).arm
        statuses = arm.robot.get_status_codes(joint_names=list(args.joints))
        unexpected = {name: status for name, status in statuses.items() if status != 0}
        if unexpected:
            raise RuntimeError(
                "selected motors must initially be disabled and fault-free: "
                + str(unexpected)
            )

        sweeps = []
        print("initial:", format_positions(initial.names, initial.positions), flush=True)
        for joint, start in zip(config.arm_joints, initial.positions):
            lower_target, upper_target = calculate_sweep_targets(
                joint,
                start,
                range_fraction=sweep_fraction,
            )
            sweeps.append((lower_target, upper_target))
            assert joint.lower_limit is not None and joint.upper_limit is not None
            print(
                f"{joint.name}: URDF="
                f"[{math.degrees(joint.lower_limit):+.3f},"
                f" {math.degrees(joint.upper_limit):+.3f}]deg "
                f"test=[{math.degrees(lower_target):+.3f},"
                f" {math.degrees(upper_target):+.3f}]deg",
                flush=True,
            )

        print(
            "Only the listed joints will be enabled. Each joint moves alone while "
            "the other listed joints hold their initial positions.",
            flush=True,
        )
        for remaining in range(args.countdown, 0, -1):
            print(f"enabling in {remaining}...", flush=True)
            time.sleep(1.0)

        arm.configure("posvel")
        arm.enable()
        command = tuple(initial.positions)
        arm.send_joint_positions(command)

        for joint_index, joint in enumerate(config.arm_joints):
            start_position = initial.positions[joint_index]
            lower_target, upper_target = sweeps[joint_index]
            for target_label, joint_target in (
                ("lower", lower_target),
                ("upper", upper_target),
                ("return", start_position),
            ):
                target = list(command)
                target[joint_index] = joint_target
                command = execute_move(
                    arm,
                    command,
                    target,
                    duration=args.duration,
                    hz=args.hz,
                    label=f"{joint.name}:{target_label}",
                )
                hold_target(arm, command, seconds=args.hold_seconds, hz=args.hz)
                reached = arm.read_state(request_feedback=True).arm
                error = abs(reached.positions[joint_index] - joint_target)
                print(
                    f"{joint.name}:{target_label} reached="
                    f"{math.degrees(reached.positions[joint_index]):+.3f}deg "
                    f"error={math.degrees(error):.3f}deg",
                    flush=True,
                )
                if error > tolerance:
                    raise RuntimeError(
                        f"{joint.name} tracking error {math.degrees(error):.3f}deg "
                        f"exceeded {args.tolerance_deg:.3f}deg"
                    )
        print("all selected joint sweeps completed", flush=True)
        return 0
    except KeyboardInterrupt:
        print("interrupted", flush=True)
        return 130
    finally:
        arm.close()
        print("all selected motors disabled", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep selected joints one at a time toward a percentage of their "
            "URDF limits."
        )
    )
    add_connection_arguments(parser)
    parser.add_argument(
        "--joints",
        type=parse_joint_names,
        default=DEFAULT_JOINTS,
        help=(
            "Comma-separated controlled joints; default: " + ",".join(DEFAULT_JOINTS)
        ),
    )
    parser.add_argument(
        "--range-percent",
        type=range_percent,
        default=95.0,
        help="Fraction of the zero-to-URDF-limit travel used in each direction",
    )
    parser.add_argument("--duration", type=positive_finite, default=6.0)
    parser.add_argument("--hold-seconds", type=nonnegative_finite, default=0.5)
    parser.add_argument("--hz", type=positive_finite, default=200.0)
    parser.add_argument("--tolerance-deg", type=positive_finite, default=1.0)
    parser.add_argument("--countdown", type=int, default=3)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.countdown < 0:
        raise SystemExit("--countdown must be non-negative")
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
