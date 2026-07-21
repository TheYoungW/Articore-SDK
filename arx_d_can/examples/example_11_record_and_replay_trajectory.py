#!/usr/bin/env python3
"""Example 11: record and replay an arm-and-gripper trajectory."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments, arm_kwargs


DEFAULT_HZ = 100.0
MAX_HZ = 500.0


def parse_hz(value: str) -> float:
    hz = float(value)
    if hz <= 0.0 or hz > MAX_HZ:
        raise argparse.ArgumentTypeError(f"hz must be greater than 0 and at most {MAX_HZ:g}")
    return hz


def run_at_hz(count: int, hz: float, action) -> None:
    """Run action(index) at a fixed frequency."""
    started = time.perf_counter()
    for index in range(count):
        if index > 0:
            remaining = started + index / hz - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
        action(index)


def save_trajectory(
    path: Path,
    hz: float,
    positions: list[list[float]],
    *,
    joint_names: tuple[str, ...] | None = None,
) -> None:
    data: dict[str, object] = {"hz": hz, "positions": positions}
    if joint_names is not None:
        data["joint_names"] = list(joint_names)
    path.write_text(
        json.dumps(data),
        encoding="utf-8",
    )


def load_trajectory(
    path: Path,
    *,
    expected_joint_names: tuple[str, ...] | None = None,
) -> tuple[float, list[list[float]]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("trajectory file must contain a JSON object")
    hz = parse_hz(str(data["hz"]))
    positions = [[float(value) for value in point] for point in data["positions"]]
    if not positions:
        raise ValueError("trajectory must contain at least one sample")
    width = len(positions[0])
    if width < 2 or any(len(point) != width for point in positions):
        raise ValueError(
            "trajectory samples must have one value per arm joint plus one gripper value"
        )
    joint_names = data.get("joint_names")
    if joint_names is not None:
        if not isinstance(joint_names, list) or len(joint_names) + 1 != width:
            raise ValueError("trajectory joint_names do not match the recorded samples")
        if expected_joint_names is not None and tuple(joint_names) != expected_joint_names:
            raise ValueError(
                f"trajectory joints {tuple(joint_names)!r} do not match selected model "
                f"joints {expected_joint_names!r}"
            )
    return hz, positions


def record(arm: ArxDCanArm, *, seconds: float, hz: float) -> list[list[float]]:
    samples: list[list[float]] = []
    count = max(1, round(seconds * hz))

    def capture(_index: int) -> None:
        state = arm.read_state(request_feedback=True)
        if state.gripper is None:
            raise RuntimeError("gripper feedback is unavailable")
        samples.append(
            [float(value) for value in state.arm.positions]
            + [float(state.gripper.position)]
        )

    run_at_hz(count, hz, capture)
    return samples


def replay(arm: ArxDCanArm, *, hz: float, positions: list[list[float]]) -> None:
    if not positions:
        raise ValueError("trajectory must contain at least one sample")
    joint_count = len(getattr(arm, "joint_names", ())) or len(positions[0]) - 1
    expected_width = joint_count + 1
    if any(len(point) != expected_width for point in positions):
        raise ValueError(
            f"trajectory has {len(positions[0]) - 1} arm joints, "
            f"but the selected model has {joint_count}"
        )

    def send(index: int) -> None:
        arm.send_joint_positions(positions[index][:joint_count])
        arm.set_gripper_motor_value(positions[index][joint_count])

    run_at_hz(len(positions), hz, send)


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(enable_gripper=True, **arm_kwargs(args))
    try:
        arm.connect()
        if args.command == "record":
            if args.seconds <= 0.0:
                raise ValueError("seconds must be greater than 0")
            positions = record(arm, seconds=args.seconds, hz=args.hz)
            save_trajectory(
                args.file,
                args.hz,
                positions,
                joint_names=arm.joint_names,
            )
            print(f"saved {len(positions)} samples at {args.hz:g} Hz to {args.file}")
            return

        hz, positions = load_trajectory(
            args.file,
            expected_joint_names=arm.joint_names,
        )
        arm.configure()
        arm.enable()
        print(f"replaying {len(positions)} samples at {hz:g} Hz")
        replay(arm, hz=hz, positions=positions)
    finally:
        arm.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    record_parser = commands.add_parser("record", help="Record joint positions")
    record_parser.add_argument("file", type=Path, help="Output JSON trajectory")
    record_parser.add_argument("--seconds", type=float, default=10.0)
    record_parser.add_argument("--hz", type=parse_hz, default=DEFAULT_HZ)

    replay_parser = commands.add_parser("replay", help="Replay a saved trajectory")
    replay_parser.add_argument("file", type=Path, help="Input JSON trajectory")

    for command in (record_parser, replay_parser):
        add_connection_arguments(command)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
