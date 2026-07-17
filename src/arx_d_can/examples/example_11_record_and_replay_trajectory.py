#!/usr/bin/env python3
"""Example 11: record and replay an arm-and-gripper trajectory."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from arx_d_can import ArxDCanArm


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


def save_trajectory(path: Path, hz: float, positions: list[list[float]]) -> None:
    path.write_text(
        json.dumps({"hz": hz, "positions": positions}),
        encoding="utf-8",
    )


def load_trajectory(path: Path) -> tuple[float, list[list[float]]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    hz = parse_hz(str(data["hz"]))
    positions = [[float(value) for value in point] for point in data["positions"]]
    if not positions or any(len(point) != 7 for point in positions):
        raise ValueError("trajectory must contain six arm joints and one gripper position")
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
    def send(index: int) -> None:
        arm.send_joint_positions(positions[index][:6])
        arm.set_gripper_motor_value(positions[index][6])

    run_at_hz(len(positions), hz, send)


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(port=args.port, baud=args.baud, enable_gripper=True)
    try:
        arm.connect()
        if args.command == "record":
            if args.seconds <= 0.0:
                raise ValueError("seconds must be greater than 0")
            positions = record(arm, seconds=args.seconds, hz=args.hz)
            save_trajectory(args.file, args.hz, positions)
            print(f"saved {len(positions)} samples at {args.hz:g} Hz to {args.file}")
            return

        hz, positions = load_trajectory(args.file)
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
        command.add_argument("--port", default="/dev/ttyACM0")
        command.add_argument("--baud", type=int, default=1_000_000)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
