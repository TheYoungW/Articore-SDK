#!/usr/bin/env python3
"""Example 11: record and replay an arm-and-gripper trajectory."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from arx_d_can import ArxDCanArm, GravityCompensationMode
from arx_d_can.examples.common import add_connection_arguments


DEFAULT_HZ = 100.0
MAX_HZ = 500.0
TRAJECTORY_FORMAT_VERSION = 2


def parse_hz(value: str) -> float:
    hz = float(value)
    if hz <= 0.0 or hz > MAX_HZ:
        raise argparse.ArgumentTypeError(f"hz must be greater than 0 and at most {MAX_HZ:g}")
    return hz


def save_trajectory(
    path: Path,
    hz: float,
    positions: list[list[float]],
    *,
    timestamps: list[float] | None = None,
    joint_names: tuple[str, ...] | None = None,
) -> None:
    data: dict[str, object] = {
        "format_version": TRAJECTORY_FORMAT_VERSION,
        "hz": hz,
        "positions": positions,
    }
    if timestamps is not None:
        if len(timestamps) != len(positions):
            raise ValueError("timestamps and positions must have the same length")
        data["timestamps"] = timestamps
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
) -> tuple[float, list[float], list[list[float]]]:
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
    raw_timestamps = data.get("timestamps")
    if raw_timestamps is None:
        timestamps = [index / hz for index in range(len(positions))]
    else:
        if not isinstance(raw_timestamps, list) or len(raw_timestamps) != len(positions):
            raise ValueError("trajectory timestamps do not match recorded samples")
        timestamps = [float(value) for value in raw_timestamps]
        if any(not math.isfinite(value) for value in timestamps):
            raise ValueError("trajectory timestamps must be finite")
        if any(
            current <= previous
            for previous, current in zip(timestamps, timestamps[1:])
        ):
            raise ValueError("trajectory timestamps must be strictly increasing")
    return hz, timestamps, positions


def record(
    arm: ArxDCanArm,
    *,
    seconds: float,
    hz: float,
    zero_stiffness: bool = False,
    gravity_mode: GravityCompensationMode | None = None,
) -> tuple[list[float], list[list[float]]]:
    samples: list[list[float]] = []
    timestamps: list[float] = []

    def capture(_index: int) -> None:
        if gravity_mode is None:
            state = arm.read_state(request_feedback=True)
            arm_positions = state.arm.positions
        else:
            gravity_sample = gravity_mode.step()
            state = arm.read_state(request_feedback=False)
            arm_positions = gravity_sample.positions
        if state.gripper is None:
            raise RuntimeError("gripper feedback is unavailable")
        if zero_stiffness:
            send_zero_stiffness(arm, state.arm.positions)
        samples.append(
            [float(value) for value in arm_positions]
            + [float(state.gripper.position)]
        )

    started = time.perf_counter()
    deadline = started + seconds
    scheduled_index = 0
    while True:
        scheduled_at = started + scheduled_index / hz
        if scheduled_at >= deadline:
            break
        remaining = scheduled_at - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        if time.perf_counter() >= deadline:
            break

        capture(scheduled_index)
        captured_at = time.perf_counter()
        timestamps.append(captured_at - started)

        # If feedback/command processing overran one or more periods, skip
        # those expired slots instead of capturing them late in a burst.
        scheduled_index = max(
            scheduled_index + 1,
            math.floor((captured_at - started) * hz) + 1,
        )

    if not samples:
        raise RuntimeError("recording produced no trajectory samples")
    first_timestamp = timestamps[0]
    timestamps = [value - first_timestamp for value in timestamps]
    return timestamps, samples


def send_zero_stiffness(
    arm: ArxDCanArm,
    positions,
    *,
    require_enabled: bool = True,
) -> None:
    """Send a zero-gain, zero-torque MIT frame without gravity compensation."""
    zeros = (0.0,) * len(positions)
    arm.send_joint_positions(
        positions,
        velocities=zeros,
        torques=zeros,
        mit_kp=zeros,
        mit_kd=zeros,
        mode="mit",
        require_enabled=require_enabled,
    )


def replay(
    arm: ArxDCanArm,
    *,
    timestamps: list[float],
    positions: list[list[float]],
) -> None:
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

    if len(timestamps) != len(positions):
        raise ValueError("timestamps and positions must have the same length")
    started = time.perf_counter()
    first_timestamp = timestamps[0]
    for index, timestamp in enumerate(timestamps):
        remaining = started + timestamp - first_timestamp - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        send(index)


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        transport=getattr(args, "transport", None),
        enable_gripper=True,
    )
    try:
        arm.connect()
        if args.command == "record":
            if args.seconds <= 0.0:
                raise ValueError("seconds must be greater than 0")
            gravity_mode = None
            if args.gravity_compensation:
                print(
                    "starting Pinocchio gravity compensation "
                    "(Kp=0, Kd=0) for recording",
                    flush=True,
                )
                gravity_mode = GravityCompensationMode(arm, hz=args.hz)
                gravity_mode.start()
                print("gravity compensation active; recording started", flush=True)
            elif args.enable:
                print(
                    "configuring MIT zero-stiffness mode (Kp=0, Kd=0, torque=0); "
                    "support the arm because gravity is not compensated",
                    flush=True,
                )
                arm.configure("mit")
                initial_state = arm.read_state(request_feedback=True)
                send_zero_stiffness(
                    arm,
                    initial_state.arm.positions,
                    require_enabled=False,
                )
                arm.enable()
                print("zero-stiffness arm enabled; recording started", flush=True)
            try:
                timestamps, positions = record(
                    arm,
                    seconds=args.seconds,
                    hz=args.hz,
                    zero_stiffness=args.enable,
                    gravity_mode=gravity_mode,
                )
            finally:
                if gravity_mode is not None and gravity_mode.active:
                    print(
                        "recording stopped; shutting down gravity compensation",
                        flush=True,
                    )
                    gravity_mode.shutdown()
                elif args.enable and arm.enabled:
                    print("recording stopped; disabling the arm", flush=True)
                    arm.disable()
            save_trajectory(
                args.file,
                args.hz,
                positions,
                timestamps=timestamps,
                joint_names=arm.joint_names,
            )
            recorded_duration = timestamps[-1] if len(timestamps) > 1 else 0.0
            achieved_hz = (
                (len(timestamps) - 1) / recorded_duration
                if recorded_duration > 0.0
                else 0.0
            )
            print(
                f"saved {len(positions)} timestamped samples over "
                f"{recorded_duration:.3f}s (requested {args.hz:g} Hz, "
                f"achieved {achieved_hz:.2f} Hz) to {args.file}"
            )
            return

        hz, timestamps, positions = load_trajectory(
            args.file,
            expected_joint_names=arm.joint_names,
        )
        arm.configure()
        arm.enable()
        duration = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0
        average_hz = (len(timestamps) - 1) / duration if duration > 0.0 else 0.0
        print(
            f"replaying {len(positions)} samples over {duration:.3f}s "
            f"(recorded average {average_hz:.2f} Hz, requested {hz:g} Hz)"
        )
        replay(arm, timestamps=timestamps, positions=positions)
    finally:
        arm.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    record_parser = commands.add_parser("record", help="Record joint positions")
    record_parser.add_argument("file", type=Path, help="Output JSON trajectory")
    record_parser.add_argument("--seconds", type=float, default=10.0)
    record_parser.add_argument("--hz", type=parse_hz, default=DEFAULT_HZ)
    record_mode = record_parser.add_mutually_exclusive_group()
    record_mode.add_argument(
        "--enable",
        action="store_true",
        help=(
            "Enable MIT zero-stiffness/zero-torque mode while recording, then "
            "disable the arm"
        ),
    )
    record_mode.add_argument(
        "--gravity-compensation",
        action="store_true",
        help=(
            "Enable MIT Pinocchio gravity compensation while recording, then "
            "disable the arm"
        ),
    )

    replay_parser = commands.add_parser("replay", help="Replay a saved trajectory")
    replay_parser.add_argument("file", type=Path, help="Input JSON trajectory")

    for command in (record_parser, replay_parser):
        add_connection_arguments(command)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
