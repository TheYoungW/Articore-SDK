"""机械臂与夹爪轨迹录制、保存和回放。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from arx_d_can import ArxDCanArm, GravityCompensationMode
from .common import add_connection_arguments, make_arm


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
    path.write_text(json.dumps(data), encoding="utf-8")


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
    """按指定频率录制机械臂和夹爪状态。"""
    samples: list[list[float]] = []
    timestamps: list[float] = []
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

        if gravity_mode is None:
            state = arm.read_state()
            arm_positions = state.arm.positions
        else:
            gravity_sample = gravity_mode.step()
            state = arm.read_cached_state()
            arm_positions = gravity_sample.positions
        if state.gripper is None:
            raise RuntimeError("gripper feedback is unavailable")
        if zero_stiffness:
            send_zero_stiffness(arm, state.arm.positions)
        samples.append(
            [float(value) for value in arm_positions]
            + [float(state.gripper.position)]
        )
        captured_at = time.perf_counter()
        timestamps.append(captured_at - started)
        scheduled_index = max(
            scheduled_index + 1,
            math.floor((captured_at - started) * hz) + 1,
        )

    if not samples:
        raise RuntimeError("recording produced no trajectory samples")
    first_timestamp = timestamps[0]
    return [value - first_timestamp for value in timestamps], samples


def send_zero_stiffness(
    arm: ArxDCanArm,
    positions,
    *,
    require_enabled: bool = True,
) -> None:
    """发送不含重力补偿的零增益、零力矩 MIT 帧。"""
    zeros = (0.0,) * len(positions)
    arm._submit_joint_positions(
        positions,
        velocities=zeros,
        torques=zeros,
        mit_kp=zeros,
        mit_kd=zeros,
        require_enabled=require_enabled,
    )


def replay(
    arm: ArxDCanArm,
    *,
    timestamps: list[float],
    positions: list[list[float]],
) -> None:
    """按照录制时间戳回放机械臂和夹爪命令。"""
    if not positions:
        raise ValueError("trajectory must contain at least one sample")
    joint_count = len(getattr(arm, "joint_names", ())) or len(positions[0]) - 1
    if any(len(point) != joint_count + 1 for point in positions):
        raise ValueError("trajectory joint count does not match the selected model")
    if len(timestamps) != len(positions):
        raise ValueError("timestamps and positions must have the same length")

    started = time.perf_counter()
    first_timestamp = timestamps[0]
    for point, timestamp in zip(positions, timestamps):
        remaining = started + timestamp - first_timestamp - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        arm.stream_joint_positions(point[:joint_count])
        arm.set_gripper_motor_value(point[joint_count])


def run(args: argparse.Namespace) -> None:
    needs_mit = args.command == "record" and (
        args.enable or args.gravity_compensation
    )
    arm = make_arm(
        args,
        enable_gripper=True,
        control_mode="mit" if needs_mit else "pv",
    )
    arm.connect()
    print("机器人连接成功")
    try:
        if args.command == "record":
            gravity_mode = None
            if args.gravity_compensation:
                gravity_mode = GravityCompensationMode(arm, hz=args.hz)
                gravity_mode.start()
            elif args.enable:
                initial = arm.read_state()
                zeros = (0.0,) * len(initial.arm.positions)
                arm.enable(
                    initial_positions=initial.arm.positions,
                    mit_kp=zeros,
                    mit_kd=zeros,
                )
            try:
                timestamps, positions = record(
                    arm,
                    seconds=args.seconds,
                    hz=args.hz,
                    zero_stiffness=args.enable,
                    gravity_mode=gravity_mode,
                )
            finally:
                if gravity_mode is not None:
                    gravity_mode.shutdown()
                elif args.enable and arm.enabled:
                    arm.disable()
            save_trajectory(
                args.file,
                args.hz,
                positions,
                timestamps=timestamps,
                joint_names=arm.joint_names,
            )
            print(f"已保存 {len(positions)} 个轨迹点：{args.file}")
            return

        _, timestamps, positions = load_trajectory(
            args.file,
            expected_joint_names=arm.joint_names,
        )
        arm.enable()
        replay(arm, timestamps=timestamps, positions=positions)
        print("轨迹回放完成")
    finally:
        arm.close()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="录制或回放机械臂与夹爪轨迹")
    commands = parser.add_subparsers(dest="command", required=True)

    record_parser = commands.add_parser("record", help="录制轨迹")
    record_parser.add_argument("file", type=Path)
    record_parser.add_argument("--seconds", type=float, default=10.0)
    record_parser.add_argument("--hz", type=parse_hz, default=DEFAULT_HZ)
    mode = record_parser.add_mutually_exclusive_group()
    mode.add_argument("--enable", action="store_true")
    mode.add_argument("--gravity-compensation", action="store_true")

    replay_parser = commands.add_parser("replay", help="回放轨迹")
    replay_parser.add_argument("file", type=Path)

    for command in (record_parser, replay_parser):
        add_connection_arguments(
            command,
            allow_custom_config=True,
            default_arm_model=None,
        )
    return parser


__all__ = [
    "build_parser",
    "load_trajectory",
    "parse_hz",
    "record",
    "replay",
    "run",
    "save_trajectory",
    "send_zero_stiffness",
]
