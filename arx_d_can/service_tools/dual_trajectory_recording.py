"""双臂关节与夹爪轨迹的录制、保存和回放。"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..sdk.dual_arm import ArxDCanDualArm


FORMAT_VERSION = 1
MAX_RECORDING_HZ = 500.0


@dataclass(slots=True, frozen=True)
class DualArmTrajectorySample:
    """一条双臂关节与可选夹爪命令。"""

    left_positions: tuple[float, ...]
    right_positions: tuple[float, ...]
    left_gripper: float | None
    right_gripper: float | None


def _frequency(value: float) -> float:
    hz = float(value)
    if not math.isfinite(hz) or hz <= 0.0:
        raise ValueError("hz must be finite and positive")
    if hz > MAX_RECORDING_HZ:
        raise ValueError(f"hz must not exceed {MAX_RECORDING_HZ:g}")
    return hz


def _speed_percent(value: float) -> float:
    speed = float(value)
    if not math.isfinite(speed) or not 0.0 <= speed <= 100.0:
        raise ValueError("velocity must be finite and in the range 0..100")
    return speed


def _duration(value: float) -> float:
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ValueError("seconds must be finite and positive")
    return seconds


def record(
    robot: ArxDCanDualArm,
    *,
    seconds: float,
    hz: float,
) -> tuple[list[float], list[DualArmTrajectorySample]]:
    """按固定频率录制双臂 Runtime 缓存反馈。"""
    duration = _duration(seconds)
    frequency = _frequency(hz)
    timestamps: list[float] = []
    samples: list[DualArmTrajectorySample] = []
    started = time.perf_counter()
    deadline = started + duration
    index = 0
    while True:
        scheduled_at = started + index / frequency
        if scheduled_at >= deadline:
            break
        remaining = scheduled_at - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        if time.perf_counter() >= deadline:
            break
        health = robot.get_health()
        if health.safe_holding or health.fault_reason:
            raise RuntimeError(health.fault_reason or "dual arm entered safe hold")
        state = robot.read_cached_state()
        left_positions = state.left.arm.positions
        right_positions = state.right.arm.positions
        samples.append(
            DualArmTrajectorySample(
                left_positions=tuple(left_positions),
                right_positions=tuple(right_positions),
                left_gripper=(
                    None
                    if state.left.gripper is None
                    else float(state.left.gripper.opening)
                ),
                right_gripper=(
                    None
                    if state.right.gripper is None
                    else float(state.right.gripper.opening)
                ),
            )
        )
        captured_at = time.perf_counter()
        timestamps.append(captured_at - started)
        index = max(index + 1, math.floor((captured_at - started) * frequency) + 1)
    if not samples:
        raise RuntimeError("recording produced no trajectory samples")
    first = timestamps[0]
    return [timestamp - first for timestamp in timestamps], samples


def save_trajectory(
    path: Path,
    *,
    hz: float,
    timestamps: list[float],
    samples: list[DualArmTrajectorySample],
    left_joint_names: tuple[str, ...],
    right_joint_names: tuple[str, ...],
) -> None:
    """保存带关节名称和时间戳的双臂轨迹。"""
    frequency = _frequency(hz)
    if len(timestamps) != len(samples) or not samples:
        raise ValueError("timestamps and samples must have the same non-zero length")
    data = {
        "format_version": FORMAT_VERSION,
        "hz": frequency,
        "left_joint_names": list(left_joint_names),
        "right_joint_names": list(right_joint_names),
        "timestamps": [float(value) for value in timestamps],
        "samples": [
            {
                "left_positions": list(sample.left_positions),
                "right_positions": list(sample.right_positions),
                "left_gripper": sample.left_gripper,
                "right_gripper": sample.right_gripper,
            }
            for sample in samples
        ],
    }
    path.write_text(json.dumps(data), encoding="utf-8")


def load_trajectory(
    path: Path,
    *,
    expected_left_joint_names: tuple[str, ...],
    expected_right_joint_names: tuple[str, ...],
) -> tuple[list[float], list[DualArmTrajectorySample]]:
    """读取并校验双臂轨迹。"""
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("format_version") != FORMAT_VERSION:
        raise ValueError("unsupported dual-arm trajectory format")
    _frequency(data.get("hz", 0.0))
    if tuple(data.get("left_joint_names", ())) != expected_left_joint_names:
        raise ValueError("trajectory left joints do not match the selected model")
    if tuple(data.get("right_joint_names", ())) != expected_right_joint_names:
        raise ValueError("trajectory right joints do not match the selected model")
    raw_timestamps = data.get("timestamps")
    raw_samples = data.get("samples")
    if not isinstance(raw_timestamps, list) or not isinstance(raw_samples, list):
        raise ValueError("trajectory timestamps and samples must be lists")
    if not raw_samples or len(raw_timestamps) != len(raw_samples):
        raise ValueError("trajectory timestamps and samples must have equal length")
    timestamps = [float(value) for value in raw_timestamps]
    if any(not math.isfinite(value) for value in timestamps):
        raise ValueError("trajectory timestamps must be finite")
    if any(
        current <= previous
        for previous, current in zip(timestamps, timestamps[1:])
    ):
        raise ValueError("trajectory timestamps must be strictly increasing")

    samples: list[DualArmTrajectorySample] = []
    for raw in raw_samples:
        if not isinstance(raw, dict):
            raise ValueError("trajectory sample must be a mapping")
        left = tuple(float(value) for value in raw.get("left_positions", ()))
        right = tuple(float(value) for value in raw.get("right_positions", ()))
        if len(left) != len(expected_left_joint_names):
            raise ValueError("trajectory left joint count does not match")
        if len(right) != len(expected_right_joint_names):
            raise ValueError("trajectory right joint count does not match")
        if any(not math.isfinite(value) for value in (*left, *right)):
            raise ValueError("trajectory positions must be finite")
        left_gripper = raw.get("left_gripper")
        right_gripper = raw.get("right_gripper")
        grippers = tuple(
            None if value is None else float(value)
            for value in (left_gripper, right_gripper)
        )
        if any(
            value is not None
            and (not math.isfinite(value) or not 0.0 <= value <= 1000.0)
            for value in grippers
        ):
            raise ValueError("trajectory gripper values must be in 0..1000")
        samples.append(
            DualArmTrajectorySample(
                left_positions=left,
                right_positions=right,
                left_gripper=grippers[0],
                right_gripper=grippers[1],
            )
        )
    return timestamps, samples


def replay(
    robot: ArxDCanDualArm,
    *,
    timestamps: list[float],
    samples: list[DualArmTrajectorySample],
    velocity: float = 50.0,
    gripper_level: int = 5,
) -> None:
    """按记录时间戳逐点发送普通 PV 或 MIT 位置命令，不做插值。"""
    if not samples or len(timestamps) != len(samples):
        raise ValueError("timestamps and samples must have the same non-zero length")
    times = [float(value) for value in timestamps]
    if any(not math.isfinite(value) for value in times):
        raise ValueError("timestamps must be finite")
    if any(current <= previous for previous, current in zip(times, times[1:])):
        raise ValueError("timestamps must be strictly increasing")
    speed = _speed_percent(velocity)
    mode = str(robot.control_mode).strip().lower()
    if mode not in {"pv", "mit"}:
        raise RuntimeError(f"unsupported replay control mode: {robot.control_mode}")

    gripper_pairs = [
        (sample.left_gripper, sample.right_gripper) for sample in samples
    ]
    if any((left is None) != (right is None) for left, right in gripper_pairs):
        raise ValueError("trajectory gripper commands require both sides")
    if any(left is not None for left, _ in gripper_pairs) and not robot.has_grippers:
        raise RuntimeError("trajectory requires the dual-arm gripper pair")

    started = time.perf_counter()
    first_timestamp = times[0]
    previous_grippers: tuple[float, float] | None = None
    for timestamp, sample, grippers in zip(times, samples, gripper_pairs):
        scheduled_at = started + timestamp - first_timestamp
        remaining = scheduled_at - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)

        command = (
            robot.set_joint_pv
            if mode == "pv"
            else robot.set_joint_mit
        )
        command(
            left=sample.left_positions,
            right=sample.right_positions,
            velocity=speed,
        )

        left_gripper, right_gripper = grippers
        if left_gripper is not None and right_gripper is not None:
            current_grippers = (float(left_gripper), float(right_gripper))
            if current_grippers != previous_grippers:
                robot.set_grippers(
                    left=current_grippers[0],
                    right=current_grippers[1],
                    gripper_level=gripper_level,
                )
                previous_grippers = current_grippers


__all__ = [
    "DualArmTrajectorySample",
    "MAX_RECORDING_HZ",
    "load_trajectory",
    "record",
    "replay",
    "save_trajectory",
]
