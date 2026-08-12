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
MAX_HZ = 500.0


@dataclass(slots=True, frozen=True)
class DualArmTrajectorySample:
    """一条双臂关节与可选夹爪命令。"""

    left_positions: tuple[float, ...]
    right_positions: tuple[float, ...]
    left_gripper: float | None
    right_gripper: float | None


def _frequency(value: float) -> float:
    hz = float(value)
    if not math.isfinite(hz) or not 0.0 < hz <= MAX_HZ:
        raise ValueError("hz must be finite, positive, and at most 500")
    return hz


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
    """按固定频率录制双臂反馈。"""
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
        state = robot.read_state()
        samples.append(
            DualArmTrajectorySample(
                left_positions=tuple(state.left.arm.positions),
                right_positions=tuple(state.right.arm.positions),
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
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
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
) -> None:
    """按照录制时间戳发送双臂轨迹。"""
    if not samples or len(timestamps) != len(samples):
        raise ValueError("timestamps and samples must have the same non-zero length")
    if any(sample.left_gripper is not None for sample in samples) and not robot.left.has_gripper:
        raise RuntimeError("trajectory requires an active left gripper")
    if any(sample.right_gripper is not None for sample in samples) and not robot.right.has_gripper:
        raise RuntimeError("trajectory requires an active right gripper")
    started = time.perf_counter()
    first_timestamp = timestamps[0]
    for timestamp, sample in zip(timestamps, samples):
        remaining = started + timestamp - first_timestamp - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        robot._submit_joint_positions(
            left=sample.left_positions,
            right=sample.right_positions,
        )
        if sample.left_gripper is not None or sample.right_gripper is not None:
            robot.set_gripper_openings(
                left=0.0 if sample.left_gripper is None else sample.left_gripper,
                right=0.0 if sample.right_gripper is None else sample.right_gripper,
            )


__all__ = [
    "DualArmTrajectorySample",
    "load_trajectory",
    "record",
    "replay",
    "save_trajectory",
]
