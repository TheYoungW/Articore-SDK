"""双臂关节与夹爪轨迹的录制、保存和回放。"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Literal, TYPE_CHECKING

if TYPE_CHECKING:
    from ..sdk.dual_arm import ArxDCanDualArm


FORMAT_VERSION = 1
InterpolationMode = Literal["none", "linear", "quintic"]
REPLAY_HZ = 100.0
DEFAULT_MIT_TARGET_VELOCITIES = (0.0,) * 7
DEFAULT_MIT_KP = (190.0, 190.0, 70.0, 125.0, 10.0, 22.0, 28.0)
DEFAULT_MIT_KD = (4.55, 4.5, 2.0, 2.9, 0.7, 0.89, 0.84)
DEFAULT_MIT_FEEDFORWARD_TORQUES = (0.0,) * 7


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
    interpolation: InterpolationMode = "quintic",
    max_speed_percent: float = 50.0,
    mit_target_velocities: tuple[float, ...] = DEFAULT_MIT_TARGET_VELOCITIES,
    mit_kp: tuple[float, ...] = DEFAULT_MIT_KP,
    mit_kd: tuple[float, ...] = DEFAULT_MIT_KD,
    mit_feedforward_torques: tuple[float, ...] = DEFAULT_MIT_FEEDFORWARD_TORQUES,
) -> None:
    """按应用层频率重采样；PV走步进位置路径，MIT保留raw参数。"""
    if not samples or len(timestamps) != len(samples):
        raise ValueError("timestamps and samples must have the same non-zero length")
    if interpolation not in {"none", "linear", "quintic"}:
        raise ValueError("interpolation must be none, linear, or quintic")
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        raise ValueError("timestamps must be strictly increasing")
    if any(
        sample.left_gripper is not None or sample.right_gripper is not None
        for sample in samples
    ) and not robot.has_grippers:
        raise RuntimeError("trajectory requires the dual-arm gripper pair")
    if robot.control_mode == "pv":
        if (
            not math.isfinite(max_speed_percent)
            or not 0.0 <= max_speed_percent <= 100.0
        ):
            raise ValueError("max_speed_percent must be in 0..100")
        robot.set_max_speed(max_speed_percent)
    replay_hz = REPLAY_HZ

    started = time.perf_counter()
    first_timestamp = timestamps[0]
    relative_timestamps = [value - first_timestamp for value in timestamps]
    duration = relative_timestamps[-1]
    tick = 0
    segment = 0
    while True:
        elapsed = min(tick / replay_hz, duration)
        while (
            segment + 1 < len(samples)
            and relative_timestamps[segment + 1] <= elapsed
        ):
            segment += 1
        if segment + 1 >= len(samples):
            sample = samples[-1]
        else:
            segment_duration = (
                relative_timestamps[segment + 1] - relative_timestamps[segment]
            )
            progress = (elapsed - relative_timestamps[segment]) / segment_duration
            sample = interpolate_sample(
                samples[segment],
                samples[segment + 1],
                progress=progress,
                mode=interpolation,
            )

        remaining = started + elapsed - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        _submit_positions(
            robot,
            sample,
            mit_target_velocities=mit_target_velocities,
            mit_kp=mit_kp,
            mit_kd=mit_kd,
            mit_feedforward_torques=mit_feedforward_torques,
        )
        if sample.left_gripper is not None or sample.right_gripper is not None:
            robot.set_grippers(
                left=0.0 if sample.left_gripper is None else sample.left_gripper,
                right=0.0 if sample.right_gripper is None else sample.right_gripper,
                gripper_level=5,
            )
        if elapsed >= duration:
            return
        captured_at = time.perf_counter()
        tick = max(tick + 1, math.floor((captured_at - started) * replay_hz) + 1)


def _submit_positions(
    robot: ArxDCanDualArm,
    sample: DualArmTrajectorySample,
    *,
    mit_target_velocities: tuple[float, ...] = DEFAULT_MIT_TARGET_VELOCITIES,
    mit_kp: tuple[float, ...] = DEFAULT_MIT_KP,
    mit_kd: tuple[float, ...] = DEFAULT_MIT_KD,
    mit_feedforward_torques: tuple[float, ...] = DEFAULT_MIT_FEEDFORWARD_TORQUES,
) -> None:
    """PV提交步进位置目标；MIT提交一帧显式动态参数。"""
    mode = robot.control_mode
    if mode == "pv":
        robot.set_joint_pv(
            left=sample.left_positions,
            right=sample.right_positions,
        )
    elif mode == "mit":
        robot.submit_raw_mit(
            left_positions=sample.left_positions,
            right_positions=sample.right_positions,
            left_velocities=mit_target_velocities,
            right_velocities=mit_target_velocities,
            kp=mit_kp,
            kd=mit_kd,
            left_feedforward_torques=mit_feedforward_torques,
            right_feedforward_torques=mit_feedforward_torques,
        )
    else:
        raise RuntimeError("trajectory replay requires PV or MIT mode")


def interpolate_sample(
    start: DualArmTrajectorySample,
    end: DualArmTrajectorySample,
    *,
    progress: float,
    mode: InterpolationMode,
) -> DualArmTrajectorySample:
    """在两条录制样本之间执行零阶、线性或五次 S 曲线插值。"""
    u = max(0.0, min(1.0, float(progress)))
    if mode == "none":
        alpha = 0.0 if u < 1.0 else 1.0
    elif mode == "linear":
        alpha = u
    elif mode == "quintic":
        alpha = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    else:
        raise ValueError("mode must be none, linear, or quintic")

    def vector(first, second) -> tuple[float, ...]:
        return tuple(
            float(left + (right - left) * alpha)
            for left, right in zip(first, second)
        )

    def optional(first: float | None, second: float | None) -> float | None:
        if first is None or second is None:
            return first if u < 1.0 else second
        return float(first + (second - first) * alpha)

    return DualArmTrajectorySample(
        left_positions=vector(start.left_positions, end.left_positions),
        right_positions=vector(start.right_positions, end.right_positions),
        left_gripper=optional(start.left_gripper, end.left_gripper),
        right_gripper=optional(start.right_gripper, end.right_gripper),
    )


__all__ = [
    "DEFAULT_MIT_FEEDFORWARD_TORQUES",
    "DEFAULT_MIT_KD",
    "DEFAULT_MIT_KP",
    "DEFAULT_MIT_TARGET_VELOCITIES",
    "DualArmTrajectorySample",
    "InterpolationMode",
    "interpolate_sample",
    "load_trajectory",
    "record",
    "replay",
    "save_trajectory",
]
