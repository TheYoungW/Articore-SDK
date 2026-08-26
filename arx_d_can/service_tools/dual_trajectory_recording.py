"""双臂关节与夹爪轨迹的录制、保存和回放。"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Literal, TYPE_CHECKING

from .._motor_abi import TrajectoryState, TrajectoryStatus

if TYPE_CHECKING:
    from ..sdk.dual_arm import ArxDCanDualArm


FORMAT_VERSION = 1
InterpolationMode = Literal["quintic"]
DEFAULT_MIT_KP = (190.0, 190.0, 70.0, 125.0, 10.0, 22.0, 28.0)
DEFAULT_MIT_KD = (4.55, 4.5, 2.0, 2.9, 0.7, 0.89, 0.84)
DEFAULT_MIT_FEEDFORWARD_TORQUES = (0.0,) * 7
DEFAULT_PV_VELOCITY_LIMITS = (2.5,) * 7


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
    mit_kp: tuple[float, ...] = DEFAULT_MIT_KP,
    mit_kd: tuple[float, ...] = DEFAULT_MIT_KD,
    mit_feedforward_torques: tuple[float, ...] = DEFAULT_MIT_FEEDFORWARD_TORQUES,
    pv_velocity_limits: tuple[float, ...] = DEFAULT_PV_VELOCITY_LIMITS,
    gripper_level: int = 5,
    timeout: float | None = None,
) -> TrajectoryStatus:
    """一次性提交原生轨迹，并仅轮询状态等待 C++ 500 Hz 执行完成。"""
    if not samples or len(timestamps) != len(samples):
        raise ValueError("timestamps and samples must have the same non-zero length")
    if interpolation != "quintic":
        raise ValueError("native trajectory replay only supports quintic interpolation")
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        raise ValueError("timestamps must be strictly increasing")
    gripper_pairs = {
        (sample.left_gripper, sample.right_gripper) for sample in samples
        if sample.left_gripper is not None or sample.right_gripper is not None
    }
    if gripper_pairs:
        if not robot.has_grippers:
            raise RuntimeError("trajectory requires the dual-arm gripper pair")
        if len(gripper_pairs) != 1:
            raise ValueError(
                "native arm trajectories do not support time-varying gripper commands"
            )
        left_gripper, right_gripper = next(iter(gripper_pairs))
        if left_gripper is None or right_gripper is None:
            raise ValueError("native trajectory gripper commands require both sides")
        robot.set_grippers(
            left=left_gripper,
            right=right_gripper,
            gripper_level=gripper_level,
        )

    relative_timestamps = [value - timestamps[0] for value in timestamps]
    robot.start_trajectory(
        timestamps=relative_timestamps,
        left_positions=[sample.left_positions for sample in samples],
        right_positions=[sample.right_positions for sample in samples],
        interpolation="quintic",
        kp=mit_kp if robot.control_mode == "mit" else None,
        kd=mit_kd if robot.control_mode == "mit" else None,
        feedforward_torque=(
            mit_feedforward_torques if robot.control_mode == "mit" else None
        ),
        pv_velocity_limits=(
            pv_velocity_limits if robot.control_mode == "pv" else 2.5
        ),
    )
    deadline = (
        time.monotonic() + timeout
        if timeout is not None
        else time.monotonic() + relative_timestamps[-1] + 30.0
    )
    while True:
        status = robot.trajectory_status
        if status.state is TrajectoryState.COMPLETED:
            return status
        if status.state in {TrajectoryState.CANCELLED, TrajectoryState.FAULT}:
            raise RuntimeError(status.error or f"trajectory {status.state.value}")
        if time.monotonic() >= deadline:
            robot.cancel_trajectory()
            raise TimeoutError("native trajectory did not complete before timeout")
        time.sleep(0.02)


__all__ = [
    "DEFAULT_MIT_FEEDFORWARD_TORQUES",
    "DEFAULT_MIT_KD",
    "DEFAULT_MIT_KP",
    "DEFAULT_PV_VELOCITY_LIMITS",
    "DualArmTrajectorySample",
    "InterpolationMode",
    "load_trajectory",
    "record",
    "replay",
    "save_trajectory",
]
