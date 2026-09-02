#!/usr/bin/env python3
"""真机采集左臂 Linear 三角形和 Circular 完整圆的实际 TCP 路径。"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path
from typing import Callable, Sequence

from arx_d_can import ArxDCanDualArm, SafetyState


TRIANGLE_SIDE_M = 0.14
TRIANGLE_CENTER = (0.403537, 0.231892, 0.381638, 0.0, -1.570796, 0.0)
CIRCLE_CENTER_YZ = (0.231892, 0.381638)
CIRCLE_RADIUS_M = 0.10
CIRCLE_START = (0.403537, 0.231892, 0.281638, 0.0, -1.570796, 0.0)
CIRCLE_VIA = (0.403537, 0.331892, 0.381638, 0.0, -1.570796, 0.0)
CIRCLE_END = (0.403537, 0.231892, 0.481638, 0.0, -1.570796, 0.0)
CIRCLE_RETURN_VIA = (0.403537, 0.131892, 0.381638, 0.0, -1.570796, 0.0)
UNSAFE_STATES = {
    SafetyState.FAULT,
    SafetyState.SAFE_HOLD,
    SafetyState.SAFE_STOP,
    SafetyState.DEGRADED,
}
CSV_FIELDS = (
    "motion",
    "phase",
    "elapsed_s",
    "sequence",
    "x_m",
    "y_m",
    "z_m",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "path_error_m",
    "max_joint_velocity_rad_s",
)


def _triangle_vertices() -> tuple[tuple[float, ...], ...]:
    x, y, z, roll, pitch, yaw = TRIANGLE_CENTER
    radius = TRIANGLE_SIDE_M / math.sqrt(3.0)
    inner_y = y - radius / 2.0
    orientation = (roll, pitch, yaw)
    return (
        (x, y + radius, z, *orientation),
        (x, inner_y, z + TRIANGLE_SIDE_M / 2.0, *orientation),
        (x, inner_y, z - TRIANGLE_SIDE_M / 2.0, *orientation),
    )


def _require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    if health.state in UNSAFE_STATES or health.feedback_issue_count:
        detail = (
            health.last_operation_error
            or health.safety_reason
            or health.fault_reason
            or f"feedback_issue_count={health.feedback_issue_count}"
        )
        raise RuntimeError(f"unsafe Runtime state={health.state.name}: {detail}")


def _rotation_from_rpy(pose: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = pose[3:]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _orientation_error(expected: Sequence[float], actual: Sequence[float]) -> float:
    expected_rotation = _rotation_from_rpy(expected)
    actual_rotation = _rotation_from_rpy(actual)
    trace = sum(
        expected_rotation[row][column] * actual_rotation[row][column]
        for row in range(3)
        for column in range(3)
    )
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.acos(cosine)


def _point_segment_distance(
    point: Sequence[float], start: Sequence[float], end: Sequence[float]
) -> float:
    delta = tuple(end[index] - start[index] for index in range(3))
    length_squared = sum(value * value for value in delta)
    if length_squared == 0.0:
        return math.dist(point[:3], start[:3])
    fraction = sum(
        (point[index] - start[index]) * delta[index] for index in range(3)
    ) / length_squared
    fraction = max(0.0, min(1.0, fraction))
    projection = tuple(
        start[index] + fraction * delta[index] for index in range(3)
    )
    return math.dist(point[:3], projection)


def _triangle_path_error(pose: Sequence[float]) -> float:
    vertices = _triangle_vertices()
    closed = vertices + vertices[:1]
    return min(
        _point_segment_distance(pose, start, end)
        for start, end in zip(closed[:-1], closed[1:], strict=True)
    )


def _circle_path_error(pose: Sequence[float]) -> float:
    center_y, center_z = CIRCLE_CENTER_YZ
    radial_error = math.hypot(pose[1] - center_y, pose[2] - center_z) - CIRCLE_RADIUS_M
    x_error = pose[0] - CIRCLE_START[0]
    return math.hypot(x_error, radial_error)


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _capture_motion(
    robot: ArxDCanDualArm,
    *,
    name: str,
    start_pose: Sequence[float],
    final_pose: Sequence[float],
    path_error: Callable[[Sequence[float]], float],
    submit: Callable[[], None],
    sample_hz: float,
    timeout_s: float,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    period_s = 1.0 / sample_hz
    submit()
    submitted = time.perf_counter()
    next_tick = submitted
    path_started = False
    path_start_elapsed: float | None = None
    missed_periods = 0
    samples: list[dict[str, object]] = []
    last_sequence: int | None = None
    unique_sequences = 0

    while True:
        remaining = next_tick - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        captured = time.perf_counter()
        if captured - next_tick >= period_s:
            missed_periods += 1
        pose_sample = robot.get_pose_sample("left")
        state = robot.read_state()
        elapsed = captured - submitted
        pose = tuple(pose_sample.values)
        maximum_velocity = max(abs(value) for value in state.left.arm.velocities)
        start_position_error = math.dist(pose[:3], start_pose[:3])
        start_orientation_error = _orientation_error(start_pose, pose)
        if not path_started and (
            start_position_error <= 0.005
            and start_orientation_error <= 0.035
            and maximum_velocity <= 0.05
        ):
            path_started = True
            path_start_elapsed = elapsed
        if pose_sample.sequence != last_sequence:
            unique_sequences += 1
            last_sequence = pose_sample.sequence
        samples.append(
            {
                "motion": name,
                "phase": "path" if path_started else "approach",
                "elapsed_s": elapsed,
                "sequence": pose_sample.sequence,
                "x_m": pose[0],
                "y_m": pose[1],
                "z_m": pose[2],
                "roll_rad": pose[3],
                "pitch_rad": pose[4],
                "yaw_rad": pose[5],
                "path_error_m": path_error(pose) if path_started else math.nan,
                "max_joint_velocity_rad_s": maximum_velocity,
            }
        )
        if len(samples) % 10 == 0:
            _require_healthy(robot)
        if state.motion_arrived:
            break
        if elapsed >= timeout_s:
            raise TimeoutError(f"{name} did not complete within {timeout_s:g}s")
        next_tick += period_s

    path_errors = [
        float(sample["path_error_m"])
        for sample in samples
        if sample["phase"] == "path"
    ]
    final = tuple(float(samples[-1][field]) for field in CSV_FIELDS[4:10])
    span = max(float(samples[-1]["elapsed_s"]), 1e-9)
    metrics = {
        "samples": len(samples),
        "sample_hz": (len(samples) - 1) / span,
        "unique_pose_feedback_hz": max(0, unique_sequences - 1) / span,
        "missed_periods": missed_periods,
        "total_elapsed_s": span,
        "inferred_path_start_s": path_start_elapsed,
        "approach_elapsed_s": path_start_elapsed,
        "path_samples": len(path_errors),
        "path_error_rms_mm": 1000.0 * math.sqrt(
            statistics.fmean(value * value for value in path_errors)
        ) if path_errors else None,
        "path_error_p95_mm": 1000.0 * _percentile(path_errors, 0.95)
        if path_errors else None,
        "path_error_max_mm": 1000.0 * max(path_errors) if path_errors else None,
        "final_position_error_mm": 1000.0 * math.dist(final[:3], final_pose[:3]),
        "final_orientation_error_rad": _orientation_error(final_pose, final),
    }
    return samples, metrics


def run(args: argparse.Namespace) -> tuple[list[dict[str, object]], dict[str, object]]:
    vertices = _triangle_vertices()
    ordered_vertices = (
        (vertices[0], vertices[2], vertices[1])
        if args.reverse_linear
        else vertices
    )
    triangle_path = ordered_vertices + ordered_vertices[:1]
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    enabled = False
    all_samples: list[dict[str, object]] = []
    report: dict[str, object] = {
        "sample_target_hz": args.sample_hz,
        "triangle": {
            "side_m": TRIANGLE_SIDE_M,
            "center_pose": TRIANGLE_CENTER,
            "poses": triangle_path,
            "speed_percent": args.linear_speed,
            "direction": "reverse" if args.reverse_linear else "forward",
        },
        "circle": {
            "radius_m": CIRCLE_RADIUS_M,
            "start_pose": CIRCLE_START,
            "via_pose": CIRCLE_VIA,
            "end_pose": CIRCLE_END,
            "return_via_pose": CIRCLE_RETURN_VIA,
            "speed_percent": 50.0,
        },
    }
    try:
        robot.connect()
        _require_healthy(robot)
        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")
        enabled = True

        robot.set_speed_percent(args.linear_speed)
        approach_samples, approach_metrics = _capture_motion(
            robot,
            name="linear_triangle_approach",
            start_pose=triangle_path[0],
            final_pose=triangle_path[0],
            path_error=lambda _pose: 0.0,
            submit=lambda: robot.move_linear(
                side="left", end_pose=triangle_path[0]
            ),
            sample_hz=args.sample_hz,
            timeout_s=args.timeout,
        )
        all_samples.extend(approach_samples)
        segment_metrics = []
        for index, (start, end) in enumerate(
            zip(triangle_path[:-1], triangle_path[1:], strict=True), start=1
        ):
            samples, metrics = _capture_motion(
                robot,
                name=f"linear_triangle_edge_{index}",
                start_pose=start,
                final_pose=end,
                path_error=_triangle_path_error,
                submit=lambda target=end: robot.move_linear(
                    side="left", end_pose=target
                ),
                sample_hz=args.sample_hz,
                timeout_s=args.timeout,
            )
            all_samples.extend(samples)
            segment_metrics.append(metrics)
        report["linear_triangle"] = {
            "approach": approach_metrics,
            "segments": segment_metrics,
        }
        total_elapsed = approach_metrics["total_elapsed_s"] + sum(
            metrics["total_elapsed_s"] for metrics in segment_metrics
        )
        maximum_error = max(
            metrics["path_error_max_mm"] or 0.0 for metrics in segment_metrics
        )
        print(
            f"Linear 完成：{total_elapsed:.3f}s，"
            f"路径最大误差={maximum_error:.2f}mm",
            flush=True,
        )

        if args.skip_circular:
            report["final_health"] = robot.get_health().state.name
            return all_samples, report

        robot.set_speed_percent(50.0)
        samples, outward_metrics = _capture_motion(
            robot,
            name="circular_outward",
            start_pose=CIRCLE_START,
            final_pose=CIRCLE_END,
            path_error=_circle_path_error,
            submit=lambda: robot.move_circular(
                side="left",
                start_pose=CIRCLE_START,
                via_pose=CIRCLE_VIA,
                end_pose=CIRCLE_END,
            ),
            sample_hz=args.sample_hz,
            timeout_s=args.timeout,
        )
        all_samples.extend(samples)
        samples, return_metrics = _capture_motion(
            robot,
            name="circular_return",
            start_pose=CIRCLE_END,
            final_pose=CIRCLE_START,
            path_error=_circle_path_error,
            submit=lambda: robot.move_circular(
                side="left",
                start_pose=CIRCLE_END,
                via_pose=CIRCLE_RETURN_VIA,
                end_pose=CIRCLE_START,
            ),
            sample_hz=args.sample_hz,
            timeout_s=args.timeout,
        )
        all_samples.extend(samples)
        report["circular_outward"] = outward_metrics
        report["circular_return"] = return_metrics
        report["final_health"] = robot.get_health().state.name
        print(
            "Circular 完成："
            f"第一段最大误差={outward_metrics['path_error_max_mm']:.2f}mm，"
            f"返回段最大误差={return_metrics['path_error_max_mm']:.2f}mm",
            flush=True,
        )
        return all_samples, report
    finally:
        try:
            robot.stop_motion()
        except Exception:
            pass
        if enabled:
            try:
                robot.disable()
            except Exception as error:
                print(f"失能失败：{error}", flush=True)
        try:
            robot.disconnect()
        except Exception as error:
            print(f"断开连接失败：{error}", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample-hz", type=float, default=100.0)
    parser.add_argument("--linear-speed", type=float, default=100.0)
    parser.add_argument("--reverse-linear", action="store_true")
    parser.add_argument("--skip-circular", action="store_true")
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(
            "artifacts/cartesian_tracking/left_linear100_circular50_20260902.json"
        ),
    )
    parser.add_argument("--yes", action="store_true")
    return parser


def main(args: argparse.Namespace) -> None:
    if args.sample_hz <= 0.0 or args.timeout <= 0.0:
        raise ValueError("sample-hz and timeout must be positive")
    if not 1.0 <= args.linear_speed <= 100.0:
        raise ValueError("linear-speed must be in [1, 100]")
    if not args.yes:
        scope = "，不执行圆弧" if args.skip_circular else "和10cm完整圆（速度50）"
        direction = "反向" if args.reverse_linear else "正向"
        input(
            f"将真机执行左臂14cm{direction}三角形"
            f"（速度{args.linear_speed:g}）{scope}；"
            "确认工作空间安全、急停可用后按回车..."
        )
    samples, report = run(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(samples)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    print(f"JSON: {args.output}", flush=True)
    print(f"CSV: {csv_path}", flush=True)


if __name__ == "__main__":
    main(build_parser().parse_args())
