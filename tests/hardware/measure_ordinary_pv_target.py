#!/usr/bin/env python3
"""真机诊断：记录一次完整双臂普通 PV 终点命令的14关节响应。"""
from __future__ import annotations

import argparse
import csv
import json
import math
import shlex
import shutil
import statistics
import subprocess
import time
from pathlib import Path

from arx_d_can import ArxDCanDualArm, SafetyState
from arx_d_can.examples.common import joint_degrees, speed_percent


JOINT_NAMES = tuple(
    [f"left/J{index}" for index in range(1, 8)]
    + [f"right/J{index}" for index in range(1, 8)]
)
UNSAFE_STATES = {
    SafetyState.FAULT,
    SafetyState.SAFE_HOLD,
    SafetyState.SAFE_STOP,
    SafetyState.DEGRADED,
}


def _matlab_batch_expression(csv_path: Path, output_dir: Path) -> str:
    script_dir = Path(__file__).resolve().parent

    def matlab_string(path: Path) -> str:
        return str(path.resolve()).replace("'", "''")

    return (
        f"addpath('{matlab_string(script_dir)}'); "
        "plot_ordinary_pv_target_matlab("
        f"'{matlab_string(csv_path)}', '{matlab_string(output_dir)}')"
    )


def _require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    invalid = tuple(
        item.role
        for item in health.motor_feedback
        if not (
            item.has_feedback
            and item.fresh
            and item.has_state
            and item.values_finite
        )
    )
    if health.state in UNSAFE_STATES or invalid or health.feedback_issue_count:
        detail = (
            health.last_operation_error
            or health.safety_reason
            or health.fault_reason
            or f"invalid feedback: {invalid}"
        )
        raise RuntimeError(f"unsafe Runtime state={health.state.name}: {detail}")


def _metrics(
    times: list[float],
    positions: list[tuple[float, ...]],
    velocities: list[tuple[float, ...]],
    start: tuple[float, ...],
    target: tuple[float, ...],
    tolerance: float,
    settled_elapsed: float | None,
) -> list[dict[str, float | int | str | None]]:
    result = []
    for joint, name in enumerate(JOINT_NAMES):
        q = [row[joint] for row in positions]
        dq = [row[joint] for row in velocities]
        frame_changes = [
            current - previous for previous, current in zip(q, q[1:])
        ]
        peak_index = max(range(len(dq)), key=lambda index: abs(dq[index]))
        first_tolerance = next(
            (
                elapsed
                for elapsed, value in zip(times, q, strict=True)
                if abs(value - target[joint]) <= tolerance
            ),
            None,
        )
        direction = 1.0 if target[joint] >= start[joint] else -1.0
        overshoot = max(
            0.0,
            max(direction * (value - target[joint]) for value in q),
        )
        hold_indices = (
            []
            if settled_elapsed is None
            else [
                index
                for index, elapsed in enumerate(times)
                if elapsed >= settled_elapsed
            ]
        )
        hold_q = [q[index] for index in hold_indices]
        hold_dq = [dq[index] for index in hold_indices]
        hold_changes = [
            q[index] - q[index - 1]
            for index in hold_indices
            if index > 0 and times[index - 1] >= settled_elapsed
        ]
        result.append(
            {
                "joint": name,
                "start_deg": math.degrees(start[joint]),
                "target_deg": math.degrees(target[joint]),
                "travel_deg": math.degrees(target[joint] - start[joint]),
                "peak_abs_velocity_deg_s": math.degrees(abs(dq[peak_index])),
                "peak_velocity_time_s": times[peak_index],
                "first_within_tolerance_s": first_tolerance,
                "overshoot_deg": math.degrees(overshoot),
                "final_error_deg": math.degrees(q[-1] - target[joint]),
                "position_peak_to_peak_deg": math.degrees(max(q) - min(q)),
                "max_inter_sample_change_deg": math.degrees(
                    max((abs(value) for value in frame_changes), default=0.0)
                ),
                "velocity_rms_deg_s": math.degrees(
                    math.sqrt(statistics.fmean(value * value for value in dq))
                ),
                "hold_samples": len(hold_q),
                "hold_position_mean_deg": (
                    math.degrees(statistics.fmean(hold_q)) if hold_q else None
                ),
                "hold_position_peak_to_peak_deg": (
                    math.degrees(max(hold_q) - min(hold_q)) if hold_q else None
                ),
                "hold_position_std_deg": (
                    math.degrees(statistics.pstdev(hold_q)) if hold_q else None
                ),
                "hold_error_abs_max_deg": (
                    math.degrees(
                        max(abs(value - target[joint]) for value in hold_q)
                    )
                    if hold_q else None
                ),
                "hold_velocity_abs_max_deg_s": (
                    math.degrees(max(abs(value) for value in hold_dq))
                    if hold_dq else None
                ),
                "hold_velocity_rms_deg_s": (
                    math.degrees(
                        math.sqrt(
                            statistics.fmean(value * value for value in hold_dq)
                        )
                    )
                    if hold_dq else None
                ),
                "hold_max_inter_sample_change_deg": (
                    math.degrees(max(abs(value) for value in hold_changes))
                    if hold_changes else None
                ),
            }
        )
    return result


def run(args: argparse.Namespace) -> dict[str, object]:
    left_target = joint_degrees(args.left)
    right_target = joint_degrees(args.right)
    target = left_target + right_target
    period = 1.0 / args.sample_hz
    tolerance = math.radians(args.position_tolerance_deg)
    velocity_tolerance = math.radians(args.velocity_tolerance_deg_s)
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    enabled = False
    cleanup_errors: list[str] = []
    times: list[float] = []
    sequences: list[int] = []
    source_timestamps_ns: list[int] = []
    motion_arrivals: list[bool] = []
    positions: list[tuple[float, ...]] = []
    velocities: list[tuple[float, ...]] = []
    try:
        robot.connect()
        _require_healthy(robot)
        limits = tuple(robot.get_joint_limits().values())
        for index, (value, limit) in enumerate(zip(target, limits, strict=True)):
            if not limit.min_angle_rad <= value <= limit.max_angle_rad:
                raise ValueError(f"{JOINT_NAMES[index]} target is outside product limits")
        initial = robot.read_state()
        start = tuple(initial.left.arm.positions) + tuple(initial.right.arm.positions)
        maximum_total_delta_degrees = math.degrees(max(
            abs(final - current)
            for current, final in zip(start, target, strict=True)
        ))
        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")
        enabled = True
        _require_healthy(robot)
        submitted = time.perf_counter()
        robot.set_joint_pv(
            left=left_target,
            right=right_target,
            velocity=args.velocity,
        )
        next_tick = submitted
        stable_since: float | None = None
        settled_elapsed: float | None = None
        timed_out = False
        missed_periods = 0
        duplicate_snapshots_skipped = 0
        previous_sequence = int(initial.sequence)
        while True:
            remaining = next_tick - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
            captured = time.perf_counter()
            if captured - next_tick >= period:
                missed_periods += 1
            state = robot.read_state()
            elapsed = captured - submitted
            sequence = int(state.sequence)
            if sequence == previous_sequence:
                duplicate_snapshots_skipped += 1
                if elapsed >= args.timeout:
                    timed_out = True
                    break
                next_tick += period
                continue
            previous_sequence = sequence
            q = tuple(state.left.arm.positions) + tuple(state.right.arm.positions)
            dq = tuple(state.left.arm.velocities) + tuple(state.right.arm.velocities)
            times.append(elapsed)
            sequences.append(sequence)
            source_timestamps_ns.append(int(state.timestamp_ns))
            motion_arrivals.append(bool(state.motion_arrived))
            positions.append(q)
            velocities.append(dq)
            if len(times) % 50 == 0:
                _require_healthy(robot)
            if settled_elapsed is None:
                if max(
                    abs(a - b) for a, b in zip(q, target, strict=True)
                ) <= tolerance and max(
                    abs(value) for value in dq
                ) <= velocity_tolerance:
                    stable_since = captured if stable_since is None else stable_since
                    if captured - stable_since >= args.stable_seconds:
                        settled_elapsed = elapsed
                else:
                    stable_since = None
            if settled_elapsed is not None and elapsed - settled_elapsed >= args.hold_seconds:
                break
            if elapsed >= args.timeout:
                timed_out = True
                break
            next_tick += period

        if not times:
            raise RuntimeError("采样期间没有收到新的 DDS RobotState")
        span = times[-1] - times[0] if len(times) > 1 else 0.0
        sequence_gaps = sum(
            max(0, current - previous - 1)
            for previous, current in zip(sequences, sequences[1:])
        )
        metrics = _metrics(
            times,
            positions,
            velocities,
            start,
            target,
            tolerance,
            settled_elapsed,
        )
        return {
            "motor_speed_percent": args.velocity,
            "sample_target_hz": args.sample_hz,
            "samples": len(times),
            "sample_hz": (len(times) - 1) / span if span > 0.0 else 0.0,
            "unique_feedback_hz": (
                (len(sequences) - 1) / span if span > 0.0 else 0.0
            ),
            "missed_periods": missed_periods,
            "duplicate_snapshots_skipped": duplicate_snapshots_skipped,
            "feedback_sequence_gaps": sequence_gaps,
            "settling_time_s": settled_elapsed,
            "timed_out": timed_out,
            "hold_seconds": args.hold_seconds,
            "start_deg": [math.degrees(value) for value in start],
            "target_deg": [math.degrees(value) for value in target],
            "maximum_total_delta_deg": maximum_total_delta_degrees,
            "joints": metrics,
            "fastest_joint": max(
                metrics, key=lambda item: float(item["peak_abs_velocity_deg_s"])
            )["joint"],
            "health": robot.get_health().state.name,
            "_samples": (
                times,
                source_timestamps_ns,
                sequences,
                motion_arrivals,
                positions,
                velocities,
            ),
        }
    finally:
        if enabled:
            try:
                robot.disable()
            except Exception as error:
                cleanup_errors.append(f"disable: {error}")
        try:
            robot.disconnect()
        except Exception as error:
            cleanup_errors.append(f"disconnect: {error}")
        if cleanup_errors:
            print("清理错误：" + "; ".join(cleanup_errors), flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True, help="左臂 J1..J7 目标角度，单位度")
    parser.add_argument("--right", required=True, help="右臂 J1..J7 目标角度，单位度")
    parser.add_argument("--velocity", type=speed_percent, default=50.0)
    parser.add_argument("--sample-hz", type=float, default=500.0)
    parser.add_argument("--position-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--velocity-tolerance-deg-s", type=float, default=1.0)
    parser.add_argument("--stable-seconds", type=float, default=0.25)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/ordinary_pv_target/ordinary_pv_target.json"),
    )
    parser.add_argument(
        "--matlab",
        action="store_true",
        help="采集完成后调用 MATLAB 生成 14 关节跟踪与抖动图",
    )
    parser.add_argument("--yes", action="store_true")
    return parser


def main(args: argparse.Namespace) -> None:
    positive_values = {
        "--sample-hz": args.sample_hz,
        "--position-tolerance-deg": args.position_tolerance_deg,
        "--velocity-tolerance-deg-s": args.velocity_tolerance_deg_s,
        "--stable-seconds": args.stable_seconds,
        "--hold-seconds": args.hold_seconds,
        "--timeout": args.timeout,
    }
    for name, value in positive_values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if args.sample_hz > 500.0:
        raise ValueError("--sample-hz must not exceed the 500 Hz DDS state rate")
    if not args.yes:
        input(
            "将使能双臂并执行指定普通 PV 目标；确认工作空间安全、急停可用后按回车..."
        )
    report = run(args)
    (
        times,
        source_timestamps_ns,
        sequences,
        motion_arrivals,
        positions,
        velocities,
    ) = report.pop("_samples")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "elapsed_s",
                "source_timestamp_ns",
                "sequence",
                "motion_arrived",
                "phase",
            ]
            + [f"{name}_position_rad" for name in JOINT_NAMES]
            + [f"{name}_velocity_rad_s" for name in JOINT_NAMES]
            + [f"{name}_position_deg" for name in JOINT_NAMES]
            + [f"{name}_velocity_deg_s" for name in JOINT_NAMES]
            + [f"{name}_target_deg" for name in JOINT_NAMES]
            + [f"{name}_error_deg" for name in JOINT_NAMES]
            + [f"{name}_frame_delta_deg" for name in JOINT_NAMES]
        )
        previous_q: tuple[float, ...] | None = None
        settled_elapsed = report["settling_time_s"]
        target = tuple(math.radians(value) for value in report["target_deg"])
        for elapsed, timestamp_ns, sequence, arrived, q, dq in zip(
            times,
            source_timestamps_ns,
            sequences,
            motion_arrivals,
            positions,
            velocities,
            strict=True,
        ):
            error = tuple(
                actual - expected
                for actual, expected in zip(q, target, strict=True)
            )
            frame_delta = (
                (0.0,) * len(q)
                if previous_q is None
                else tuple(
                    current - previous
                    for previous, current in zip(previous_q, q, strict=True)
                )
            )
            phase = (
                "hold"
                if settled_elapsed is not None and elapsed >= settled_elapsed
                else "tracking"
            )
            writer.writerow(
                (
                    elapsed,
                    timestamp_ns,
                    sequence,
                    int(arrived),
                    phase,
                    *q,
                    *dq,
                    *(math.degrees(value) for value in q),
                    *(math.degrees(value) for value in dq),
                    *report["target_deg"],
                    *(math.degrees(value) for value in error),
                    *(math.degrees(value) for value in frame_delta),
                )
            )
            previous_q = q
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    print("\n稳态抖动统计（到位后的 hold 阶段）：", flush=True)
    print("joint       p-p(deg)    std(deg)    max-step(deg)  max-|dq|(deg/s)")
    for joint in report["joints"]:
        values = (
            joint["hold_position_peak_to_peak_deg"],
            joint["hold_position_std_deg"],
            joint["hold_max_inter_sample_change_deg"],
            joint["hold_velocity_abs_max_deg_s"],
        )
        formatted = [
            "n/a" if value is None else f"{value:.6f}" for value in values
        ]
        print(
            f"{joint['joint']:<10} {formatted[0]:>10} {formatted[1]:>11} "
            f"{formatted[2]:>14} {formatted[3]:>17}",
            flush=True,
        )
    print(f"JSON: {args.output}", flush=True)
    print(f"CSV: {csv_path}", flush=True)
    matlab_output_dir = args.output.parent / "matlab"
    matlab_expression = _matlab_batch_expression(csv_path, matlab_output_dir)
    matlab_command = shlex.join(["matlab", "-batch", matlab_expression])
    print(f"MATLAB: {matlab_command}", flush=True)
    if args.matlab:
        matlab_executable = shutil.which("matlab")
        if matlab_executable is None:
            print(
                "未在 PATH 中找到 MATLAB；数据已保存，请安装/加载 MATLAB 后执行上面的命令。",
                flush=True,
            )
        else:
            subprocess.run(
                [matlab_executable, "-batch", matlab_expression],
                check=True,
            )


if __name__ == "__main__":
    main(build_parser().parse_args())
