#!/usr/bin/env python3
"""真机诊断：逐关节用普通 PV 阶跃比较不同速度百分比的响应。"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path

from arx_d_can import ArxDCanDualArm, SafetyState


SAMPLE_HZ = 500.0
UNSAFE_STATES = {
    SafetyState.FAULT,
    SafetyState.SAFE_HOLD,
    SafetyState.SAFE_STOP,
    SafetyState.DEGRADED,
}


def _speeds(text: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if not values or any(
        not math.isfinite(value) or not 1.0 <= value <= 100.0
        for value in values
    ):
        raise argparse.ArgumentTypeError("speeds must be finite values in 1..100")
    return values


def _joints(text: str) -> tuple[int, ...]:
    try:
        values = tuple(int(value) for value in text.split(",") if value.strip())
    except ValueError as error:
        raise argparse.ArgumentTypeError("joints must be comma-separated integers") from error
    if not values or len(set(values)) != len(values) or any(
        not 1 <= value <= 7 for value in values
    ):
        raise argparse.ArgumentTypeError(
            "joints must be unique comma-separated values in 1..7"
        )
    return values


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


def _preflight(robot: ArxDCanDualArm, seconds: float) -> dict[str, object]:
    started = time.monotonic()
    samples = 0
    left_j1_updates: list[int] = []
    fps: list[float] = []
    while time.monotonic() - started < seconds:
        robot.read_state()
        _require_healthy(robot)
        health = robot.get_health()
        left_j1 = next(
            item
            for item in health.motor_feedback
            if item.side == 0 and item.can_id == 1 and not item.is_gripper
        )
        left_j1_updates.append(left_j1.update_count)
        fps.append(robot.get_fps())
        samples += 1
        time.sleep(0.01)
    if left_j1_updates[-1] <= left_j1_updates[0]:
        raise RuntimeError("left J1 feedback update_count did not advance")
    return {
        "seconds": time.monotonic() - started,
        "samples": samples,
        "left_j1_update_delta": left_j1_updates[-1] - left_j1_updates[0],
        "can_fps_mean": statistics.fmean(fps),
        "can_fps_min": min(fps),
    }


def _measure_step(
    robot: ArxDCanDualArm,
    *,
    left: tuple[float, ...],
    right: tuple[float, ...],
    joint_index: int,
    target: float,
    start: float,
    speed: float,
    timeout_s: float,
    hold_s: float,
    position_tolerance: float,
    velocity_tolerance: float,
) -> dict[str, object]:
    robot.set_joint_pv(left=left, right=right, velocity=speed)
    period_s = 1.0 / SAMPLE_HZ
    started = time.perf_counter()
    next_tick = started
    stable_since: float | None = None
    settled_elapsed: float | None = None
    times: list[float] = []
    positions: list[float] = []
    velocities: list[float] = []
    sequences: list[int] = []
    missed_periods = 0
    direction = 1.0 if target >= start else -1.0
    while True:
        remaining = next_tick - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        captured = time.perf_counter()
        if captured - next_tick >= period_s:
            missed_periods += 1
        state = robot.read_state()
        elapsed = captured - started
        position = float(state.right.arm.positions[joint_index])
        velocity = float(state.right.arm.velocities[joint_index])
        times.append(elapsed)
        positions.append(position)
        velocities.append(velocity)
        sequences.append(int(state.sequence))
        if len(times) % 50 == 0:
            _require_healthy(robot)
        if settled_elapsed is None:
            if (
                abs(position - target) <= position_tolerance
                and abs(velocity) <= velocity_tolerance
            ):
                stable_since = captured if stable_since is None else stable_since
                if captured - stable_since >= 0.25:
                    settled_elapsed = elapsed
            else:
                stable_since = None
            if elapsed >= timeout_s:
                raise TimeoutError(
                    f"PV speed {speed:g}% did not settle within {timeout_s:g}s"
                )
        elif elapsed - settled_elapsed >= hold_s:
            break
        next_tick += period_s

    amplitude = abs(target - start)
    ten_percent_error = amplitude * 0.1
    rise_time = next(
        (
            elapsed
            for elapsed, position in zip(times, positions)
            if abs(target - position) <= ten_percent_error
        ),
        None,
    )
    signed_overshoot = [direction * (position - target) for position in positions]
    errors = [position - target for position in positions]
    hold_indices = [
        index for index, elapsed in enumerate(times) if elapsed >= settled_elapsed
    ]
    hold_positions = [positions[index] for index in hold_indices]
    hold_velocities = [velocities[index] for index in hold_indices]
    hold_errors = [position - target for position in hold_positions]
    span = times[-1] - times[0] if len(times) > 1 else 0.0
    return {
        "speed_percent": speed,
        "start_deg": math.degrees(start),
        "target_deg": math.degrees(target),
        "settling_time_s": settled_elapsed,
        "rise_time_90_s": rise_time,
        "peak_velocity_rad_s": max(abs(value) for value in velocities),
        "peak_velocity_deg_s": math.degrees(max(abs(value) for value in velocities)),
        "target_error_abs_max_deg": math.degrees(max(abs(value) for value in errors)),
        "target_error_rms_deg": math.degrees(
            math.sqrt(statistics.fmean(value * value for value in errors))
        ),
        "overshoot_deg": math.degrees(max(0.0, max(signed_overshoot))),
        "final_error_deg": math.degrees(errors[-1]),
        "hold_duration_s": times[-1] - settled_elapsed,
        "hold_position_peak_to_peak_deg": math.degrees(
            max(hold_positions) - min(hold_positions)
        ),
        "hold_position_std_deg": math.degrees(statistics.pstdev(hold_positions)),
        "hold_error_abs_max_deg": math.degrees(
            max(abs(value) for value in hold_errors)
        ),
        "hold_error_rms_deg": math.degrees(
            math.sqrt(statistics.fmean(value * value for value in hold_errors))
        ),
        "hold_velocity_abs_max_deg_s": math.degrees(
            max(abs(value) for value in hold_velocities)
        ),
        "hold_velocity_rms_deg_s": math.degrees(
            math.sqrt(statistics.fmean(value * value for value in hold_velocities))
        ),
        "hold_unique_position_values": len(set(hold_positions)),
        "samples": len(times),
        "sample_hz": (len(times) - 1) / span if span > 0.0 else 0.0,
        "unique_feedback_hz": (
            (len(set(sequences)) - 1) / span if span > 0.0 else 0.0
        ),
        "missed_periods": missed_periods,
    }


def run(args: argparse.Namespace) -> dict[str, object]:
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    enabled = False
    cleanup_errors: list[str] = []
    report: dict[str, object] = {
        "joints": [f"right/J{joint}" for joint in args.joints],
        "step_deg": args.step_deg,
        "speeds_percent": list(args.speeds),
        "sample_target_hz": SAMPLE_HZ,
    }
    try:
        robot.connect()
        report["preflight"] = _preflight(robot, args.preflight_seconds)
        if args.preflight_only:
            return report
        initial = robot.read_state()
        baseline_left = tuple(initial.left.arm.positions)
        baseline_right = tuple(initial.right.arm.positions)
        step = math.radians(args.step_deg)
        joint_limits = robot.get_joint_limits()

        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")
        enabled = True
        robot.set_joint_pv(
            left=baseline_left,
            right=baseline_right,
            velocity=min(args.speeds),
        )
        time.sleep(0.5)
        _require_healthy(robot)

        results: list[dict[str, object]] = []
        for joint in args.joints:
            joint_index = joint - 1
            joint_name = f"right/J{joint}"
            limits = joint_limits[f"r-joint{joint}"]
            start = baseline_right[joint_index]
            candidate = start + step
            if candidate > limits.max_angle_rad - math.radians(1.0):
                candidate = start - step
            if candidate < limits.min_angle_rad + math.radians(1.0):
                raise RuntimeError(
                    f"{joint_name} has insufficient margin for the requested step"
                )
            target_right_values = list(baseline_right)
            target_right_values[joint_index] = candidate
            target_right = tuple(target_right_values)
            speed_results: list[dict[str, object]] = []
            for speed in args.speeds:
                print(f"测试普通 PV {speed:g}%：{joint_name} 外移", flush=True)
                outbound = _measure_step(
                    robot,
                    left=baseline_left,
                    right=target_right,
                    joint_index=joint_index,
                    target=candidate,
                    start=start,
                    speed=speed,
                    timeout_s=args.timeout,
                    hold_s=args.hold,
                    position_tolerance=math.radians(args.position_tolerance_deg),
                    velocity_tolerance=math.radians(args.velocity_tolerance_deg_s),
                )
                print(f"测试普通 PV {speed:g}%：{joint_name} 返回", flush=True)
                returned = _measure_step(
                    robot,
                    left=baseline_left,
                    right=baseline_right,
                    joint_index=joint_index,
                    target=start,
                    start=candidate,
                    speed=speed,
                    timeout_s=args.timeout,
                    hold_s=args.hold,
                    position_tolerance=math.radians(args.position_tolerance_deg),
                    velocity_tolerance=math.radians(args.velocity_tolerance_deg_s),
                )
                speed_results.append(
                    {
                        "speed_percent": speed,
                        "outbound": outbound,
                        "return": returned,
                    }
                )
            results.append(
                {
                    "joint": joint_name,
                    "baseline_deg": math.degrees(start),
                    "target_deg": math.degrees(candidate),
                    "speeds": speed_results,
                }
            )
        report["results"] = results
        report["final_health"] = robot.get_health().state.name
        return report
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
    parser.add_argument("--speeds", type=_speeds, default=(25.0, 50.0, 75.0, 100.0))
    parser.add_argument("--joints", type=_joints, default=(1, 2, 4))
    parser.add_argument("--step-deg", type=float, default=10.0)
    parser.add_argument("--preflight-seconds", type=float, default=10.0)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--position-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--velocity-tolerance-deg-s", type=float, default=1.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/ordinary_pv_right_joints_speed_sweep.json"),
    )
    parser.add_argument("--yes", action="store_true")
    return parser


def main(args: argparse.Namespace) -> None:
    if not math.isfinite(args.step_deg) or args.step_deg <= 0.0:
        raise ValueError("--step-deg must be a finite positive value")
    if not args.preflight_only and not args.yes:
        joint_names = ", ".join(f"J{joint}" for joint in args.joints)
        input(
            f"将用普通 PV 让右臂 {joint_names} 依次往返约 {args.step_deg:g}°；"
            "确认工作空间安全、急停可用后按回车..."
        )
    report = run(args)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if not args.preflight_only:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"结果：{args.output.resolve()}")


if __name__ == "__main__":
    main(build_parser().parse_args())
