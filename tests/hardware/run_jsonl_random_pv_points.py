#!/usr/bin/env python3
"""真机测试：从100 Hz JSONL随机抽点，普通PV逐点运动并每次返回首帧。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import time

from arx_d_can import ArxDCanDualArm, SafetyState
from arx_d_can.examples.common import speed_percent
from run_jsonl_pv_10ms_replay import ReplayFrame, load_frames


DEFAULT_INPUT = Path(
    "/home/ubuntu/vr-pico/logs/yunyi_v1_0_sessions/"
    "20260831_142859_923277_pid5143_control_100hz.jsonl"
)
JOINT_ROLES = tuple(
    [f"left/J{index}" for index in range(1, 8)]
    + [f"right/J{index}" for index in range(1, 8)]
)
UNSAFE_STATES = {
    SafetyState.FAULT,
    SafetyState.SAFE_HOLD,
    SafetyState.SAFE_STOP,
    SafetyState.DEGRADED,
}


def _positive_int(text: str) -> int:
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("value must be a positive integer")
    return value


def _positive_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return value


def _all_positions(frame: ReplayFrame) -> tuple[float, ...]:
    return frame.left + frame.right


def _select_frames(
    frames: list[ReplayFrame], *, count: int, seed: int
) -> list[ReplayFrame]:
    candidates = [frame for frame in frames[1:] if frame.originally_accepted]
    if count > len(candidates):
        raise ValueError(
            f"cannot select {count} unique points from {len(candidates)} candidates"
        )
    return random.Random(seed).sample(candidates, count)


def _validate_product_limits(
    robot: ArxDCanDualArm,
    frames: list[ReplayFrame],
) -> None:
    limits = tuple(robot.get_joint_limits().items())
    if len(limits) != len(JOINT_ROLES):
        raise RuntimeError(f"Runtime returned {len(limits)} limits; expected 14")
    for frame in frames:
        for (role, limit), position in zip(limits, _all_positions(frame)):
            if not limit.min_angle_rad <= position <= limit.max_angle_rad:
                raise ValueError(
                    f"line {frame.line_number} tick={frame.tick}: {role} "
                    f"position={position:.9f} rad is outside "
                    f"[{limit.min_angle_rad:.9f}, {limit.max_angle_rad:.9f}] rad"
                )


def _require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    invalid_feedback = tuple(
        item.role
        for item in health.motor_feedback
        if not (
            item.has_feedback
            and item.fresh
            and item.has_state
            and item.values_finite
        )
    )
    if (
        health.state in UNSAFE_STATES
        or invalid_feedback
        or health.feedback_issue_count
    ):
        detail = (
            health.last_operation_error
            or health.safety_reason
            or health.fault_reason
            or f"invalid feedback: {invalid_feedback}"
        )
        raise RuntimeError(f"unsafe Runtime state={health.state.name}: {detail}")


def _maximum_origin_delta(
    origin: ReplayFrame,
    selected: list[ReplayFrame],
) -> tuple[float, str, ReplayFrame]:
    maximum = -1.0
    maximum_role = JOINT_ROLES[0]
    maximum_frame = selected[0]
    origin_positions = _all_positions(origin)
    for frame in selected:
        for role, home, target in zip(
            JOINT_ROLES, origin_positions, _all_positions(frame)
        ):
            delta = abs(target - home)
            if delta > maximum:
                maximum = delta
                maximum_role = role
                maximum_frame = frame
    return maximum, maximum_role, maximum_frame


def _wait_target(
    robot: ArxDCanDualArm,
    frame: ReplayFrame,
    *,
    timeout_s: float,
    stable_s: float,
    position_tolerance: float,
    velocity_tolerance: float,
) -> dict[str, float | int]:
    started = time.monotonic()
    deadline = started + timeout_s
    stable_since: float | None = None
    peak_velocity = 0.0
    max_error = math.inf
    samples = 0
    while time.monotonic() < deadline:
        _require_healthy(robot)
        state = robot.read_state()
        positions = state.left.arm.positions + state.right.arm.positions
        velocities = state.left.arm.velocities + state.right.arm.velocities
        max_error = max(
            abs(actual - target)
            for actual, target in zip(positions, _all_positions(frame))
        )
        current_peak_velocity = max(abs(value) for value in velocities)
        peak_velocity = max(peak_velocity, current_peak_velocity)
        samples += 1
        now = time.monotonic()
        if (
            max_error <= position_tolerance
            and current_peak_velocity <= velocity_tolerance
        ):
            stable_since = now if stable_since is None else stable_since
            if now - stable_since >= stable_s:
                return {
                    "elapsed_s": now - started,
                    "final_max_error_deg": math.degrees(max_error),
                    "peak_velocity_deg_s": math.degrees(peak_velocity),
                    "samples": samples,
                }
        else:
            stable_since = None
        time.sleep(0.01)
    raise TimeoutError(
        f"tick={frame.tick} did not settle within {timeout_s:g}s; "
        f"last max error={math.degrees(max_error):.3f} deg"
    )


def _move_to_frame(
    robot: ArxDCanDualArm,
    frame: ReplayFrame,
    *,
    velocity: float,
    timeout_s: float,
    stable_s: float,
    position_tolerance: float,
    velocity_tolerance: float,
) -> dict[str, float | int]:
    robot.set_joint_pv(left=frame.left, right=frame.right, velocity=velocity)
    return _wait_target(
        robot,
        frame,
        timeout_s=timeout_s,
        stable_s=stable_s,
        position_tolerance=position_tolerance,
        velocity_tolerance=velocity_tolerance,
    )


def _plan_summary(
    path: Path,
    frames: list[ReplayFrame],
    origin: ReplayFrame,
    selected: list[ReplayFrame],
    *,
    seed: int,
) -> dict[str, object]:
    maximum, role, maximum_frame = _maximum_origin_delta(origin, selected)
    summary: dict[str, object] = {
        "input": str(path),
        "available_control_ticks": len(frames),
        "accepted_candidate_ticks": sum(
            frame.originally_accepted for frame in frames[1:]
        ),
        "origin": {
            "line_number": origin.line_number,
            "tick": origin.tick,
            "left": origin.left,
            "right": origin.right,
        },
        "random_seed": seed,
        "selected_count": len(selected),
        "selected_ticks": [frame.tick for frame in selected],
        "maximum_origin_delta_deg": math.degrees(maximum),
        "maximum_origin_delta_role": role,
        "maximum_origin_delta_tick": maximum_frame.tick,
    }
    print(f"输入：{path}")
    print(
        f"有效control_tick={len(frames)}，其中accepted候选="
        f"{sum(frame.originally_accepted for frame in frames[1:])}；"
        f"原点=首帧tick={origin.tick}；"
        f"随机种子={seed}；抽取={len(selected)}个不重复点"
    )
    print(
        f"随机点相对原点的最大单关节位移="
        f"{math.degrees(maximum):.2f}°（{role}, tick={maximum_frame.tick}）"
    )
    print("随机tick：" + ",".join(str(frame.tick) for frame in selected))
    return summary


def run(args: argparse.Namespace) -> dict[str, object]:
    frames = load_frames(args.input)
    origin = frames[0]
    selected = _select_frames(frames, count=args.count, seed=args.seed)
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    _validate_product_limits(robot, [origin, *selected])
    report = _plan_summary(
        args.input, frames, origin, selected, seed=args.seed
    )
    report["velocity_percent"] = args.velocity
    if args.inspect_only:
        return report
    if not args.yes:
        input(
            f"将以普通PV {args.velocity:g}%执行{args.count}个双臂随机姿态，"
            "每个姿态后返回JSONL首帧原点。运动范围可能较大；确认工作空间安全、"
            "人员远离、急停可用后按回车..."
        )

    enabled = False
    cleanup_errors: list[str] = []
    results: list[dict[str, object]] = []
    try:
        robot.connect()
        _require_healthy(robot)
        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")
        enabled = True

        print(f"先移动到首帧原点，普通PV速度={args.velocity:g}%", flush=True)
        report["initial_origin"] = _move_to_frame(
            robot,
            origin,
            velocity=args.velocity,
            timeout_s=args.timeout,
            stable_s=args.stable,
            position_tolerance=math.radians(args.position_tolerance_deg),
            velocity_tolerance=math.radians(args.velocity_tolerance_deg_s),
        )
        time.sleep(args.hold)

        for index, frame in enumerate(selected, start=1):
            print(
                f"[{index:02d}/{len(selected)}] 移动到随机点 tick={frame.tick}",
                flush=True,
            )
            target_result = _move_to_frame(
                robot,
                frame,
                velocity=args.velocity,
                timeout_s=args.timeout,
                stable_s=args.stable,
                position_tolerance=math.radians(args.position_tolerance_deg),
                velocity_tolerance=math.radians(args.velocity_tolerance_deg_s),
            )
            time.sleep(args.hold)
            print(
                f"[{index:02d}/{len(selected)}] 返回首帧原点",
                flush=True,
            )
            origin_result = _move_to_frame(
                robot,
                origin,
                velocity=args.velocity,
                timeout_s=args.timeout,
                stable_s=args.stable,
                position_tolerance=math.radians(args.position_tolerance_deg),
                velocity_tolerance=math.radians(args.velocity_tolerance_deg_s),
            )
            time.sleep(args.hold)
            results.append(
                {
                    "index": index,
                    "line_number": frame.line_number,
                    "tick": frame.tick,
                    "target": target_result,
                    "return_to_origin": origin_result,
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
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--count", type=_positive_int, default=50)
    parser.add_argument("--seed", type=int, default=20260831)
    parser.add_argument(
        "--velocity",
        type=speed_percent,
        default=100.0,
        help="普通PV速度百分比，范围1～100，默认100",
    )
    parser.add_argument("--timeout", type=_positive_float, default=30.0)
    parser.add_argument("--stable", type=_positive_float, default=0.5)
    parser.add_argument("--hold", type=_positive_float, default=0.5)
    parser.add_argument(
        "--position-tolerance-deg", type=_positive_float, default=1.0
    )
    parser.add_argument(
        "--velocity-tolerance-deg-s", type=_positive_float, default=2.0
    )
    parser.add_argument("--inspect-only", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/jsonl_random_50_pv_points.json"),
    )
    return parser


def main(args: argparse.Namespace) -> None:
    report = run(args)
    if args.inspect_only:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"全部随机点完成，结果：{args.output.resolve()}", flush=True)


if __name__ == "__main__":
    main(build_parser().parse_args())
