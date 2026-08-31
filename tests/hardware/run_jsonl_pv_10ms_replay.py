#!/usr/bin/env python3
"""开发人员真机测试：读取100 Hz JSONL并每10 ms调用一次普通双臂PV。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import time

from arx_d_can import ArxDCanDualArm, SafetyState
from arx_d_can.examples.common import speed_percent


DEFAULT_INPUT = Path(
    "/home/ubuntu/vr-pico/logs/yunyi_v1_0_sessions/"
    "20260831_142859_923277_pid5143_control_100hz.jsonl"
)
PERIOD_S = 0.010
JOINT_COUNT_PER_ARM = 7


@dataclass(frozen=True, slots=True)
class ReplayFrame:
    line_number: int
    tick: int
    monotonic_ns: int
    left: tuple[float, ...]
    right: tuple[float, ...]
    originally_submitted: bool
    originally_accepted: bool


def _joint_values(value: object, *, line_number: int, side: str) -> tuple[float, ...]:
    if not isinstance(value, list):
        raise ValueError(
            f"line {line_number}: {side}.command_target_rad must be a list"
        )
    positions = tuple(float(item) for item in value)
    if len(positions) != JOINT_COUNT_PER_ARM:
        raise ValueError(
            f"line {line_number}: {side}.command_target_rad must contain "
            f"7 values, got {len(positions)}"
        )
    if any(not math.isfinite(item) for item in positions):
        raise ValueError(
            f"line {line_number}: {side}.command_target_rad contains "
            "a non-finite value"
        )
    return positions


def load_frames(path: Path) -> list[ReplayFrame]:
    """读取全部 control_tick 的 command_target_rad，不重采样或插值。"""
    frames: list[ReplayFrame] = []
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(
                    f"line {line_number}: invalid JSON: {error.msg}"
                ) from error
            if record.get("record_type") != "control_tick":
                continue
            try:
                left_record = record["left"]
                right_record = record["right"]
                tick = int(record["tick"])
                monotonic_ns = int(record["monotonic_ns"])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    f"line {line_number}: incomplete control_tick record"
                ) from error
            sdk = record.get("sdk") or {}
            frames.append(
                ReplayFrame(
                    line_number=line_number,
                    tick=tick,
                    monotonic_ns=monotonic_ns,
                    left=_joint_values(
                        left_record.get("command_target_rad"),
                        line_number=line_number,
                        side="left",
                    ),
                    right=_joint_values(
                        right_record.get("command_target_rad"),
                        line_number=line_number,
                        side="right",
                    ),
                    originally_submitted=bool(sdk.get("submitted")),
                    originally_accepted=bool(sdk.get("accepted")),
                )
            )
    if not frames:
        raise ValueError(f"no control_tick records found in {path}")
    for previous, current in zip(frames, frames[1:]):
        if current.tick <= previous.tick:
            raise ValueError(
                f"line {current.line_number}: tick must be strictly increasing"
            )
        if current.monotonic_ns <= previous.monotonic_ns:
            raise ValueError(
                f"line {current.line_number}: monotonic_ns must be strictly increasing"
            )
    return frames


def _all_positions(frame: ReplayFrame) -> tuple[float, ...]:
    return frame.left + frame.right


def _validate_product_limits(
    robot: ArxDCanDualArm,
    frames: list[ReplayFrame],
) -> None:
    limits = tuple(robot.get_joint_limits().items())
    if len(limits) != 14:
        raise RuntimeError(f"Runtime returned {len(limits)} limits; expected 14")
    for frame in frames:
        for (role, limit), position in zip(limits, _all_positions(frame)):
            if not limit.min_angle_rad <= position <= limit.max_angle_rad:
                raise ValueError(
                    f"line {frame.line_number} tick={frame.tick}: {role} "
                    f"position={position:.9f} rad is outside "
                    f"[{limit.min_angle_rad:.9f}, "
                    f"{limit.max_angle_rad:.9f}] rad"
                )


def _maximum_step(frames: list[ReplayFrame]) -> tuple[float, str]:
    roles = tuple(
        [f"left/J{index}" for index in range(1, 8)]
        + [f"right/J{index}" for index in range(1, 8)]
    )
    maximum = 0.0
    maximum_role = roles[0]
    for previous, current in zip(frames, frames[1:]):
        for role, before, after in zip(
            roles,
            _all_positions(previous),
            _all_positions(current),
        ):
            step = abs(after - before)
            if step > maximum:
                maximum = step
                maximum_role = role
    return maximum, maximum_role


def _require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    if health.state in {
        SafetyState.FAULT,
        SafetyState.SAFE_HOLD,
        SafetyState.SAFE_STOP,
    }:
        detail = (
            health.last_operation_error
            or health.safety_reason
            or health.fault_reason
            or "unknown Runtime error"
        )
        raise RuntimeError(f"Runtime state={health.state.name}: {detail}")


def _stream_frames(
    robot: ArxDCanDualArm,
    frames: list[ReplayFrame],
    *,
    velocity: float,
) -> dict[str, float | int]:
    call_durations: list[float] = []
    dispatch_lateness: list[float] = []
    missed_periods = 0
    started = time.perf_counter()
    last_dispatch = started
    for index, frame in enumerate(frames):
        scheduled = started + index * PERIOD_S
        remaining = scheduled - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        dispatch_started = time.perf_counter()
        lateness = max(0.0, dispatch_started - scheduled)
        dispatch_lateness.append(lateness)
        if lateness >= PERIOD_S:
            missed_periods += 1
        robot.set_joint_pv(
            left=frame.left,
            right=frame.right,
            velocity=velocity,
        )
        call_durations.append(time.perf_counter() - dispatch_started)
        last_dispatch = dispatch_started
        if index % 100 == 0:
            _require_healthy(robot)
    elapsed = time.perf_counter() - started
    dispatch_span = max(0.0, last_dispatch - started)
    return {
        "points": len(frames),
        "elapsed_s": elapsed,
        "dispatch_hz": (
            (len(frames) - 1) / dispatch_span
            if len(frames) > 1 and dispatch_span > 0.0
            else 0.0
        ),
        "missed_periods": missed_periods,
        "mean_call_us": statistics.fmean(call_durations) * 1e6,
        "max_call_us": max(call_durations) * 1e6,
        "max_dispatch_lateness_us": max(dispatch_lateness) * 1e6,
    }


def _print_summary(path: Path, frames: list[ReplayFrame]) -> None:
    recorded_duration_s = (
        frames[-1].monotonic_ns - frames[0].monotonic_ns
    ) / 1e9
    replay_duration_s = (len(frames) - 1) * PERIOD_S
    maximum_step, maximum_role = _maximum_step(frames)
    print(f"输入：{path}")
    print(
        f"control_tick={len(frames)}，原记录跨度={recorded_duration_s:.3f}s，"
        f"10ms回放计划跨度={replay_duration_s:.3f}s"
    )
    print(
        f"原记录 submitted={sum(frame.originally_submitted for frame in frames)}，"
        f"accepted={sum(frame.originally_accepted for frame in frames)}"
    )
    print(
        f"最大相邻目标步长={maximum_step:.6f} rad（{maximum_role}），"
        f"按10ms换算={maximum_step / PERIOD_S:.2f} rad/s"
    )
    print(
        "执行方式：逐点调用普通 set_joint_pv()，固定调度周期10ms；"
        "不调用轨迹接口，不插值，不跳点。"
    )


def main(args: argparse.Namespace) -> None:
    frames = load_frames(args.input)
    robot = ArxDCanDualArm(control_mode="pv")
    enabled = False
    try:
        _validate_product_limits(robot, frames)
        _print_summary(args.input, frames)
        print("全部双臂关节目标均为有限弧度值，并位于Runtime产品限位内")
        if args.inspect_only:
            return

        input(
            "确认工作空间安全、急停可用后按回车；"
            "程序将连接、使能并立即从第一点开始逐点发送..."
        )
        robot.connect()
        _require_healthy(robot)
        if not robot.enable():
            raise RuntimeError("whole-robot enable was not confirmed")
        enabled = True
        print(
            f"开始逐点发送{len(frames)}个普通PV目标："
            f"velocity={args.velocity:g}，周期=10ms"
        )
        result = _stream_frames(robot, frames, velocity=args.velocity)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        input("全部点已发送；观察结束后按回车失能并退出...")
    except KeyboardInterrupt:
        print("\n用户中断，正在安全失能并断开")
    finally:
        try:
            if enabled:
                robot.disable()
        finally:
            robot.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help=f"control_100hz JSONL；默认 {DEFAULT_INPUT}",
    )
    parser.add_argument(
        "--velocity",
        type=speed_percent,
        default=100.0,
        help="普通PV速度百分比，范围1～100，默认100",
    )
    parser.add_argument(
        "--inspect-only",
        action="store_true",
        help="只解析并检查轨迹，不连接或运动真机",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
