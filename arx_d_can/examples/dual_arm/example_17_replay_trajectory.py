#!/usr/bin/env python3
"""示例 17：安全回到双臂轨迹起点后，按原始时间戳原子回放。"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.driver import damiao_model_limits
from arx_d_can.examples.single_arm.common import positive_velocity_degrees
from arx_d_can.service_tools.dual_trajectory_recording import (
    DualArmTrajectorySample,
    _submit_raw_positions,
    interpolate_sample,
    load_trajectory,
    replay,
)


def _safe_positions(arm, values) -> tuple[float, ...]:
    """将录制反馈裁剪为该侧 Runtime 的软限位目标。"""
    margin = arm.config.soft_limit_margin
    result = []
    for value, joint in zip(values, arm.config.arm_joints):
        position_range, _, _ = damiao_model_limits(joint.model)
        lower = (
            -position_range if joint.lower_limit is None else joint.lower_limit
        ) + margin
        upper = (
            position_range if joint.upper_limit is None else joint.upper_limit
        ) - margin
        result.append(max(lower, min(upper, float(value))))
    return tuple(result)


def _safe_samples(robot, samples) -> tuple[list[DualArmTrajectorySample], int]:
    output = []
    clipped = 0
    for sample in samples:
        left = _safe_positions(robot.left, sample.left_positions)
        right = _safe_positions(robot.right, sample.right_positions)
        if any(
            not math.isclose(actual, safe, abs_tol=1e-12)
            for actual, safe in zip(
                (*sample.left_positions, *sample.right_positions),
                (*left, *right),
            )
        ):
            clipped += 1
        output.append(
            DualArmTrajectorySample(
                left_positions=left,
                right_positions=right,
                left_gripper=sample.left_gripper,
                right_gripper=sample.right_gripper,
            )
        )
    return output, clipped


def _move_to_start(
    robot: ArxDCanDualArm,
    target: DualArmTrajectorySample,
    *,
    start_velocity: float,
    velocity_limit: float | None,
    timeout: float,
    position_tolerance: float,
    velocity_tolerance: float,
    control_hz: float,
) -> None:
    state = robot.read_cached_state()
    current = DualArmTrajectorySample(
        left_positions=_safe_positions(robot.left, state.left.arm.positions),
        right_positions=_safe_positions(robot.right, state.right.arm.positions),
        left_gripper=target.left_gripper,
        right_gripper=target.right_gripper,
    )
    largest_move = max(
        *(
            abs(end - start)
            for start, end in zip(current.left_positions, target.left_positions)
        ),
        *(
            abs(end - start)
            for start, end in zip(current.right_positions, target.right_positions)
        ),
    )
    # 五次 smoothstep 的最大归一化速度为 1.875；按此放大时长可保证
    # 生成 reference 的峰值速度不超过 start_velocity。
    duration = 1.875 * largest_move / start_velocity
    started = time.perf_counter()
    tick = 0
    while True:
        elapsed = min(tick / control_hz, duration)
        progress = 1.0 if duration == 0.0 else elapsed / duration
        sample = interpolate_sample(
            current,
            target,
            progress=progress,
            mode="quintic",
        )
        remaining = started + elapsed - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        _submit_raw_positions(robot, sample, velocity_limit=velocity_limit)
        if elapsed >= duration:
            break
        captured_at = time.perf_counter()
        tick = max(tick + 1, math.floor((captured_at - started) * control_hz) + 1)

    deadline = time.monotonic() + timeout
    stable_since = None
    next_tick = time.perf_counter()
    while time.monotonic() < deadline:
        _submit_raw_positions(robot, target, velocity_limit=velocity_limit)
        state = robot.read_cached_state()
        position_error = max(
            *(
                abs(actual - expected)
                for actual, expected in zip(
                    state.left.arm.positions,
                    target.left_positions,
                )
            ),
            *(
                abs(actual - expected)
                for actual, expected in zip(
                    state.right.arm.positions,
                    target.right_positions,
                )
            ),
        )
        peak_velocity = max(
            *(abs(value) for value in state.left.arm.velocities),
            *(abs(value) for value in state.right.arm.velocities),
        )
        if (
            position_error <= position_tolerance
            and peak_velocity <= velocity_tolerance
        ):
            stable_since = time.monotonic() if stable_since is None else stable_since
            if time.monotonic() - stable_since >= 0.5:
                return
        else:
            stable_since = None
        next_tick += 1.0 / control_hz
        remaining = next_tick - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_tick = time.perf_counter()
    raise TimeoutError("双臂未能在超时前稳定到达轨迹起点")


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode=args.mode)
    timestamps, recorded = load_trajectory(
        args.input,
        expected_left_joint_names=robot.left.joint_names,
        expected_right_joint_names=robot.right.joint_names,
    )
    samples, clipped = _safe_samples(robot, recorded)
    first = samples[0]
    product_velocity_limit = min(
        joint.pv_vlim
        for arm in (robot.left, robot.right)
        for joint in arm.config.arm_joints
    )
    if args.mode == "pv" and args.pv_velocity_limit > product_velocity_limit:
        raise ValueError(
            "--pv-velocity-limit exceeds the product PV velocity limit "
            f"({math.degrees(product_velocity_limit):g} deg/s)"
        )
    if args.mode == "pv" and args.start_velocity > args.pv_velocity_limit:
        raise ValueError("--start-velocity cannot exceed --pv-velocity-limit")
    if args.mode == "mit" and args.start_velocity > math.radians(200.0):
        raise ValueError("MIT --start-velocity cannot exceed 200 deg/s")

    robot.connect()
    control_hz = robot._effective_control_hz
    print("机器人连接成功")
    try:
        robot.enable()
        print(
            f"正以 {math.degrees(args.start_velocity):g}°/s "
            "原子移动双臂到轨迹起点……"
        )
        _move_to_start(
            robot,
            first,
            start_velocity=args.start_velocity,
            velocity_limit=(args.pv_velocity_limit if args.mode == "pv" else None),
            timeout=args.start_timeout,
            position_tolerance=args.position_tolerance,
            velocity_tolerance=args.velocity_tolerance,
            control_hz=control_hz,
        )
        if clipped:
            print(f"已将 {clipped} 个越界采样点裁剪到双臂软命令限位")
        print(
            f"开始原子回放 {len(samples)} 个双臂轨迹点，"
            f"控制模式：{args.mode.upper()}，插值模式：{args.interpolation}；"
            "按 Ctrl+C 可安全停止"
        )
        replay(
            robot,
            timestamps=timestamps,
            samples=samples,
            interpolation=args.interpolation,
            velocity_limit=(args.pv_velocity_limit if args.mode == "pv" else None),
        )
        print("双臂轨迹回放完成")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.close()
        print("双臂已失能并断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="录制的双臂轨迹 JSON")
    parser.add_argument(
        "--mode",
        choices=("pv", "mit"),
        default="pv",
        help="raw 回放控制模式；默认 pv",
    )
    parser.add_argument(
        "--start-velocity",
        type=positive_velocity_degrees,
        default=math.radians(30.0),
        help="返回轨迹起点的统一速度，单位为度/秒；默认 30",
    )
    parser.add_argument(
        "--pv-velocity-limit",
        type=positive_velocity_degrees,
        default=math.radians(100.0),
        help="raw PV 协议统一速度上限，单位为度/秒；MIT 模式忽略；默认 100",
    )
    parser.add_argument(
        "--interpolation",
        choices=("none", "linear", "quintic"),
        default="quintic",
        help="回放插值：none=零阶保持，linear=线性，quintic=五次 S 曲线",
    )
    parser.add_argument("--start-timeout", type=float, default=30.0)
    parser.add_argument(
        "--position-tolerance",
        type=positive_velocity_degrees,
        default=math.radians(1.0),
        help="起点位置容差，单位为度；默认 1",
    )
    parser.add_argument(
        "--velocity-tolerance",
        type=positive_velocity_degrees,
        default=math.radians(2.0),
        help="起点速度容差，单位为度/秒；默认 2",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
