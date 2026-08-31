#!/usr/bin/env python3
"""控制示例 14：回到起点后，按时间戳逐点发送普通 PV 或 MIT 轨迹。"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import (
    positive_velocity_degrees,
    speed_percent,
)
from arx_d_can.service_tools.dual_trajectory_recording import (
    DualArmTrajectorySample,
    load_trajectory,
    replay,
)


def _move_to_start(
    robot: ArxDCanDualArm,
    target: DualArmTrajectorySample,
    *,
    velocity: float,
    timeout: float,
    position_tolerance: float,
    velocity_tolerance: float,
) -> None:
    if robot.control_mode == "pv":
        robot.set_joint_pv(
            left=target.left_positions,
            right=target.right_positions,
            velocity=velocity,
        )
    else:
        robot.set_joint_mit(
            left=target.left_positions,
            right=target.right_positions,
        )

    deadline = time.monotonic() + timeout
    stable_since = None
    next_tick = time.perf_counter()
    while time.monotonic() < deadline:
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
        next_tick += 0.02
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
        expected_left_joint_names=robot.joint_names,
        expected_right_joint_names=robot.joint_names,
    )
    samples = recorded
    first = samples[0]
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        detail = f"，速度 {args.velocity:g}%" if args.mode == "pv" else ""
        print(f"正以普通 {args.mode.upper()}{detail} 移动到轨迹起点……")
        _move_to_start(
            robot,
            first,
            velocity=args.velocity,
            timeout=args.start_timeout,
            position_tolerance=args.position_tolerance,
            velocity_tolerance=args.velocity_tolerance,
        )
        print(
            f"开始按记录时间戳回放 {len(samples)} 个双臂轨迹点，"
            f"控制模式：普通 {args.mode.upper()}，无插值；"
            "按 Ctrl+C 可安全停止"
        )
        replay(
            robot,
            timestamps=timestamps,
            samples=samples,
            velocity=args.velocity,
        )
        print("双臂轨迹回放完成")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.disconnect()
        print("双臂已失能并断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="录制的双臂轨迹 JSON")
    parser.add_argument(
        "--mode",
        choices=("pv", "mit"),
        default="pv",
        help="回放控制模式；默认 pv",
    )
    parser.add_argument(
        "--velocity",
        type=speed_percent,
        default=50.0,
        help="普通 PV 速度百分比，范围 1–100；MIT 模式忽略；默认 50",
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
