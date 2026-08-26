#!/usr/bin/env python3
"""控制示例 10：安全回到双臂轨迹起点后，按原始时间戳原子回放。"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import (
    joint_values,
    positive_velocity_degrees,
    speed_percent,
)
from arx_d_can.service_tools.dual_trajectory_recording import (
    DEFAULT_MIT_FEEDFORWARD_TORQUES,
    DEFAULT_MIT_KD,
    DEFAULT_MIT_KP,
    DualArmTrajectorySample,
    load_trajectory,
    replay,
)


def _move_to_start(
    robot: ArxDCanDualArm,
    target: DualArmTrajectorySample,
    *,
    start_velocity: float,
    max_speed_percent: float,
    timeout: float,
    position_tolerance: float,
    velocity_tolerance: float,
    mit_kp: tuple[float, ...] = DEFAULT_MIT_KP,
    mit_kd: tuple[float, ...] = DEFAULT_MIT_KD,
    mit_feedforward_torques: tuple[float, ...] = DEFAULT_MIT_FEEDFORWARD_TORQUES,
) -> None:
    state = robot.read_cached_state()
    current_left = tuple(state.left.arm.positions)
    current_right = tuple(state.right.arm.positions)
    if robot.control_mode == "pv":
        robot.set_max_speed(max_speed_percent)
        robot.set_joint_pv(
            left=target.left_positions,
            right=target.right_positions,
        )
    else:
        largest_move = max(
            *(abs(end - start) for start, end in zip(current_left, target.left_positions)),
            *(abs(end - start) for start, end in zip(current_right, target.right_positions)),
        )
        duration = max(0.5, 1.875 * largest_move / start_velocity)
        robot.start_trajectory(
            timestamps=[0.0, duration],
            left_positions=[current_left, target.left_positions],
            right_positions=[current_right, target.right_positions],
            kp=mit_kp,
            kd=mit_kd,
            feedforward_torque=mit_feedforward_torques,
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
        if args.mode == "pv":
            print(f"正以 {args.max_speed:g}% 最大速度移动双臂到轨迹起点……")
        else:
            print(
                f"正以 {math.degrees(args.start_velocity):g}°/s "
                "原子移动双臂到轨迹起点……"
            )
        _move_to_start(
            robot,
            first,
            start_velocity=args.start_velocity,
            max_speed_percent=args.max_speed,
            timeout=args.start_timeout,
            position_tolerance=args.position_tolerance,
            velocity_tolerance=args.velocity_tolerance,
            mit_kp=args.mit_kp,
            mit_kd=args.mit_kd,
            mit_feedforward_torques=args.mit_feedforward_torque,
        )
        print(
            f"开始原子回放 {len(samples)} 个双臂轨迹点，"
            f"控制模式：{args.mode.upper()}，插值模式：{args.interpolation}；"
            "按 Ctrl+C 可安全停止"
        )
        replay(
            robot,
            timestamps=timestamps,
            samples=samples,
            interpolation="quintic",
            mit_kp=args.mit_kp,
            mit_kd=args.mit_kd,
            mit_feedforward_torques=args.mit_feedforward_torque,
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
        "--start-velocity",
        type=positive_velocity_degrees,
        default=math.radians(30.0),
        help="MIT 返回轨迹起点的统一速度，单位为度/秒；PV 模式忽略；默认 30",
    )
    parser.add_argument(
        "--max-speed",
        type=speed_percent,
        default=50.0,
        help="PV reference 速度百分比 0–100；MIT 模式忽略；默认 50（1 rad/s）",
    )
    parser.add_argument(
        "--interpolation",
        choices=("quintic",),
        default="quintic",
        help="原生轨迹固定使用五次多项式插值",
    )
    parser.add_argument(
        "--mit-kp",
        type=lambda text: joint_values(text, name="MIT Kp"),
        default=DEFAULT_MIT_KP,
        help="MIT 7 轴 Kp；默认 190,190,70,125,10,22,28",
    )
    parser.add_argument(
        "--mit-kd",
        type=lambda text: joint_values(text, name="MIT Kd"),
        default=DEFAULT_MIT_KD,
        help="MIT 7 轴 Kd；默认 4.55,4.5,2,2.9,0.7,0.89,0.84",
    )
    parser.add_argument(
        "--mit-feedforward-torque",
        type=lambda text: joint_values(text, name="MIT 前馈力矩"),
        default=DEFAULT_MIT_FEEDFORWARD_TORQUES,
        help="MIT 7 轴前馈力矩，单位为 N·m；默认全部为 0",
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
