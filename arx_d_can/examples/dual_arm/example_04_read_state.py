#!/usr/bin/env python3
"""示例 04：在不使能电机的情况下读取双臂和夹爪状态。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm


READ_HZ = 100.0
DEFAULT_DISPLAY_HZ = 10.0


def _print_state(state) -> None:
    print("左臂角度 (deg):", [round(math.degrees(q), 2) for q in state.left.positions])
    print("右臂角度 (deg):", [round(math.degrees(q), 2) for q in state.right.positions])
    print("左臂速度 (rad/s):", [round(v, 3) for v in state.left.arm.velocities])
    print("右臂速度 (rad/s):", [round(v, 3) for v in state.right.arm.velocities])
    print("左臂力矩 (N·m):", [round(t, 3) for t in state.left.arm.torques])
    print("右臂力矩 (N·m):", [round(t, 3) for t in state.right.arm.torques])
    left_gripper = 0.0 if state.left.gripper is None else state.left.gripper.opening
    right_gripper = (
        0.0 if state.right.gripper is None else state.right.gripper.opening
    )
    print(f"左夹爪开合度: {left_gripper:.0f} / 1000")
    print(f"右夹爪开合度: {right_gripper:.0f} / 1000")


def _read_continuously(
    robot: ArxDCanDualArm,
    *,
    display_hz: float,
) -> None:
    period = 1.0 / READ_HZ
    display_period = 1.0 / display_hz
    deadline = time.perf_counter()
    next_display = deadline
    print(
        f"开始以 {READ_HZ:.0f} Hz 采集、{display_hz:g} Hz 显示，"
        "按 Ctrl+C 停止"
    )
    while True:
        state = robot.read_state()
        now = time.perf_counter()
        if now >= next_display:
            _print_state(state)
            next_display = now + display_period
        deadline += period
        remaining = deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            deadline = time.perf_counter()


def _display_hz(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or not 0.0 < value <= READ_HZ:
        raise argparse.ArgumentTypeError(
            f"display frequency must be greater than 0 and at most {READ_HZ:g} Hz"
        )
    return value


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect(read_only=True)
    print("机器人连接成功")
    try:
        if args.mode == "once":
            _print_state(robot.read_state())
        else:
            _read_continuously(
                robot,
                display_hz=args.display_hz,
            )
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.close()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("once", "continuous"),
        default="once",
        help="once 读取一次；continuous 以 100 Hz 持续读取",
    )
    parser.add_argument(
        "--display-hz",
        type=_display_hz,
        default=DEFAULT_DISPLAY_HZ,
        help="continuous 模式的终端刷新频率；默认 10 Hz，最高 100 Hz",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
