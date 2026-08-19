#!/usr/bin/env python3
"""示例 07（MIT）：持续发送包含位置、速度、Kp、Kd 和前馈力矩的完整双臂 MIT 帧。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import (
    joint_degrees,
    joint_values,
    joint_velocity_degrees,
)


COMMAND_HZ = 100.0
DEFAULT_TARGET_VELOCITY = "0,0,0,0,0,0,0"
DEFAULT_KP = "190,190,70,125,10,22,28"
DEFAULT_KD = "4.55,4.5,2,2.9,0.7,0.89,0.84"
DEFAULT_FEEDFORWARD_TORQUE = "0,0,0,0,0,0,0"


def main(args: argparse.Namespace) -> None:
    left_positions = joint_degrees(args.left)
    right_positions = joint_degrees(args.right)
    robot = ArxDCanDualArm(control_mode="mit")
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        print(
            "开始发送完整 MIT 帧："
            f"target_velocity={args.target_velocity}, Kp={args.kp}, "
            f"Kd={args.kd}, feedforward_torque={args.feedforward_torque}"
        )
        period = 1.0 / COMMAND_HZ
        deadline = time.perf_counter()
        while True:
            robot.submit_raw_mit(
                left_positions=left_positions,
                right_positions=right_positions,
                left_velocities=args.target_velocity,
                right_velocities=args.target_velocity,
                kp=args.kp,
                kd=args.kd,
                left_feedforward_torques=args.feedforward_torque,
                right_feedforward_torques=args.feedforward_torque,
            )
            deadline += period
            remaining = deadline - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                deadline = time.perf_counter()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.disconnect()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--left",
        required=True,
        help='左臂 7 个目标角度，单位为度，例如 "0,0,0,90,0,0,0"',
    )
    parser.add_argument(
        "--right",
        required=True,
        help='右臂 7 个目标角度，单位为度，例如 "0,0,0,90,0,0,0"',
    )
    parser.add_argument(
        "--target-velocity",
        type=joint_velocity_degrees,
        default=DEFAULT_TARGET_VELOCITY,
        help=f"双臂 7 个目标速度，单位为度/秒；默认 {DEFAULT_TARGET_VELOCITY}",
    )
    parser.add_argument(
        "--kp",
        type=lambda text: joint_values(text, name="Kp"),
        default=DEFAULT_KP,
        help=f"双臂 7 个 Kp；默认 {DEFAULT_KP}",
    )
    parser.add_argument(
        "--kd",
        type=lambda text: joint_values(text, name="Kd"),
        default=DEFAULT_KD,
        help=f"双臂 7 个 Kd；默认 {DEFAULT_KD}",
    )
    parser.add_argument(
        "--feedforward-torque",
        type=lambda text: joint_values(text, name="前馈力矩"),
        default=DEFAULT_FEEDFORWARD_TORQUE,
        help=f"双臂 7 个前馈力矩，单位为 N·m；默认 {DEFAULT_FEEDFORWARD_TORQUE}",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
