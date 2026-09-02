#!/usr/bin/env python3
"""控制示例 17（快速 MIT）：遥操/高频连续关节角目标接口。

真实遥操程序应在每次收到新的小角度目标时高频调用
set_joint_mit_fast()。本示例只提交一帧目标，用于验证接口。
"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import joint_degrees, speed_percent


DEFAULT_JOINT_TARGET_DEGREES = "0,0,0,20,0,0,0"


def main(args: argparse.Namespace) -> None:
    print(
        "安全警告：快速 MIT 只适合小角度连续目标；请勿一次提交与当前姿态"
        "差异过大的目标，否则机械臂可能快速、大幅运动。"
    )
    robot = ArxDCanDualArm(control_mode="mit")
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        print("已进入快速 MIT 模式")
        # 遥操集成时，应把这次调用放进上层的高频控制循环，每次
        # 传入最新的完整双臂目标。相邻帧应连续且与当前姿态接近。
        left = joint_degrees(args.left)
        right = joint_degrees(args.right)
        print("正以 100 Hz 重复提交快速 MIT 目标；按 Ctrl+C 失能并退出")
        while True:
            robot.set_joint_mit_fast(
                left=left,
                right=right,
                velocity=args.velocity,
            )
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.disconnect()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--left",
        default=DEFAULT_JOINT_TARGET_DEGREES,
        help="左臂 7 个关节角度，单位为度；默认 J4=20，其余为 0",
    )
    parser.add_argument(
        "--right",
        default=DEFAULT_JOINT_TARGET_DEGREES,
        help="右臂 7 个关节角度，单位为度；默认 J4=20，其余为 0",
    )
    parser.add_argument(
        "--velocity",
        type=speed_percent,
        default=100.0,
        help="快速 MIT 参考步进速度百分比，范围 1～100，默认 100",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
