#!/usr/bin/env python3
"""控制示例 17（快速跟随 MIT）：以高频最新目标驱动双臂。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import joint_degrees


def main(args: argparse.Namespace) -> None:
    print(
        "安全警告：快速跟随 MIT 只适合小角度连续目标；请勿一次提交与当前姿态"
        "差异过大的目标，否则机械臂可能快速、大幅运动。"
    )
    robot = ArxDCanDualArm(control_mode="mit")
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        print("已进入快速跟随 MIT 模式")
        robot.set_joint_mit_fast_follow(
            left=joint_degrees(args.left),
            right=joint_degrees(args.right),
        )
        print(
            "快速跟随目标已提交；按 Ctrl+C 失能并退出"
        )
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.disconnect()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True, help="左臂 7 个关节角度，单位为度")
    parser.add_argument("--right", required=True, help="右臂 7 个关节角度，单位为度")
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
