#!/usr/bin/env python3
"""示例 08：使用 0～1000 开合度设置左右夹爪。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import gripper_opening


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        robot.set_gripper_openings(
            left=args.left_gripper,
            right=args.right_gripper,
        )
        print(
            f"夹爪目标已设置：left={args.left_gripper:g}, "
            f"right={args.right_gripper:g}"
        )
        input("按回车退出并失能...")
    finally:
        robot.close()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--left-gripper",
        required=True,
        type=gripper_opening,
        help="左夹爪开合度，0=闭合，1000=打开",
    )
    parser.add_argument(
        "--right-gripper",
        required=True,
        type=gripper_opening,
        help="右夹爪开合度，0=闭合，1000=打开",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
