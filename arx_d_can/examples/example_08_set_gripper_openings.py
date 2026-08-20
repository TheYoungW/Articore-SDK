#!/usr/bin/env python3
"""示例 08：使用 0～1000 开合度设置左右夹爪。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import gripper_opening


def _gripper_level(text: str) -> int:
    value = int(text)
    if not 0 <= value <= 10:
        raise argparse.ArgumentTypeError("gripper level must be in 0..10")
    return value


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功")
    try:
        if args.mode == "direct":
            print(
                "警告：直驱模式不会执行夹爪堵转判断或过载退让，"
                "请关注温升、机械过载和被夹物体安全"
            )
        robot.enable()
        robot.set_grippers(
            left=args.left_gripper,
            right=args.right_gripper,
            gripper_level=args.gripper_level,
            mode=args.mode,
        )
        print(
            f"夹爪目标已设置：left={args.left_gripper:g}, "
            f"right={args.right_gripper:g}, "
            f"gripper_level={args.gripper_level}, mode={args.mode}"
        )
        input("按回车退出并失能...")
    finally:
        robot.disconnect()
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
    parser.add_argument(
        "--gripper-level",
        type=_gripper_level,
        default=5,
        help="夹持力等级 0..10，0=无主动夹持刚度；默认 5",
    )
    parser.add_argument(
        "--mode",
        choices=("protected", "direct"),
        default="protected",
        help="protected=防堵转保护；direct=持续直驱，默认 protected",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
