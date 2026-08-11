#!/usr/bin/env python3
"""示例 05：同时张开和闭合左右夹爪。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(
        transport=args.transport,
        left_channel=args.left_channel,
        right_channel=args.right_channel,
        baud=args.baud,
    )
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        print("张开左右夹爪")
        robot.move_grippers(left=1000, right=1000, seconds=args.seconds)
        print("闭合左右夹爪")
        robot.move_grippers(left=0, right=0, seconds=args.seconds)
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=2.0, help="每次动作持续时间")
    add_connection_arguments(parser)
    main(parser.parse_args())
