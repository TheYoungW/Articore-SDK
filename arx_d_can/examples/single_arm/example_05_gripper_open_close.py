#!/usr/bin/env python3
"""示例 05：张开和闭合夹爪。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanArm
from arx_d_can.examples.single_arm.common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        port=args.port,
        transport=args.transport,
        baud=args.baud,
        enable_gripper=True,
    )
    arm.connect()
    print("机器人连接成功")

    try:
        arm.enable()

        print("张开夹爪")
        arm.move_gripper(1000, seconds=args.seconds)

        print("闭合夹爪")
        arm.move_gripper(0, seconds=args.seconds)
    finally:
        arm.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=2.0, help="每次动作持续时间")
    add_connection_arguments(parser)
    main(parser.parse_args())
