#!/usr/bin/env python3
"""示例 05：显式设置夹爪的 0～1000 开合度。"""
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
        arm.set_gripper_opening(args.opening)
        print(f"夹爪开合度已设置为 {args.opening:g}")
        input("按回车失能并退出...")
    finally:
        arm.close()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--opening",
        type=float,
        required=True,
        help="夹爪开合度：0 表示闭合，1000 表示打开",
    )
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
