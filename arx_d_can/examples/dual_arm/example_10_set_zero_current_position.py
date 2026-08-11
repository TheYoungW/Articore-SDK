#!/usr/bin/env python3
"""示例 10：将双臂当前位置设置为电机零点。"""
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
    print("机器人连接成功，电机保持失能状态")
    try:
        left = robot.left.set_zero(joint_names=robot.left.joint_names)
        right = robot.right.set_zero(joint_names=robot.right.joint_names)
        print("左臂零点设置完成：", ", ".join(left))
        print("右臂零点设置完成：", ", ".join(right))
    finally:
        robot.close(disable=False)
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    main(parser.parse_args())
