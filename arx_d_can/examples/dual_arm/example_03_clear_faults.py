#!/usr/bin/env python3
"""示例 03：清除左右臂和夹爪电机故障。"""
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
        left = robot.left.clear_motor_faults()
        right = robot.right.clear_motor_faults()
        print("左臂故障已清除：", ", ".join(left))
        print("右臂故障已清除：", ", ".join(right))
        print("所有电机保持失能状态")
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    main(parser.parse_args())
