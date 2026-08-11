#!/usr/bin/env python3
"""示例 07：沿同一时间轴平滑移动左右臂。"""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import add_connection_arguments, joint_degrees


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
        state = robot.move_joint_positions(
            left=joint_degrees(args.left),
            right=joint_degrees(args.right),
            seconds=args.seconds,
        )
        print("左臂到位角度：", [round(math.degrees(q), 2) for q in state.left.positions])
        print("右臂到位角度：", [round(math.degrees(q), 2) for q in state.right.positions])
        if args.return_zero:
            robot.move_joint_positions(
                left=[0.0] * len(robot.left.joint_names),
                right=[0.0] * len(robot.right.joint_names),
                seconds=args.seconds,
            )
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True, help="左臂 7 个目标角度，单位为度")
    parser.add_argument("--right", required=True, help="右臂 7 个目标角度，单位为度")
    parser.add_argument("--seconds", type=float, default=6.0, help="运动时间")
    parser.add_argument("--return-zero", action="store_true", help="到位后返回零位")
    add_connection_arguments(parser)
    main(parser.parse_args())
