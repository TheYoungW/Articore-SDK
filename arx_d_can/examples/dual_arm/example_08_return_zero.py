#!/usr/bin/env python3
"""示例 08：平滑控制双臂和夹爪返回零位。"""
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
        robot.move_joint_positions(
            left=[0.0] * len(robot.left.joint_names),
            right=[0.0] * len(robot.right.joint_names),
            seconds=args.seconds,
        )
        if robot.left.has_gripper and robot.right.has_gripper:
            robot.move_grippers(left=0, right=0)
        print("双臂已返回零位")
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=6.0, help="回零运动时间")
    add_connection_arguments(parser)
    main(parser.parse_args())
