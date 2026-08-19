#!/usr/bin/env python3
"""示例 11：使用普通 MIT 位置接口控制双臂返回零位。"""
from __future__ import annotations

import argparse
from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import speed_percent


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        robot.set_joint_mit(
            left=[0.0] * len(robot.joint_names),
            right=[0.0] * len(robot.joint_names),
            velocity=args.velocity,
        )
        input("目标已提交，确认双臂回到零位后按回车...")
        robot.set_grippers(left=0, right=0, gripper_level=3)
        print("双臂已返回零位")
    finally:
        robot.disconnect()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--velocity",
        type=speed_percent,
        default=30.0,
        help="统一速度档位，范围 0～100；默认 30",
    )
    main(parser.parse_args())
