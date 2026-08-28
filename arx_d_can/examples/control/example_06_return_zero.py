#!/usr/bin/env python3
"""控制示例 06：控制双臂和已安装夹爪返回零位。"""
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
        if robot.has_grippers:
            robot.set_grippers(left=0, right=0, gripper_level=5)
            target = "双臂和夹爪"
        else:
            target = "双臂"
        input(f"目标已提交，确认{target}回到零位后按回车...")
        print(f"{target}已返回零位")
    finally:
        robot.disconnect()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--velocity",
        type=speed_percent,
        default=50.0,
        help="统一速度档位，范围 0～100；默认 50",
    )
    main(parser.parse_args())
