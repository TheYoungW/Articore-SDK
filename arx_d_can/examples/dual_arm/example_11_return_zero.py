#!/usr/bin/env python3
"""示例 11：使用普通 MIT 位置接口控制双臂返回零位。"""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import mit_velocity_degrees


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        robot.set_joint_mit(
            left=[0.0] * len(robot.left.joint_names),
            right=[0.0] * len(robot.right.joint_names),
            velocity=args.velocity,
        )
        input("目标已提交，确认双臂回到零位后按回车...")
        if robot.left.has_gripper and robot.right.has_gripper:
            robot.set_gripper_openings(left=0, right=0)
        print("双臂已返回零位")
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--velocity",
        type=mit_velocity_degrees,
        default=math.radians(60.0),
        help="统一最大参考速度，单位为度/秒，范围 (0, 200]；默认 60",
    )
    main(parser.parse_args())
