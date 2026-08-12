#!/usr/bin/env python3
"""示例 11：平滑控制双臂和夹爪返回零位。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import positive_velocity_degrees


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        robot.move_joint_positions(
            left=[0.0] * len(robot.left.joint_names),
            right=[0.0] * len(robot.right.joint_names),
            velocity=args.velocity,
        )
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
        type=positive_velocity_degrees,
        help="回零速度，单位为度/秒；省略时使用 SDK 默认轨迹速度",
    )
    main(parser.parse_args())
