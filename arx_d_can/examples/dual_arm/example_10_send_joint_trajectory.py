#!/usr/bin/env python3
"""示例 10：沿同一时间轴平滑移动左右臂。"""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import (
    joint_degrees,
    positive_velocity_degrees,
)


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        state = robot.move_joint_positions(
            left=joint_degrees(args.left),
            right=joint_degrees(args.right),
            velocity=args.velocity,
            profile=args.profile,
        )
        print("左臂到位角度：", [round(math.degrees(q), 2) for q in state.left.positions])
        print("右臂到位角度：", [round(math.degrees(q), 2) for q in state.right.positions])
        if args.return_zero:
            robot.move_joint_positions(
                left=[0.0] * len(robot.left.joint_names),
                right=[0.0] * len(robot.right.joint_names),
                velocity=args.velocity,
                profile=args.profile,
            )
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True, help="左臂 7 个目标角度，单位为度")
    parser.add_argument("--right", required=True, help="右臂 7 个目标角度，单位为度")
    parser.add_argument(
        "--velocity",
        type=positive_velocity_degrees,
        help="双臂轨迹速度，单位为度/秒；省略时使用 SDK 默认轨迹速度",
    )
    parser.add_argument(
        "--profile",
        choices=("min_jerk", "linear"),
        default="min_jerk",
        help="轨迹插值方式",
    )
    parser.add_argument("--return-zero", action="store_true", help="到位后返回零位")
    main(parser.parse_args())
