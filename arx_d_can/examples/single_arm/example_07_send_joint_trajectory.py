#!/usr/bin/env python3
"""示例 07：平滑移动机械臂关节。"""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanArm
from arx_d_can.examples.single_arm.common import (
    add_connection_arguments,
    parse_joint_positions_degrees,
    positive_velocity_degrees,
)


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        port=args.port,
        transport=args.transport,
        baud=args.baud,
        enable_gripper=True,
    )
    target = parse_joint_positions_degrees(
        args.positions,
        expected_count=len(arm.joint_names),
    )

    arm.connect()
    print("机器人连接成功")

    try:
        arm.enable()

        print("开始平滑移动")
        state = arm.move_joint_positions(
            target,
            velocity=args.velocity,
            profile=args.profile,
        )
        print(
            "到位角度：",
            [round(math.degrees(value), 2) for value in state.arm.positions],
        )

        if args.return_zero:
            print("平滑返回零位")
            arm.move_joint_positions(
                [0.0] * len(arm.joint_names),
                velocity=args.velocity,
                profile=args.profile,
            )
    finally:
        arm.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("positions", help="逗号分隔的目标角度，单位为度")
    parser.add_argument(
        "--velocity",
        type=positive_velocity_degrees,
        help="轨迹速度，单位为度/秒；省略时使用 SDK 默认轨迹速度",
    )
    parser.add_argument(
        "--profile",
        choices=("min_jerk", "linear"),
        default="min_jerk",
        help="轨迹插值方式",
    )
    parser.add_argument("--return-zero", action="store_true", help="到位后平滑返回零位")
    add_connection_arguments(parser)
    main(parser.parse_args())
