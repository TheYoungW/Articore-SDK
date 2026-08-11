#!/usr/bin/env python3
"""示例 08：平滑控制机械臂回到零位。"""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanArm
from arx_d_can.examples.single_arm.common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        port=args.port,
        transport=args.transport,
        baud=args.baud,
        enable_gripper=True,
    )
    zero = [0.0] * len(arm.joint_names)

    arm.connect()
    print("机器人连接成功")

    try:
        arm.enable()

        print("机械臂开始返回零位")
        state = arm.move_joint_positions(zero, seconds=args.seconds)
        print(
            "当前角度：",
            [round(math.degrees(value), 2) for value in state.arm.positions],
        )

        if arm.has_gripper:
            print("夹爪返回闭合位置")
            arm.move_gripper(0, seconds=2.0)
    finally:
        arm.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=6.0, help="回零运动时间")
    add_connection_arguments(parser)
    main(parser.parse_args())
