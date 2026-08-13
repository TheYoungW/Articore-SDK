#!/usr/bin/env python3
"""示例 08：使用普通 PV 位置接口控制机械臂回到零位。"""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanArm
from arx_d_can.examples.single_arm.common import (
    add_connection_arguments,
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
    zero = [0.0] * len(arm.joint_names)

    arm.connect()
    print("机器人连接成功")

    try:
        arm.enable()

        print("机械臂开始返回零位")
        arm.set_joint_pv(zero, velocity=args.velocity)
        input("目标已提交，确认机械臂回到零位后按回车...")
        state = arm.read_state()
        print(
            "当前角度：",
            [round(math.degrees(value), 2) for value in state.arm.positions],
        )

        if arm.has_gripper:
            print("夹爪返回闭合位置")
            arm.set_gripper_opening(0)
    finally:
        arm.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--velocity",
        type=positive_velocity_degrees,
        default=math.radians(60.0),
        help="统一最大参考速度，单位为度/秒；默认 60",
    )
    add_connection_arguments(parser)
    main(parser.parse_args())
