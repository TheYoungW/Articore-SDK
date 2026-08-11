#!/usr/bin/env python3
"""示例 02：在不使能电机的情况下读取双臂和夹爪状态。"""
from __future__ import annotations

import argparse
import math

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
        state = robot.read_state()
        print("左臂角度 (deg):", [round(math.degrees(q), 2) for q in state.left.positions])
        print("右臂角度 (deg):", [round(math.degrees(q), 2) for q in state.right.positions])
        print("左臂速度 (rad/s):", [round(v, 3) for v in state.left.arm.velocities])
        print("右臂速度 (rad/s):", [round(v, 3) for v in state.right.arm.velocities])
        print("左臂力矩 (N·m):", [round(t, 3) for t in state.left.arm.torques])
        print("右臂力矩 (N·m):", [round(t, 3) for t in state.right.arm.torques])
        if state.left.gripper is not None:
            print(f"左夹爪开合度: {state.left.gripper.opening:.0f} / 1000")
        if state.right.gripper is not None:
            print(f"右夹爪开合度: {state.right.gripper.opening:.0f} / 1000")
    finally:
        robot.close(disable=False)
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    main(parser.parse_args())
