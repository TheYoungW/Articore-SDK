#!/usr/bin/env python3
"""示例 02：在不使能电机的情况下读取机械臂和夹爪状态。"""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        port=args.port,
        transport=args.transport,
        baud=args.baud,
        enable_gripper=True,
    )

    arm.connect()
    print("机器人连接成功", flush=True)

    try:
        state = arm.read_state()
        angles_rad = state.arm.positions
        angles_deg = [math.degrees(value) for value in angles_rad]

        print("\n--- 关节状态 ---", flush=True)
        print(
            "  关节角度 (deg):   ["
            + ", ".join(f"{value:.2f}" for value in angles_deg)
            + "]",
            flush=True,
        )
        print(
            "  关节角度 (rad):   ["
            + ", ".join(f"{value:.4f}" for value in angles_rad)
            + "]",
            flush=True,
        )
        print(
            "  关节速度 (rad/s): ["
            + ", ".join(f"{value:.3f}" for value in state.arm.velocities)
            + "]",
            flush=True,
        )
        print(
            "  关节力矩 (N·m):   ["
            + ", ".join(f"{value:.3f}" for value in state.arm.torques)
            + "]",
            flush=True,
        )

        if state.gripper is not None:
            print(
                f"  夹爪开合度:        {state.gripper.opening:.0f} / 1000",
                flush=True,
            )
    finally:
        arm.close(disable=False)
        print("已断开连接", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="读取一次机械臂状态，不使能电机")
    add_connection_arguments(parser)
    main(parser.parse_args())
