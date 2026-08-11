#!/usr/bin/env python3
"""示例 03：安全清除所有活动 ARX-D-CAN 电机故障。"""
from __future__ import annotations

import argparse

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
    print("机器人连接成功")

    try:
        names = arm.clear_motor_faults()
        print("故障已清除：", ", ".join(names))
        print("所有电机保持失能状态")
    finally:
        arm.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    main(parser.parse_args())
