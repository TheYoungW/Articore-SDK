#!/usr/bin/env python3
"""示例 10：将所有已启用电机的当前位置设置为零点。"""
from __future__ import annotations

import argparse

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

    print("正在以维护模式连接；不会配置 PV/MIT，也不会使能电机")
    try:
        completed = arm.set_zero()
        print("零点设置完成：", ", ".join(completed))
    finally:
        arm.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    main(parser.parse_args())
