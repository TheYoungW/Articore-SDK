#!/usr/bin/env python3
"""示例 12：开启单臂重力补偿。"""
from __future__ import annotations

import time

from arx_d_can import ArxDCanArm, GravityCompensationMode
from arx_d_can.service_tools.gravity_compensation_cli import build_parser


def main(args) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        port=args.port,
        transport=args.transport,
        baud=args.baud,
        control_mode="mit",
        enable_gripper=True,
    )
    gravity = GravityCompensationMode(arm)

    print("请托稳机械臂，重力补偿启动后机械臂可以被手动拖动")
    for remaining in range(3, 0, -1):
        print(f"{remaining} 秒后启动……")
        time.sleep(1.0)

    try:
        with gravity:
            print("重力补偿已启动，按 Ctrl+C 停止")
            gravity.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    print("机械臂已失能")


if __name__ == "__main__":
    main(build_parser().parse_args())
