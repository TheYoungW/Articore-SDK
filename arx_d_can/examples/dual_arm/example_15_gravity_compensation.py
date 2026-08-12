#!/usr/bin/env python3
"""示例 15：开启 Yunyi 双臂重力补偿。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm, DualArmGravityCompensationMode


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    gravity = DualArmGravityCompensationMode(robot)
    print("请托稳双臂，重力补偿启动后机械臂可以被手动拖动")
    for remaining in range(3, 0, -1):
        print(f"{remaining} 秒后启动……")
        time.sleep(1.0)
    try:
        with gravity:
            print("双臂重力补偿已启动，按 Ctrl+C 停止")
            gravity.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    print("双臂已失能")


def build_parser() -> argparse.ArgumentParser:
    return argparse.ArgumentParser(description=__doc__)


if __name__ == "__main__":
    main(build_parser().parse_args())
