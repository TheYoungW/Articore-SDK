#!/usr/bin/env python3
"""示例 09：读取电机状态、温度和控制模式。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanArm
from arx_d_can.sdk.diagnostics import print_diagnostic_summary
from arx_d_can.examples.single_arm.common import add_connection_arguments


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
        diagnostics = []
        for sample in range(1, args.samples + 1):
            diagnostics = arm.read_motor_diagnostics()
            print(f"\n--- 诊断数据 #{sample} ---")
            for item in diagnostics:
                if item.error is not None:
                    print(f"  {item.name}: 读取失败：{item.error}")
                    continue
                print(
                    f"  {item.name}: 状态={item.status} 模式={item.mode} "
                    f"位置={math.degrees(item.position or 0.0):+.2f}° "
                    f"温度={item.rotor_temperature:.0f}°C"
                )
            if sample < args.samples:
                time.sleep(args.interval)

        print_diagnostic_summary(
            diagnostics,
            temperature_warning=args.temperature_warning,
        )
    finally:
        arm.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=3, help="读取次数")
    parser.add_argument("--interval", type=float, default=0.1, help="读取间隔")
    parser.add_argument("--temperature-warning", type=float, default=80.0)
    add_connection_arguments(parser)
    main(parser.parse_args())
