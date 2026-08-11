#!/usr/bin/env python3
"""示例 09：读取左右臂电机状态、温度和控制模式。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.diagnostics import print_diagnostic_summary
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
        left = []
        right = []
        for sample in range(1, args.samples + 1):
            left = robot.left.read_motor_diagnostics(timeout_ms=args.timeout_ms)
            right = robot.right.read_motor_diagnostics(timeout_ms=args.timeout_ms)
            print(f"\n--- 诊断数据 #{sample} ---")
            for label, diagnostics in (("左", left), ("右", right)):
                for item in diagnostics:
                    if item.error is not None:
                        print(f"  {label} {item.name}: 读取失败：{item.error}")
                    else:
                        print(
                            f"  {label} {item.name}: 状态={item.status} 模式={item.mode} "
                            f"位置={math.degrees(item.position or 0.0):+.2f}° "
                            f"温度={item.rotor_temperature:.0f}°C"
                        )
            if sample < args.samples:
                time.sleep(args.interval)
        print("\n左臂汇总：")
        print_diagnostic_summary(left, temperature_warning=args.temperature_warning)
        print("右臂汇总：")
        print_diagnostic_summary(right, temperature_warning=args.temperature_warning)
    finally:
        robot.close(disable=False)
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=3, help="读取次数")
    parser.add_argument("--interval", type=float, default=0.1, help="读取间隔")
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument("--temperature-warning", type=float, default=80.0)
    add_connection_arguments(parser)
    main(parser.parse_args())
