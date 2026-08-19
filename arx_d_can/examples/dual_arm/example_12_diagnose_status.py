#!/usr/bin/env python3
"""示例 12：读取左右臂电机状态、温度和控制模式。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.sdk.diagnostics import print_diagnostic_summary


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect(read_only=True)
    print("机器人连接成功")
    try:
        left = []
        right = []
        for sample in range(1, args.samples + 1):
            left, right = robot.read_motor_diagnostics()
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
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=3, help="读取次数")
    parser.add_argument("--interval", type=float, default=0.1, help="读取间隔")
    parser.add_argument("--temperature-warning", type=float, default=80.0)
    main(parser.parse_args())
