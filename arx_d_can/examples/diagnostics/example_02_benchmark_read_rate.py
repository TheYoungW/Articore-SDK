#!/usr/bin/env python3
"""诊断示例 02：测试一帧完整双臂状态的读取频率。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import benchmark_state_reads


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功")
    try:
        result = benchmark_state_reads(
            robot,
            seconds=args.seconds,
            target_hz=args.hz,
        )
        print(f"读取次数：{result.samples}")
        print(f"实际频率：{result.achieved_hz:.2f} Hz")
        print(f"平均耗时：{result.avg_read_s * 1000.0:.3f} ms")
        print(f"最大耗时：{result.max_read_s * 1000.0:.3f} ms")
        print(f"错过周期：{result.missed_deadlines}")
    finally:
        robot.disconnect()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0, help="测试时间")
    parser.add_argument("--hz", type=float, default=500.0, help="目标读取频率")
    main(parser.parse_args())
