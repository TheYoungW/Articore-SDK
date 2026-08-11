#!/usr/bin/env python3
"""示例 06：测试一帧完整双臂状态的读取频率。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import add_connection_arguments
from arx_d_can.service_tools.read_benchmark import benchmark_state_reads


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
        if args.cached:
            robot.read_state()
        result = benchmark_state_reads(
            robot,
            seconds=args.seconds,
            target_hz=args.hz,
            cached=args.cached,
        )
        print(f"读取次数：{result.samples}")
        print(f"实际频率：{result.achieved_hz:.2f} Hz")
        print(f"平均耗时：{result.avg_read_s * 1000.0:.3f} ms")
        print(f"最大耗时：{result.max_read_s * 1000.0:.3f} ms")
        print(f"错过周期：{result.missed_deadlines}")
    finally:
        robot.close(disable=False)
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0, help="测试时间")
    parser.add_argument("--hz", type=float, default=500.0, help="目标读取频率")
    parser.add_argument("--cached", action="store_true", help="测试缓存读取")
    add_connection_arguments(parser)
    main(parser.parse_args())
