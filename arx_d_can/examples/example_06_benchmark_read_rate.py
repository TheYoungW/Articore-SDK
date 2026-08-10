#!/usr/bin/env python3
"""示例 06：测试状态读取频率。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments
from arx_d_can.service_tools.read_benchmark import benchmark_state_reads


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        transport=args.transport,
        enable_gripper=True,
    )
    arm.connect()
    print("机器人连接成功")

    try:
        if args.cached:
            arm.read_state()
        result = benchmark_state_reads(
            arm,
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
        arm.close(disable=False)
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0, help="测试时间")
    parser.add_argument("--hz", type=float, default=500.0, help="目标读取频率")
    parser.add_argument("--cached", action="store_true", help="测试缓存读取")
    add_connection_arguments(parser)
    main(parser.parse_args())
