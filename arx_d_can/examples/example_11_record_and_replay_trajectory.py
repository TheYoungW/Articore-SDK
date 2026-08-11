#!/usr/bin/env python3
"""示例 11：录制或回放机械臂与夹爪轨迹。"""
from __future__ import annotations

import argparse
from pathlib import Path

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        port=args.port,
        transport=args.transport,
        baud=args.baud,
        control_mode="pv",
        enable_gripper=True,
    )

    arm.connect()
    print("机器人连接成功")

    try:
        if args.command == "record":
            print("电机保持失能，请手动拖动机械臂和夹爪")
            count = arm.record_trajectory(
                args.file,
                seconds=args.seconds,
                hz=args.hz,
            )
            print(f"已保存 {count} 个轨迹点：{args.file}")
        else:
            arm.enable()
            print("开始回放轨迹")
            count = arm.replay_trajectory(args.file)
            print(f"轨迹回放完成，共 {count} 个轨迹点")
    finally:
        arm.close()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("record", "replay"), help="录制或回放")
    parser.add_argument("file", type=Path, help="轨迹 JSON 文件")
    parser.add_argument("--seconds", type=float, default=10.0, help="录制时间")
    parser.add_argument("--hz", type=float, default=100.0, help="录制频率")
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
