#!/usr/bin/env python3
"""示例 11：录制或回放双臂与夹爪轨迹。"""
from __future__ import annotations

import argparse
from pathlib import Path

from arx_d_can import ArxDCanDualArm
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
        if args.command == "record":
            print("电机保持失能，请手动拖动双臂和夹爪")
            count = robot.record_trajectory(args.file, seconds=args.seconds, hz=args.hz)
            print(f"已保存 {count} 个双臂轨迹点：{args.file}")
        else:
            robot.enable()
            print("开始回放双臂轨迹")
            count = robot.replay_trajectory(args.file)
            print(f"轨迹回放完成，共 {count} 个轨迹点")
    finally:
        robot.close()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("record", "replay"), help="录制或回放")
    parser.add_argument("file", type=Path, help="双臂轨迹 JSON 文件")
    parser.add_argument("--seconds", type=float, default=10.0, help="录制时间")
    parser.add_argument("--hz", type=float, default=100.0, help="录制频率")
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
