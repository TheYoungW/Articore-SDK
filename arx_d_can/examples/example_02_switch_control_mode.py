#!/usr/bin/env python3
"""示例 02：在双臂失能状态下切换 PV/MIT 控制模式。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功，所有电机保持失能")
    try:
        robot.configure_mode(args.mode)
        print(f"左右臂已切换到 {args.mode.upper()} 模式")
    finally:
        robot.disconnect()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        required=True,
        choices=("pv", "mit"),
        help="要写入左右机械臂的控制模式",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
