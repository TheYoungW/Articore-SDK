#!/usr/bin/env python3
"""示例 04：使用 PV 或 MIT 模式平滑移动到目标位置。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanArm
from arx_d_can.examples.single_arm.common import (
    add_connection_arguments,
    parse_joint_positions_degrees,
    speed_percent,
)


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        port=args.port,
        transport=args.transport,
        baud=args.baud,
        control_mode=args.mode,
        enable_gripper=True,
    )
    target = parse_joint_positions_degrees(
        args.positions,
        expected_count=len(arm.joint_names),
    )

    arm.connect()
    print("机器人连接成功")

    try:
        arm.enable()
        print(f"已进入 {args.mode.upper()} 模式")
        print("目标角度：", [round(math.degrees(value), 2) for value in target])
        getattr(arm, f"set_joint_{args.mode}")(target, velocity=args.velocity)
        print("目标已提交，Runtime 正在平滑推进；按 Ctrl+C 失能并退出")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        arm.close()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--positions",
        required=True,
        help="逗号分隔的关节角度，单位为度",
    )
    parser.add_argument("--mode", choices=("pv", "mit"), default="pv")
    parser.add_argument(
        "--velocity",
        type=speed_percent,
        default=30.0,
        help="统一速度档位，范围 0～100；默认 30",
    )
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
