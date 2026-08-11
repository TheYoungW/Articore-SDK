#!/usr/bin/env python3
"""示例 04：使用 PV 或 MIT 模式发送一组机械臂关节目标。"""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import (
    add_connection_arguments,
    parse_joint_positions_degrees,
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
        print("保持目标位置，按 Ctrl+C 停止并失能机械臂")
        arm.hold_joint_positions(target)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        arm.close()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--positions",
        default="0,-57.30,-57.30,0,34.38,0",
        help="逗号分隔的关节角度，单位为度",
    )
    parser.add_argument("--mode", choices=("pv", "mit"), default="pv")
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
