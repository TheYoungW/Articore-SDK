#!/usr/bin/env python3
"""示例 04：使用 PV 或 MIT 模式发送并保持双臂关节目标。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import add_connection_arguments, joint_degrees


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(
        transport=args.transport,
        left_channel=args.left_channel,
        right_channel=args.right_channel,
        baud=args.baud,
        control_mode=args.mode,
    )
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        print(f"已进入 {args.mode.upper()} 模式")
        print("保持双臂目标位置，按 Ctrl+C 停止并失能")
        robot.hold_joint_positions(
            left=joint_degrees(args.left),
            right=joint_degrees(args.right),
        )
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True, help="左臂 7 个关节角度，单位为度")
    parser.add_argument("--right", required=True, help="右臂 7 个关节角度，单位为度")
    parser.add_argument("--mode", choices=("pv", "mit"), default="pv")
    add_connection_arguments(parser)
    main(parser.parse_args())
