#!/usr/bin/env python3
"""示例 01：在不使能双臂的情况下分别扫描左右通道电机 ID。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(
        transport=args.transport,
        left_channel=args.left_channel,
        right_channel=args.right_channel,
        baud=args.baud,
        left_gripper=False,
        right_gripper=False,
    )
    left_ids = robot.left.scan_ids(
        start_id=args.start_id,
        end_id=args.end_id,
        model=args.model,
        feedback_base=args.feedback_base,
    )
    right_ids = robot.right.scan_ids(
        start_id=args.start_id,
        end_id=args.end_id,
        model=args.model,
        feedback_base=args.feedback_base,
    )
    print("左臂扫描结果：", " ".join(f"0x{value:02X}" for value in left_ids) or "未发现电机")
    print("右臂扫描结果：", " ".join(f"0x{value:02X}" for value in right_ids) or "未发现电机")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-id", type=int, default=1)
    parser.add_argument("--end-id", type=int, default=16)
    parser.add_argument("--model", default="4340P")
    parser.add_argument("--feedback-base", default="0x10")
    add_connection_arguments(parser)
    main(parser.parse_args())
