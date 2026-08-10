#!/usr/bin/env python3
"""示例 01：在不使能机械臂的情况下扫描达妙电机 ID。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        transport=args.transport,
    )
    ids = arm.scan_ids(
        start_id=args.start_id,
        end_id=args.end_id,
        model=args.model,
        feedback_base=args.feedback_base,
    )
    print("扫描结果：", " ".join(f"0x{motor_id:02X}" for motor_id in ids) or "未发现电机")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-id", type=int, default=1)
    parser.add_argument("--end-id", type=int, default=16)
    parser.add_argument("--model", default="4340P")
    parser.add_argument("--feedback-base", default="0x10")
    add_connection_arguments(parser)
    main(parser.parse_args())
