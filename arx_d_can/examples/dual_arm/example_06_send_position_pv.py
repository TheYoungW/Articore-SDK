#!/usr/bin/env python3
"""示例 06（PV）：以统一速度设置双臂目标位置。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import joint_degrees, speed_percent


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        print("已进入 PV 模式")
        robot.set_joint_pv(
            left=joint_degrees(args.left),
            right=joint_degrees(args.right),
            velocity=args.velocity,
        )
        print("目标已提交，Runtime 正在平滑推进；按 Ctrl+C 失能并退出")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.close()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True, help="左臂 7 个关节角度，单位为度")
    parser.add_argument("--right", required=True, help="右臂 7 个关节角度，单位为度")
    parser.add_argument(
        "--velocity",
        type=speed_percent,
        required=True,
        help="双臂统一速度档位，范围 0～100",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
