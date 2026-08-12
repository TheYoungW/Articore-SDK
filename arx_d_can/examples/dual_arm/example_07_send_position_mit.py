#!/usr/bin/env python3
"""示例 07（MIT）：平滑移动双臂并保持最终位置。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.dual_arm.common import (
    joint_degrees,
    scaled_joint_velocities,
    speed_level,
)


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    if args.velocity == 0.0:
        print("速度档位为 0，不执行运动")
        return
    velocity = scaled_joint_velocities(robot.left, args.velocity)
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        print("已进入 MIT 模式")
        print("开始平滑移动")
        robot.move_joint_positions(
            left=joint_degrees(args.left),
            right=joint_degrees(args.right),
            velocity=velocity,
        )
        print("已到达目标位置，按 Ctrl+C 失能并退出")
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
        type=speed_level,
        required=True,
        help="产品速度档位 0～400；400 对应 Yunyi 产品速度曲线",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
