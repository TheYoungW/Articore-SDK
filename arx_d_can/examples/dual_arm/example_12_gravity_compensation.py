#!/usr/bin/env python3
"""示例 12：运行双臂 MIT 重力补偿。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm, DualArmGravityCompensationMode
from arx_d_can.examples.dual_arm.common import add_connection_arguments
from arx_d_can.service_tools.gravity_compensation_cli import (
    non_negative_float,
    non_negative_int,
    parse_joint_values,
    positive_float,
)


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(
        transport=args.transport,
        left_channel=args.left_channel,
        right_channel=args.right_channel,
        baud=args.baud,
        control_mode="mit",
    )
    left_scales = (
        None
        if args.left_joint_scales is None
        else parse_joint_values(
            args.left_joint_scales,
            expected_count=len(robot.left.joint_names),
            name="left joint scale",
            allow_negative=True,
        )
    )
    right_scales = (
        None
        if args.right_joint_scales is None
        else parse_joint_values(
            args.right_joint_scales,
            expected_count=len(robot.right.joint_names),
            name="right joint scale",
            allow_negative=True,
        )
    )
    gravity = DualArmGravityCompensationMode(
        robot,
        hz=args.hz,
        transition_seconds=args.transition_seconds,
        settle_seconds=args.settle_seconds,
        gravity_scale=args.gravity_scale,
        left_joint_scales=left_scales,
        right_joint_scales=right_scales,
        damping=args.damping,
    )
    print("请托稳双臂，重力补偿启动后机械臂可以被手动拖动")
    for remaining in range(args.countdown, 0, -1):
        print(f"{remaining} 秒后启动……")
        time.sleep(1.0)
    try:
        gravity.start()
        print("双臂重力补偿已启动，按 Ctrl+C 停止")
        gravity.run(seconds=args.seconds)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        gravity.shutdown()
        print("双臂已失能")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=non_negative_float, default=0.0)
    parser.add_argument("--hz", type=positive_float, default=100.0)
    parser.add_argument("--transition-seconds", type=non_negative_float, default=0.0)
    parser.add_argument("--settle-seconds", type=non_negative_float, default=0.0)
    parser.add_argument("--gravity-scale", type=non_negative_float, default=1.0)
    parser.add_argument("--left-joint-scales")
    parser.add_argument("--right-joint-scales")
    parser.add_argument("--damping", type=non_negative_float, default=0.0)
    parser.add_argument("--countdown", type=non_negative_int, default=3)
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
