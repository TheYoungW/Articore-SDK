#!/usr/bin/env python3
"""控制示例 07（PV）：双臂 Pose 经一次 IK 后执行普通关节点到点。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import pose_values, speed_percent


DEFAULT_LEFT_TARGET_POSE = (
    0.403537,
    0.231892,
    0.381638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_TARGET_POSE = (
    0.403537,
    -0.231889,
    0.381639,
    0.0,
    -1.570796,
    0.0,
)


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    try:
        robot.connect()
        left_q, right_q = robot.solve_ik(
            left_target_pose=args.left_target,
            right_target_pose=args.right_target,
        )
        print("双臂 IK 已求解；solve_ik() 未使能电机、未提交运动")
        robot.enable()
        robot.set_joint_pv(
            left=left_q,
            right=right_q,
            velocity=args.speed,
        )
        input("双臂普通 PV 关节点到点目标已提交；按回车失能并退出...")
        robot.disable()
        print("双臂已失能")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.disconnect()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--left-target",
        type=pose_values,
        default=DEFAULT_LEFT_TARGET_POSE,
        help="左臂目标 x,y,z,roll,pitch,yaw（米、弧度）",
    )
    parser.add_argument(
        "--right-target",
        type=pose_values,
        default=DEFAULT_RIGHT_TARGET_POSE,
        help="右臂目标 x,y,z,roll,pitch,yaw（米、弧度）",
    )
    parser.add_argument(
        "--speed",
        type=speed_percent,
        default=50.0,
        help="普通 PV 速度百分比，范围 [0, 100]，默认 50（1 rad/s）",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
