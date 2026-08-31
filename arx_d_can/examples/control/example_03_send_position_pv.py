#!/usr/bin/env python3
"""控制示例 03（PV）：提交双臂普通位置目标。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import joint_degrees, speed_percent


DEFAULT_JOINT_TARGET_DEGREES = "0,0,0,90,0,0,0"


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    robot.connect()
    print("机器人连接成功")
    try:
        if args.max_speed is not None:
            robot.set_max_speed(args.max_speed)
            print(f"普通 PV 最大速度基础上限：{robot.get_max_speed():g} rad/s")
        if args.max_acceleration is not None:
            robot.set_max_acceleration(args.max_acceleration)
            print(
                "普通 PV 最大加速度基础上限："
                f"{robot.get_max_acceleration():g} rad/s²"
            )
        robot.enable()
        print("已进入 PV 模式")
        robot.set_joint_pv(
            left=joint_degrees(args.left),
            right=joint_degrees(args.right),
            velocity=args.velocity,
        )
        input("目标已提交，确认双臂到位后按回车失能并退出...")
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
        "--left",
        default=DEFAULT_JOINT_TARGET_DEGREES,
        help="左臂 7 个关节角度，单位为度；默认 J4=90，其余为 0",
    )
    parser.add_argument(
        "--right",
        default=DEFAULT_JOINT_TARGET_DEGREES,
        help="右臂 7 个关节角度，单位为度；默认 J4=90，其余为 0",
    )
    parser.add_argument(
        "--velocity",
        type=speed_percent,
        default=50.0,
        help="本次 PV 命令速度百分比，范围 1～100，默认 50",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        help="可选：普通 PV 在 100%% 时的全局速度基础上限，rad/s；0 恢复默认",
    )
    parser.add_argument(
        "--max-acceleration",
        type=float,
        help="可选：普通 PV 在 100%% 时的全局加速度基础上限，rad/s²；0 恢复默认",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
