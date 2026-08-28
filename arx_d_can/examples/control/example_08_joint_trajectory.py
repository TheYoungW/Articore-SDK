#!/usr/bin/env python3
"""控制示例 08：调用 move_joint_trajectory() 提交双臂原生关节轨迹。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm, MotionState
from arx_d_can.examples.common import (
    joint_degrees,
    joint_values,
    positive_duration_s,
)


DEFAULT_JOINT_TARGET_DEGREES = "0,0,0,90,0,0,0"
DEFAULT_MIT_KP = (190.0, 190.0, 70.0, 125.0, 10.0, 22.0, 28.0)
DEFAULT_MIT_KD = (4.55, 4.5, 2.0, 2.9, 0.7, 0.89, 0.84)
DEFAULT_MIT_FEEDFORWARD_TORQUE = (0.0,) * 7


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode=args.mode)
    motion_id: int | None = None
    try:
        robot.connect()
        robot.enable()
        current = robot.read_state()
        arguments = {
            "timestamps": [0.0, args.duration],
            "left_positions": [
                tuple(current.left.arm.positions),
                joint_degrees(args.left),
            ],
            "right_positions": [
                tuple(current.right.arm.positions),
                joint_degrees(args.right),
            ],
        }
        if args.mode == "mit":
            arguments.update(
                kp=args.mit_kp,
                kd=args.mit_kd,
                feedforward_torque=args.mit_feedforward_torque,
            )
        motion_id = robot.move_joint_trajectory(**arguments)
        print(
            f"原生双臂关节轨迹已提交：motion_id={motion_id}，"
            f"模式={args.mode.upper()}，时长={args.duration:g}s"
        )

        while True:
            status = robot.get_motion_status(motion_id)
            print(
                f"\rstate={status.state.value} progress={status.progress:.1%}",
                end="",
                flush=True,
            )
            if status.state is MotionState.COMPLETED:
                print("\n原生关节轨迹已真实到位")
                try:
                    input("按回车失能并退出...")
                except EOFError:
                    # 非交互式运行没有标准输入；轨迹已完成，不应误报失败。
                    pass
                robot.disable()
                return
            if status.state is MotionState.CANCELLED:
                print("\n原生关节轨迹已取消")
                return
            if status.state is MotionState.FAULT:
                health = robot.get_health()
                detail = (
                    health.last_operation_error
                    or health.safety_reason
                    or health.fault_reason
                    or status.error
                    or "未知错误"
                )
                raise RuntimeError(f"原生关节轨迹失败：{detail}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        if motion_id is not None:
            print("\n正在取消原生关节轨迹……")
            robot.cancel_motion(motion_id)
        else:
            print("\n用户中断")
    finally:
        robot.disconnect()
        print("已失能并断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("pv", "mit"),
        default="pv",
        help="原生关节轨迹控制模式；默认 pv",
    )
    parser.add_argument(
        "--left",
        default=DEFAULT_JOINT_TARGET_DEGREES,
        help="左臂终点角度（7 轴，度）；默认 J4=90，其余为 0",
    )
    parser.add_argument(
        "--right",
        default=DEFAULT_JOINT_TARGET_DEGREES,
        help="右臂终点角度（7 轴，度）；默认 J4=90，其余为 0",
    )
    parser.add_argument(
        "--duration",
        type=positive_duration_s,
        default=5.0,
        help="从当前反馈位置到终点的计划时间，单位为秒；默认 5",
    )
    parser.add_argument(
        "--mit-kp",
        type=lambda text: joint_values(text, name="MIT Kp"),
        default=DEFAULT_MIT_KP,
        help="MIT 7 轴 Kp；默认 190,190,70,125,10,22,28",
    )
    parser.add_argument(
        "--mit-kd",
        type=lambda text: joint_values(text, name="MIT Kd"),
        default=DEFAULT_MIT_KD,
        help="MIT 7 轴 Kd；默认 4.55,4.5,2,2.9,0.7,0.89,0.84",
    )
    parser.add_argument(
        "--mit-feedforward-torque",
        type=lambda text: joint_values(text, name="MIT 前馈力矩"),
        default=DEFAULT_MIT_FEEDFORWARD_TORQUE,
        help="MIT 7 轴前馈力矩，单位 N·m；默认全部为 0",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
