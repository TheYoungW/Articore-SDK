#!/usr/bin/env python3
"""控制示例 04（标准 MIT）：显式提交 q/dq/Kp/Kd/tau_ff。"""
from __future__ import annotations

import argparse
import math

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import joint_degrees, joint_values


DEFAULT_MAX_TOTAL_DELTA_DEGREES = 20.0
DEFAULT_KP = (40.0, 40.0, 35.0, 30.0, 25.0, 20.0, 15.0)
DEFAULT_KD = (2.0, 2.0, 1.8, 1.5, 1.2, 1.0, 0.8)


def _positive_number(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("必须是正的有限数值")
    return value


def _gain_values(text: str, *, name: str, maximum: float) -> tuple[float, ...]:
    values = joint_values(text, name=name)
    if any(not 0.0 <= value <= maximum for value in values):
        raise argparse.ArgumentTypeError(
            f"{name}的 7 个值必须全部在 [0, {maximum:g}] 范围内"
        )
    return values


def _kp_values(text: str) -> tuple[float, ...]:
    return _gain_values(text, name="Kp", maximum=500.0)


def _kd_values(text: str) -> tuple[float, ...]:
    return _gain_values(text, name="Kd", maximum=5.0)


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.connect()
    print("机器人连接成功")
    enabled = False
    try:
        target_left = joint_degrees(args.left)
        target_right = joint_degrees(args.right)
        current = robot.read_state()
        current_positions = (*current.left.positions, *current.right.positions)
        target_positions = (*target_left, *target_right)
        maximum_total_delta_degrees = math.degrees(max(
            abs(final - initial)
            for initial, final in zip(
                current_positions, target_positions, strict=True
            )
        ))
        if maximum_total_delta_degrees > args.max_total_delta_deg:
            raise ValueError(
                "目标与当前反馈的最大关节差值为 "
                f"{maximum_total_delta_degrees:.2f}°，超过 example 的 "
                f"{args.max_total_delta_deg:g}° 安全上限"
            )
        print(
            "将一次提交完整双臂标准 MIT 目标；"
            f"当前反馈到目标的最大关节变化为 {maximum_total_delta_degrees:.2f}°。"
        )
        print(
            f"标准 MIT 显式使用单臂 7 轴 Kp={args.kp}、Kd={args.kd}，"
            "dq=0、tau_ff=0；新帧原子覆盖旧帧。"
        )
        input("确认机器人周围安全后按回车开始...")
        if not robot.enable():
            raise RuntimeError("机器人使能失败")
        enabled = True
        print("已进入标准 MIT 模式")
        robot.set_joint_mit(
            left_positions=target_left,
            right_positions=target_right,
            left_velocities=(0.0,) * 7,
            right_velocities=(0.0,) * 7,
            kp=args.kp,
            kd=args.kd,
            left_feedforward_torques=(0.0,) * 7,
            right_feedforward_torques=(0.0,) * 7,
        )
        print("标准 MIT 完整目标已提交一次；即将失能。")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        try:
            if enabled:
                robot.disable()
                print("双臂已失能")
        finally:
            robot.disconnect()
            print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True, help="左臂 7 个关节角度，单位为度")
    parser.add_argument("--right", required=True, help="右臂 7 个关节角度，单位为度")
    parser.add_argument(
        "--kp", type=_kp_values, default=DEFAULT_KP,
        help=(
            "单臂 J1..J7 位置增益，以逗号分隔；每项范围 [0, 500]；"
            "默认 40,40,35,30,25,20,15"
        ),
    )
    parser.add_argument(
        "--kd", type=_kd_values, default=DEFAULT_KD,
        help=(
            "单臂 J1..J7 速度增益，以逗号分隔；每项范围 [0, 5]；"
            "默认 2,2,1.8,1.5,1.2,1,0.8"
        ),
    )
    parser.add_argument(
        "--max-total-delta-deg",
        type=_positive_number,
        default=DEFAULT_MAX_TOTAL_DELTA_DEGREES,
        help="目标相对当前反馈允许的最大单关节总变化，单位为度；默认 20",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
