#!/usr/bin/env python3
"""控制示例 04（标准 MIT）：显式提交 q/dq/Kp/Kd/tau_ff。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import joint_degrees


DEFAULT_MAX_STEP_DEGREES = 2.0
DEFAULT_MAX_TOTAL_DELTA_DEGREES = 20.0
DEFAULT_STEP_INTERVAL_SECONDS = 0.5
DEFAULT_STREAM_HZ = 100.0


def _positive_number(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("必须是正的有限数值")
    return value


def _staged_targets(
    current_left: tuple[float, ...],
    current_right: tuple[float, ...],
    target_left: tuple[float, ...],
    target_right: tuple[float, ...],
    max_step_degrees: float,
) -> tuple[tuple[tuple[float, ...], tuple[float, ...]], ...]:
    """从当前反馈到最终目标生成等比例小步目标。"""
    current = (*current_left, *current_right)
    target = (*target_left, *target_right)
    maximum_delta = max(
        abs(final - initial) for initial, final in zip(current, target, strict=True)
    )
    step_count = max(
        1,
        math.ceil(maximum_delta / math.radians(max_step_degrees)),
    )
    stages = []
    for index in range(1, step_count + 1):
        amount = index / step_count
        values = tuple(
            initial + (final - initial) * amount
            for initial, final in zip(current, target, strict=True)
        )
        stages.append((values[:7], values[7:]))
    return tuple(stages)


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
        stages = _staged_targets(
            tuple(current.left.positions),
            tuple(current.right.positions),
            target_left,
            target_right,
            args.max_step_deg,
        )
        print(
            f"将从当前反馈分 {len(stages)} 段提交目标，"
            f"单段最大关节变化不超过 {args.max_step_deg:g}°，"
            f"段间等待 {args.step_interval:g} s。"
        )
        print(
            f"标准 MIT 显式使用 Kp={args.kp:g}、Kd={args.kd:g}，"
            "dq=0、tau_ff=0；新帧原子覆盖旧帧。"
        )
        input("确认机器人周围安全后按回车开始...")
        if not robot.enable():
            raise RuntimeError("机器人使能失败")
        enabled = True
        print("已进入标准 MIT 模式")
        repeats = max(1, math.ceil(args.step_interval * args.stream_hz))
        for index, (left, right) in enumerate(stages, start=1):
            for _ in range(repeats):
                robot.set_joint_mit(
                    left_positions=left,
                    right_positions=right,
                    left_velocities=(0.0,) * 7,
                    right_velocities=(0.0,) * 7,
                    kp=args.kp,
                    kd=args.kd,
                    left_feedforward_torques=(0.0,) * 7,
                    right_feedforward_torques=(0.0,) * 7,
                )
                time.sleep(1.0 / args.stream_hz)
            print(f"已提交第 {index}/{len(stages)} 段")
        print(
            "标准 MIT 流式演示完成；即将失能。"
        )
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        if enabled:
            robot.disable()
            print("双臂已失能")
        robot.disconnect()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True, help="左臂 7 个关节角度，单位为度")
    parser.add_argument("--right", required=True, help="右臂 7 个关节角度，单位为度")
    parser.add_argument(
        "--kp", type=float, required=True,
        help="显式 MIT 位置增益，范围 [0, 500]",
    )
    parser.add_argument(
        "--kd", type=float, required=True,
        help="显式 MIT 速度增益，范围 [0, 5]",
    )
    parser.add_argument(
        "--max-step-deg",
        type=_positive_number,
        default=DEFAULT_MAX_STEP_DEGREES,
        help="相邻提交目标的最大单关节变化，单位为度；默认 2",
    )
    parser.add_argument(
        "--max-total-delta-deg",
        type=_positive_number,
        default=DEFAULT_MAX_TOTAL_DELTA_DEGREES,
        help="目标相对当前反馈允许的最大单关节总变化，单位为度；默认 20",
    )
    parser.add_argument(
        "--step-interval",
        type=_positive_number,
        default=DEFAULT_STEP_INTERVAL_SECONDS,
        help="相邻目标的等待时间，单位为秒；默认 0.5",
    )
    parser.add_argument(
        "--stream-hz",
        type=_positive_number,
        default=DEFAULT_STREAM_HZ,
        help="标准 MIT 帧重复发送频率，单位 Hz；默认 100",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
