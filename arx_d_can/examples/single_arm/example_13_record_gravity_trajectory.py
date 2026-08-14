#!/usr/bin/env python3
"""示例 13：在单臂重力补偿模式下录制示教轨迹。"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from arx_d_can import ArxDCanArm, GravityCompensationMode
from arx_d_can.examples.single_arm.common import add_connection_arguments
from arx_d_can.service_tools.trajectory_recording import (
    parse_hz,
    record,
    save_trajectory,
)


def positive_seconds(text: str) -> float:
    """解析有限正数录制时长。"""
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("录制时长必须是有限正数")
    return value


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        port=args.port,
        transport=args.transport,
        baud=args.baud,
        control_mode="mit",
        enable_gripper=True,
    )
    gravity = GravityCompensationMode(arm, hz=args.hz)

    print("请托稳机械臂；重力补偿启动后，手动拖动机械臂完成示教")
    for remaining in range(3, 0, -1):
        print(f"{remaining} 秒后开始录制……")
        time.sleep(1.0)

    try:
        with gravity:
            print(
                f"正在录制 {args.seconds:g} 秒，采样频率 {args.hz:g} Hz；"
                "按 Ctrl+C 可安全停止"
            )
            timestamps, positions = record(
                arm,
                seconds=args.seconds,
                hz=args.hz,
                gravity_mode=gravity,
            )
    except KeyboardInterrupt:
        print("\n用户中断，未保存不完整轨迹")
        return

    args.output.parent.mkdir(parents=True, exist_ok=True)
    save_trajectory(
        args.output,
        args.hz,
        positions,
        timestamps=timestamps,
        joint_names=arm.joint_names,
    )
    print(f"机械臂已失能，已保存 {len(positions)} 个轨迹点：{args.output}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="输出 JSON 文件；父文件夹不存在时自动创建",
    )
    parser.add_argument(
        "--seconds",
        type=positive_seconds,
        default=30.0,
        help="录制时长，单位为秒；默认 30",
    )
    parser.add_argument(
        "--hz",
        type=parse_hz,
        default=100.0,
        help="录制和重力补偿更新频率，范围 (0, 500] Hz；默认 100",
    )
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
