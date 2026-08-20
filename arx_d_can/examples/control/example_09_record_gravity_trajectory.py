#!/usr/bin/env python3
"""控制示例 09：双臂同时进入重力补偿并录制示教轨迹。"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.control.example_08_gravity_compensation import (
    _stop_and_close,
)
from arx_d_can.service_tools.dual_trajectory_recording import (
    record,
    save_trajectory,
)


def positive_seconds(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("录制时长必须是有限正数")
    return value


def positive_hz(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("调用频率必须是有限正数")
    return value


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    print("请同时托稳双臂；倒计时后双臂会一起进入重力补偿")
    for remaining in range(3, 0, -1):
        print(f"{remaining} 秒后开始录制……")
        time.sleep(1.0)

    interrupted = False
    try:
        robot.connect()
        robot.enable()
        robot.start_gravity_compensation(transition_ms=500)
        print(
            f"正在录制双臂 {args.seconds:g} 秒，采样频率 {args.hz:g} Hz；"
            "按 Ctrl+C 可安全停止"
        )
        timestamps, samples = record(
            robot,
            seconds=args.seconds,
            hz=args.hz,
        )
    except KeyboardInterrupt:
        print("\n用户中断，未保存不完整轨迹")
        interrupted = True
    finally:
        _stop_and_close(robot)

    if interrupted:
        return

    args.output.parent.mkdir(parents=True, exist_ok=True)
    save_trajectory(
        args.output,
        hz=args.hz,
        timestamps=timestamps,
        samples=samples,
        left_joint_names=robot.joint_names,
        right_joint_names=robot.joint_names,
    )
    print(f"双臂已失能，已保存 {len(samples)} 个轨迹点：{args.output}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="输出双臂轨迹 JSON；父文件夹不存在时自动创建",
    )
    parser.add_argument(
        "--seconds",
        type=positive_seconds,
        default=30.0,
        help="录制时长，单位为秒；默认 30",
    )
    parser.add_argument(
        "--hz",
        type=positive_hz,
        default=100.0,
        help="反馈录制频率；重力控制周期由 Runtime 独立执行，默认 100",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
