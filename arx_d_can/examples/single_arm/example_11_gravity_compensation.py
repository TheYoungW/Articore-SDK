#!/usr/bin/env python3
"""示例 11：启动单侧 Yunyi 七轴 Runtime 原生重力补偿。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import (
    ArxDCanArm,
    GravityCompensationPhase,
)
from arx_d_can.examples.single_arm.common import add_connection_arguments


def transition_ms(text: str) -> int:
    value = int(text)
    if not 0 <= value <= 60_000:
        raise argparse.ArgumentTypeError("渐入时间必须在 0..60000 ms 范围内")
    return value


def wait_until_inactive(arm: ArxDCanArm, *, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while arm.gravity_compensation_status.phase is not GravityCompensationPhase.INACTIVE:
        if time.monotonic() >= deadline:
            raise RuntimeError("等待 Runtime 退出重力补偿超时")
        time.sleep(0.02)


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        port=args.port,
        transport=args.transport,
        baud=args.baud,
        control_mode="mit",
        enable_gripper=True,
    )
    print("请托稳机械臂并准备物理急停；倒计时后将进入原生重力补偿")
    for remaining in range(3, 0, -1):
        print(f"{remaining} 秒后启动……")
        time.sleep(1.0)

    arm.connect()
    try:
        arm.enable()
        arm.start_gravity_compensation(transition_ms=args.transition_ms)
        print("重力补偿已启动，可手动拖动；按 Ctrl+C 停止")
        while True:
            status = arm.gravity_compensation_status
            print(
                f"phase={status.phase.name} progress={status.transition_progress:.3f} "
                f"cycles={status.control_cycles}",
                end="\r",
                flush=True,
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        if arm.connected and arm.enabled:
            arm.stop_gravity_compensation()
            wait_until_inactive(arm)
        arm.close()
        print("机械臂已失能并断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--transition-ms",
        type=transition_ms,
        default=1000,
        help="渐入和渐出时间，单位 ms；0 使用 Runtime 默认 500 ms，默认 1000",
    )
    add_connection_arguments(parser)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
