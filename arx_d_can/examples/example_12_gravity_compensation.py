#!/usr/bin/env python3
"""示例 12：运行 MIT 重力补偿。"""
from __future__ import annotations

import time

from arx_d_can import ArxDCanArm, GravityCompensationMode
from arx_d_can.service_tools.gravity_compensation_cli import (
    build_parser,
    parse_joint_values,
)


def main(args) -> None:
    arm = ArxDCanArm(
        model=args.arm_model,
        config_path=args.config_path,
        port=args.port,
        baud=args.baud,
        transport=args.transport,
        control_mode="mit",
        enable_gripper=True,
    )
    joint_scales = (
        None
        if args.joint_scales is None
        else parse_joint_values(
            args.joint_scales,
            expected_count=len(arm.joint_names),
            name="joint scale",
            allow_negative=True,
        )
    )
    gravity = GravityCompensationMode(
        arm,
        hz=args.hz,
        transition_seconds=args.transition_seconds,
        settle_seconds=args.settle_seconds,
        gravity_scale=args.gravity_scale,
        joint_scales=joint_scales,
        damping=args.damping,
    )

    print("请托稳机械臂，重力补偿启动后机械臂可以被手动拖动")
    for remaining in range(args.countdown, 0, -1):
        print(f"{remaining} 秒后启动……")
        time.sleep(1.0)

    try:
        gravity.start()
        print("重力补偿已启动，按 Ctrl+C 停止")
        gravity.run(seconds=args.seconds)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        gravity.shutdown()
        print("机械臂已失能")


if __name__ == "__main__":
    main(build_parser().parse_args())
