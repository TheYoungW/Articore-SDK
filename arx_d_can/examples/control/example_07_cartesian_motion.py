#!/usr/bin/env python3
"""控制示例 07（PV）：执行单侧原生点到点、直线或三点圆弧运动。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm, CartesianMotionState
from arx_d_can.examples.common import pose_values, positive_speed_percent


def _start_motion(robot: ArxDCanDualArm, args: argparse.Namespace) -> int:
    if args.motion == "ptp":
        return robot.move_pose(
            side=args.side,
            target_pose=args.target,
            speed_percent=args.speed,
        )
    if args.motion == "linear":
        return robot.move_linear(
            side=args.side,
            target_pose=args.target,
            speed_percent=args.speed,
        )
    return robot.move_circular(
        side=args.side,
        via_pose=args.via,
        end_pose=args.target,
        speed_percent=args.speed,
    )


def main(args: argparse.Namespace) -> None:
    if args.motion == "circular" and args.via is None:
        raise ValueError("圆弧运动必须提供 --via")

    robot = ArxDCanDualArm(control_mode="pv")
    submitted = False
    try:
        robot.connect()
        robot.enable()
        motion_id = _start_motion(robot, args)
        submitted = True
        print(f"运动已提交：motion_id={motion_id}")
        while True:
            status = robot.cartesian_motion_status
            print(
                f"\rstate={status.state.value} progress={status.progress:.1%}",
                end="",
                flush=True,
            )
            if status.state is CartesianMotionState.COMPLETED:
                print("\n机械臂已真实到位")
                return
            if status.state is CartesianMotionState.CANCELLED:
                print("\n运动已取消")
                return
            if status.state is CartesianMotionState.FAULT:
                health = robot.get_health()
                detail = (
                    health.last_operation_error
                    or health.safety_reason
                    or health.fault_reason
                    or status.error
                    or "未知错误"
                )
                raise RuntimeError(f"笛卡尔运动失败：{detail}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        if submitted:
            print("\n正在取消当前笛卡尔运动……")
            robot.cancel_cartesian_motion()
        else:
            print("\n用户中断")
    finally:
        robot.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--motion", choices=("ptp", "linear", "circular"), required=True
    )
    parser.add_argument(
        "--target", type=pose_values, required=True,
        help="目标/圆弧终点 x,y,z,roll,pitch,yaw（米、弧度）",
    )
    parser.add_argument(
        "--via", type=pose_values,
        help="圆弧中间点 x,y,z,roll,pitch,yaw",
    )
    parser.add_argument(
        "--speed", type=positive_speed_percent, required=True,
        help="速度百分比，范围 (0, 100]",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
