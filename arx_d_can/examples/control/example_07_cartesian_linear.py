#!/usr/bin/env python3
"""控制示例 07-2（PV）：指定手臂沿 base_link 横向向外直线移动 10 cm。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm, CartesianMotionState
from arx_d_can.examples.common import pose_values, positive_speed_percent


DEFAULT_LEFT_START_POSE = (
    0.403537,
    0.231892,
    0.381638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_LEFT_END_POSE = (
    0.403537,
    0.331892,
    0.381638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_START_POSE = (
    0.403537,
    -0.231889,
    0.381639,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_END_POSE = (
    0.403537,
    -0.331889,
    0.381639,
    0.0,
    -1.570796,
    0.0,
)


def _apply_default_poses(args: argparse.Namespace) -> None:
    if (args.start is None) != (args.end is None):
        raise ValueError("自定义直线路径时必须同时提供 --start 与 --end")
    if args.start is not None:
        return
    if args.side == "left":
        args.start = DEFAULT_LEFT_START_POSE
        args.end = DEFAULT_LEFT_END_POSE
    else:
        args.start = DEFAULT_RIGHT_START_POSE
        args.end = DEFAULT_RIGHT_END_POSE


def main(args: argparse.Namespace) -> None:
    _apply_default_poses(args)
    robot = ArxDCanDualArm(control_mode="pv")
    submitted = False
    try:
        robot.connect()
        robot.enable()
        motion_id = robot.move_linear(
            side=args.side,
            start_pose=args.start,
            end_pose=args.end,
            speed_percent=args.speed,
        )
        submitted = True
        print(f"{args.side} 直线运动已提交：motion_id={motion_id}")
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
                raise RuntimeError(f"直线运动失败：{detail}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        if submitted:
            print("\n正在取消当前直线运动……")
            robot.cancel_cartesian_motion()
        else:
            print("\n用户中断")
    finally:
        robot.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--start",
        type=pose_values,
        default=None,
        help="显式起点 x,y,z,roll,pitch,yaw（米、弧度）",
    )
    parser.add_argument(
        "--end",
        type=pose_values,
        default=None,
        help="终点 x,y,z,roll,pitch,yaw（米、弧度）",
    )
    parser.add_argument(
        "--speed",
        type=positive_speed_percent,
        required=True,
        help="速度百分比，范围 (0, 100]",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
