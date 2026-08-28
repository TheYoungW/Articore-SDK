#!/usr/bin/env python3
"""控制示例 09（PV）：用一条自动圆角融合的 Linear 轨迹画等边三角形。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm, MotionState
from arx_d_can.examples.common import pose_values, positive_duration_s


TRIANGLE_SIDE_M = 0.14

DEFAULT_LEFT_CENTER_POSE = (
    0.403537,
    0.231892,
    0.381638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_CENTER_POSE = (
    0.403537,
    -0.231889,
    0.381639,
    0.0,
    -1.570796,
    0.0,
)


def _apply_default_center(args: argparse.Namespace) -> None:
    if args.center is None:
        args.center = (
            DEFAULT_LEFT_CENTER_POSE
            if args.side == "left"
            else DEFAULT_RIGHT_CENTER_POSE
        )


def _triangle_vertices(
    center: tuple[float, ...], side: str
) -> tuple[tuple[float, ...], ...]:
    x, y, z, roll, pitch, yaw = center
    outward_sign = 1.0 if side == "left" else -1.0
    circumradius = TRIANGLE_SIDE_M / math.sqrt(3.0)
    inner_y = y - outward_sign * circumradius / 2.0
    orientation = (roll, pitch, yaw)
    return (
        (x, y + outward_sign * circumradius, z, *orientation),
        (x, inner_y, z + TRIANGLE_SIDE_M / 2.0, *orientation),
        (x, inner_y, z - TRIANGLE_SIDE_M / 2.0, *orientation),
    )


def main(args: argparse.Namespace) -> None:
    _apply_default_center(args)
    vertices = _triangle_vertices(args.center, args.side)
    path = vertices + vertices[:1]
    robot = ArxDCanDualArm(control_mode="pv")
    submitted = False
    try:
        robot.connect()
        robot.enable()
        try:
            motion_id = robot.move_linear_trajectory(
                side=args.side,
                poses=path,
                duration_s=args.duration,
            )
            submitted = True
        except Exception:
            if submitted:
                robot.cancel_all_motions()
            raise
        print(
            f"{args.side} 等边三角形已提交：motion_id={motion_id}，"
            f"模式=PV 50，默认圆角=10 mm，"
            f"边长={TRIANGLE_SIDE_M * 100:.1f} cm"
        )
        while True:
            status = robot.get_motion_status(motion_id)
            print(
                f"\rstate={status.state.value} progress={status.progress:.1%}",
                end="",
                flush=True,
            )
            if status.state is MotionState.COMPLETED:
                print("\n三角形执行完成，机械臂已回到第一个顶点")
                return
            if status.state is MotionState.FAULT:
                health = robot.get_health()
                detail = (
                    status.error
                    or health.last_operation_error
                    or health.safety_reason
                    or health.fault_reason
                    or "未知错误"
                )
                raise RuntimeError(
                    f"三角形运动失败：motion_id={motion_id}，{detail}"
                )
            if status.state is MotionState.CANCELLED:
                print("\n运动已取消")
                return
            time.sleep(0.05)
    except KeyboardInterrupt:
        if submitted:
            print("\n正在取消当前直线运动……")
            robot.cancel_all_motions()
        else:
            print("\n用户中断")
    finally:
        robot.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--center",
        type=pose_values,
        default=None,
        help="三角形中心 x,y,z,roll,pitch,yaw（米、弧度）",
    )
    parser.add_argument(
        "--duration",
        type=positive_duration_s,
        required=True,
        help="每条原始边的参考时间（秒；整条路径统一五次时间律和10 mm圆角）",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
