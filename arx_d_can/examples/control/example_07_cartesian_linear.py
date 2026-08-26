#!/usr/bin/env python3
"""控制示例 07-2（PV）：指定手臂用三段 Linear 画边长 7 cm 的等边三角形。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm, CartesianMotionState
from arx_d_can.examples.common import pose_values, positive_speed_percent


TRIANGLE_SIDE_M = 0.07

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
    edges = tuple(zip(vertices, vertices[1:] + vertices[:1]))
    robot = ArxDCanDualArm(control_mode="pv")
    submitted = False
    try:
        robot.connect()
        robot.enable()
        motion_ids: list[int] = []
        try:
            for start_pose, end_pose in edges:
                motion_ids.append(
                    robot.move_linear(
                        side=args.side,
                        start_pose=start_pose,
                        end_pose=end_pose,
                        speed_percent=args.speed,
                    )
                )
                submitted = True
        except Exception:
            if submitted:
                robot.cancel_cartesian_motion()
            raise
        print(
            f"{args.side} 等边三角形已提交：motion_ids={motion_ids}，"
            f"边长={TRIANGLE_SIDE_M * 100:.1f} cm"
        )
        while True:
            statuses = [
                robot.get_cartesian_motion_status(motion_id)
                for motion_id in motion_ids
            ]
            progress = sum(status.progress for status in statuses) / len(statuses)
            print(
                f"\rstate={[status.state.value for status in statuses]} "
                f"progress={progress:.1%}",
                end="",
                flush=True,
            )
            if all(
                status.state is CartesianMotionState.COMPLETED
                for status in statuses
            ):
                print("\n三角形执行完成，机械臂已回到第一个顶点")
                return
            if any(
                status.state is CartesianMotionState.CANCELLED
                for status in statuses
            ):
                print("\n运动已取消")
                return
            fault_status = next(
                (
                    status
                    for status in statuses
                    if status.state is CartesianMotionState.FAULT
                ),
                None,
            )
            if fault_status is not None:
                health = robot.get_health()
                detail = (
                    health.last_operation_error
                    or health.safety_reason
                    or health.fault_reason
                    or fault_status.error
                    or "未知错误"
                )
                raise RuntimeError(f"三角形运动失败：{detail}")
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
        "--center",
        type=pose_values,
        default=None,
        help="三角形中心 x,y,z,roll,pitch,yaw（米、弧度）",
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
