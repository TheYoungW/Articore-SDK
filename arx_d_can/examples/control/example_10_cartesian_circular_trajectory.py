#!/usr/bin/env python3
"""控制示例 10（PV）：在 base_link 的 YZ 平面执行半径 10 cm 圆弧轨迹。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm, MotionState
from arx_d_can.examples.common import pose_values, positive_duration_s


CIRCLE_RADIUS_M = 0.10

DEFAULT_LEFT_START_POSE = (
    0.403537,
    0.231892,
    0.281638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_LEFT_VIA_POSE = (
    0.403537,
    0.331892,
    0.381638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_LEFT_END_POSE = (
    0.403537,
    0.231892,
    0.481638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_LEFT_RETURN_VIA_POSE = (
    0.403537,
    0.131892,
    0.381638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_START_POSE = (
    0.403537,
    -0.231889,
    0.281639,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_VIA_POSE = (
    0.403537,
    -0.331889,
    0.381639,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_END_POSE = (
    0.403537,
    -0.231889,
    0.481639,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_RETURN_VIA_POSE = (
    0.403537,
    -0.131889,
    0.381639,
    0.0,
    -1.570796,
    0.0,
)


def _apply_default_poses(args: argparse.Namespace) -> None:
    provided = (
        args.start is not None,
        args.via is not None,
        args.end is not None,
        args.return_via is not None,
    )
    if any(provided) and not all(provided):
        raise ValueError(
            "自定义完整圆路径时必须同时提供 --start、--via、--end 与 --return-via"
        )
    if all(provided):
        return
    if args.side == "left":
        args.start = DEFAULT_LEFT_START_POSE
        args.via = DEFAULT_LEFT_VIA_POSE
        args.end = DEFAULT_LEFT_END_POSE
        args.return_via = DEFAULT_LEFT_RETURN_VIA_POSE
    else:
        args.start = DEFAULT_RIGHT_START_POSE
        args.via = DEFAULT_RIGHT_VIA_POSE
        args.end = DEFAULT_RIGHT_END_POSE
        args.return_via = DEFAULT_RIGHT_RETURN_VIA_POSE


def main(args: argparse.Namespace) -> None:
    _apply_default_poses(args)
    robot = ArxDCanDualArm(control_mode="pv")
    submitted = False
    try:
        robot.connect()
        robot.enable()
        outward_motion_id = robot.move_circular_trajectory(
            side=args.side,
            start_pose=args.start,
            via_pose=args.via,
            end_pose=args.end,
            duration_s=args.duration,
        )
        submitted = True
        try:
            return_motion_id = robot.move_circular_trajectory(
                side=args.side,
                start_pose=args.end,
                via_pose=args.return_via,
                end_pose=args.start,
                duration_s=args.duration,
            )
        except Exception:
            robot.cancel_all_motions()
            raise
        print(
            f"{args.side} 完整圆运动已提交："
            f"motion_ids=[{outward_motion_id}, {return_motion_id}]"
        )
        while True:
            outward_status = robot.get_motion_status(outward_motion_id)
            return_status = robot.get_motion_status(return_motion_id)
            progress = (outward_status.progress + return_status.progress) / 2.0
            print(
                f"\rstate=[{outward_status.state.value}, "
                f"{return_status.state.value}] progress={progress:.1%}",
                end="",
                flush=True,
            )
            if (
                outward_status.state is MotionState.COMPLETED
                and return_status.state is MotionState.COMPLETED
            ):
                print("\n完整圆执行完成，机械臂已回到起点")
                return
            fault = next(
                (
                    (motion_id, status)
                    for motion_id, status in (
                        (outward_motion_id, outward_status),
                        (return_motion_id, return_status),
                    )
                    if status.state is MotionState.FAULT
                ),
                None,
            )
            if fault is not None:
                fault_motion_id, fault_status = fault
                health = robot.get_health()
                detail = (
                    fault_status.error
                    or health.last_operation_error
                    or health.safety_reason
                    or health.fault_reason
                    or "未知错误"
                )
                raise RuntimeError(
                    f"完整圆运动失败：motion_id={fault_motion_id}，{detail}"
                )
            if (
                outward_status.state is MotionState.CANCELLED
                or return_status.state is MotionState.CANCELLED
            ):
                print("\n运动已取消")
                return
            time.sleep(0.05)
    except KeyboardInterrupt:
        if submitted:
            print("\n正在取消当前圆弧运动……")
            robot.cancel_all_motions()
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
        "--via",
        type=pose_values,
        default=None,
        help="圆弧经由点 x,y,z,roll,pitch,yaw（米、弧度）",
    )
    parser.add_argument(
        "--end",
        type=pose_values,
        default=None,
        help="第一段半圆终点 x,y,z,roll,pitch,yaw（米、弧度）",
    )
    parser.add_argument(
        "--return-via",
        type=pose_values,
        default=None,
        help="返回半圆经由点 x,y,z,roll,pitch,yaw（米、弧度）",
    )
    parser.add_argument(
        "--duration",
        type=positive_duration_s,
        required=True,
        help="每段半圆完整任务的预计时间（秒，包含自动接近起点）",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
