#!/usr/bin/env python3
"""控制示例 07-3（PV）：指定手臂在 base_link 的 YZ 平面执行完整圆运动。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm, CartesianMotionState
from arx_d_can.examples.common import pose_values, positive_speed_percent


DEFAULT_LEFT_START_POSE = (
    0.403537,
    0.231892,
    0.301638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_LEFT_VIA_POSE = (
    0.403537,
    0.311892,
    0.381638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_LEFT_END_POSE = (
    0.403537,
    0.231892,
    0.461638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_LEFT_RETURN_VIA_POSE = (
    0.403537,
    0.151892,
    0.381638,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_START_POSE = (
    0.403537,
    -0.231889,
    0.301639,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_VIA_POSE = (
    0.403537,
    -0.311889,
    0.381639,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_END_POSE = (
    0.403537,
    -0.231889,
    0.461639,
    0.0,
    -1.570796,
    0.0,
)
DEFAULT_RIGHT_RETURN_VIA_POSE = (
    0.403537,
    -0.151889,
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
        outward_motion_id = robot.move_circular(
            side=args.side,
            start_pose=args.start,
            via_pose=args.via,
            end_pose=args.end,
            speed_percent=args.speed,
        )
        submitted = True
        try:
            return_motion_id = robot.move_circular(
                side=args.side,
                start_pose=args.end,
                via_pose=args.return_via,
                end_pose=args.start,
                speed_percent=args.speed,
            )
        except Exception:
            robot.cancel_cartesian_motion()
            raise
        print(
            f"{args.side} 完整圆运动已提交："
            f"motion_ids=[{outward_motion_id}, {return_motion_id}]"
        )
        while True:
            outward_status = robot.get_cartesian_motion_status(outward_motion_id)
            return_status = robot.get_cartesian_motion_status(return_motion_id)
            progress = (outward_status.progress + return_status.progress) / 2.0
            print(
                f"\rstate=[{outward_status.state.value}, "
                f"{return_status.state.value}] progress={progress:.1%}",
                end="",
                flush=True,
            )
            if (
                outward_status.state is CartesianMotionState.COMPLETED
                and return_status.state is CartesianMotionState.COMPLETED
            ):
                print("\n完整圆执行完成，机械臂已回到起点")
                return
            if (
                outward_status.state is CartesianMotionState.CANCELLED
                or return_status.state is CartesianMotionState.CANCELLED
            ):
                print("\n运动已取消")
                return
            fault_status = next(
                (
                    status
                    for status in (outward_status, return_status)
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
                raise RuntimeError(f"完整圆运动失败：{detail}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        if submitted:
            print("\n正在取消当前圆弧运动……")
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
        "--speed",
        type=positive_speed_percent,
        required=True,
        help="速度百分比，范围 (0, 100]",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
