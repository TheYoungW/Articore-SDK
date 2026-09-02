#!/usr/bin/env python3
"""控制示例 10（PV）：在 base_link 的 YZ 平面执行半径 10 cm 圆弧轨迹。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm, SafetyState
from arx_d_can.examples.common import positive_duration_s, pose_values, speed_percent


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
        robot.set_speed_percent(args.speed)
        robot.enable()
        robot.move_circular(
            side=args.side,
            start_pose=args.start,
            via_pose=args.via,
            end_pose=args.end,
        )
        submitted = True
        print(
            f"{args.side} 第一段半圆已提交，速度={args.speed:g}%"
        )
        for stage in ("outward", "return"):
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                if robot.read_state().motion_arrived:
                    break
                health = robot.get_health()
                if health.state in {SafetyState.FAULT, SafetyState.SAFE_STOP}:
                    detail = (
                        health.last_operation_error
                        or health.safety_reason
                        or health.fault_reason
                        or "未知错误"
                    )
                    raise RuntimeError(f"完整圆运动失败：{detail}")
                time.sleep(0.05)
            else:
                raise TimeoutError(
                    f"{stage} 圆弧在 {args.timeout:g} 秒内未完成"
                )
            if stage == "outward":
                robot.move_circular(
                    side=args.side,
                    start_pose=args.end,
                    via_pose=args.return_via,
                    end_pose=args.start,
                )
                print("第一段已到达，返回半圆已提交")
        print("完整圆执行完成，机械臂已回到起点")
    except Exception:
        if submitted:
            robot.stop_motion()
        raise
    except KeyboardInterrupt:
        if submitted:
            print("\n正在取消当前圆弧运动……")
            robot.stop_motion()
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
        type=speed_percent,
        default=50.0,
        help="Runtime 共享速度百分比，范围 1～100，默认 50",
    )
    parser.add_argument(
        "--timeout",
        type=positive_duration_s,
        default=30.0,
        help="等待每段圆弧完成的超时秒数，默认 30",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
