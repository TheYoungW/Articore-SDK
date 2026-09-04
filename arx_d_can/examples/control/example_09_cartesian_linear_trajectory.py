#!/usr/bin/env python3
"""控制示例 09（PV）：指定手臂沿 base_link 横向向外直线移动 15 cm。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm, SafetyState
from arx_d_can.examples.common import positive_duration_s, pose_values, speed_percent


LINE_DISTANCE_M = 0.15

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
    0.381892,
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
    -0.381889,
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
        robot.set_speed_percent(args.speed)
        robot.enable()
        before_sequence = robot.read_state().sequence
        robot.move_linear(
            side=args.side,
            start_pose=args.start,
            end_pose=args.end,
        )
        submitted = True
        deadline = time.monotonic() + args.timeout
        seen_running = False
        while time.monotonic() < deadline:
            state = robot.read_state()
            health = robot.get_health()
            if health.state in {SafetyState.FAULT, SafetyState.SAFE_STOP}:
                detail = (
                    health.last_operation_error
                    or health.safety_reason
                    or health.fault_reason
                    or "未知错误"
                )
                raise RuntimeError(f"直线运动失败：{detail}")
            # The control reply can arrive before the asynchronous RobotState
            # update. Ignore the pre-submit cached `motion_arrived=True`, then
            # require this motion to enter RUNNING before accepting completion.
            if state.sequence > before_sequence:
                if not state.motion_arrived:
                    seen_running = True
                elif seen_running:
                    break
            time.sleep(0.01)
        else:
            raise TimeoutError(f"直线运动在 {args.timeout:g} 秒内未完成")
        print(
            f"\n{args.side} 直线运动完成：模式=PV，速度={args.speed:g}%，"
            f"距离={math.dist(args.start[:3], args.end[:3]) * 100:.1f} cm"
        )
        return
    except Exception:
        if submitted:
            robot.stop_motion()
        raise
    except KeyboardInterrupt:
        if submitted:
            print("\n正在取消当前直线运动……")
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
        "--end",
        type=pose_values,
        default=None,
        help="终点 x,y,z,roll,pitch,yaw（米、弧度）",
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
        help="等待轨迹完成的超时秒数，默认 30",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
