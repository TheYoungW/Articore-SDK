#!/usr/bin/env python3
"""控制示例 09（PV）：用四次板端 Linear 运动画等边三角形。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm, SafetyState
from arx_d_can.examples.common import positive_duration_s, pose_values, speed_percent


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
        robot.set_speed_percent(args.speed)
        robot.enable()
        deadline = time.monotonic() + args.timeout
        for target in path:
            robot.move_linear(side=args.side, end_pose=target)
            submitted = True
            time.sleep(0.01)
            while time.monotonic() < deadline:
                state = robot.read_state()
                if state.motion_arrived:
                    break
                health = robot.get_health()
                if health.state in {SafetyState.FAULT, SafetyState.SAFE_STOP}:
                    detail = (
                        health.last_operation_error
                        or health.safety_reason
                        or health.fault_reason
                        or "未知错误"
                    )
                    raise RuntimeError(f"三角形运动失败：{detail}")
                time.sleep(0.05)
            else:
                raise TimeoutError(f"三角形运动在 {args.timeout:g} 秒内未完成")
        print(
            f"\n{args.side} 三角形执行完成：模式=PV，速度={args.speed:g}%，"
            f"边长={TRIANGLE_SIDE_M * 100:.1f} cm"
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
        "--center",
        type=pose_values,
        default=None,
        help="三角形中心 x,y,z,roll,pitch,yaw（米、弧度）",
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
