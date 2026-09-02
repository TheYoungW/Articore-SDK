#!/usr/bin/env python3
"""用 move_pose 依次验证 Linear 三角形示例的三个顶点。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm, SafetyState
from arx_d_can.examples.common import positive_duration_s, speed_percent
from arx_d_can.examples.control.example_09_cartesian_linear_trajectory import (
    DEFAULT_LEFT_CENTER_POSE,
    DEFAULT_RIGHT_CENTER_POSE,
    TRIANGLE_SIDE_M,
    _triangle_vertices,
)


def _rotation_from_rpy(pose: tuple[float, ...] | list[float]):
    roll, pitch, yaw = pose[3:]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _orientation_error(
    expected: tuple[float, ...], actual: list[float]
) -> float:
    expected_rotation = _rotation_from_rpy(expected)
    actual_rotation = _rotation_from_rpy(actual)
    trace = sum(
        expected_rotation[row][column] * actual_rotation[row][column]
        for row in range(3)
        for column in range(3)
    )
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.acos(cosine)


def _require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    if health.state in {
        SafetyState.FAULT,
        SafetyState.SAFE_STOP,
        SafetyState.SAFE_HOLD,
    }:
        detail = (
            health.last_operation_error
            or health.safety_reason
            or health.fault_reason
            or "未知 Runtime 错误"
        )
        raise RuntimeError(f"Runtime state={health.state.name}: {detail}")


def _wait_pose(
    robot: ArxDCanDualArm,
    side: str,
    target: tuple[float, ...],
    timeout_s: float,
) -> tuple[list[float], float, float, float]:
    deadline = time.monotonic() + timeout_s
    stable_samples = 0
    while time.monotonic() < deadline:
        _require_healthy(robot)
        state = robot.read_state()
        actual = robot.get_pose(side)
        arm = getattr(state, side).arm
        position_error = math.dist(actual[:3], target[:3])
        orientation_error = _orientation_error(target, actual)
        maximum_velocity = max(abs(value) for value in arm.velocities)
        if (
            state.motion_arrived
            and position_error <= 0.005
            and orientation_error <= 0.035
            and maximum_velocity <= 0.05
        ):
            stable_samples += 1
            if stable_samples >= 25:
                return (
                    actual,
                    position_error,
                    orientation_error,
                    maximum_velocity,
                )
        else:
            stable_samples = 0
        time.sleep(0.002)
    raise TimeoutError(f"{side} 未在 {timeout_s:.1f}s 内稳定到达目标")


def main(args: argparse.Namespace) -> None:
    center = (
        DEFAULT_LEFT_CENTER_POSE
        if args.side == "left"
        else DEFAULT_RIGHT_CENTER_POSE
    )
    vertices = _triangle_vertices(center, args.side)
    print(
        f"将以 move_pose 依次运动到 {args.side} 三角形的3个顶点，"
        f"边长={TRIANGLE_SIDE_M * 100:.1f} cm："
    )
    for index, pose in enumerate(vertices, start=1):
        print(f"  point{index}: [{', '.join(f'{value:.6f}' for value in pose)}]")
    input("确认周围安全后按回车连接并使能机械臂...")

    robot = ArxDCanDualArm(control_mode="pv")
    enabled = False
    try:
        robot.connect()
        _require_healthy(robot)
        if not robot.enable():
            raise RuntimeError("整机使能未得到确认")
        enabled = True

        for index, target in enumerate(vertices, start=1):
            input(f"按回车发送 point{index}...")
            robot.set_speed_percent(args.speed)
            robot.move_pose(side=args.side, target_pose=target)
            actual, position_error, orientation_error, maximum_velocity = _wait_pose(
                robot,
                args.side,
                target,
                args.timeout,
            )
            print(
                f"point{index} 已稳定到位：actual="
                f"[{', '.join(f'{value:.6f}' for value in actual)}]，"
                f"位置误差={position_error * 1000:.2f} mm，"
                f"姿态误差={orientation_error:.5f} rad，"
                f"最大关节速度={maximum_velocity:.4f} rad/s"
            )

        input("3个顶点测试完成；观察结束后按回车失能...")
    finally:
        if enabled:
            robot.disable()
        robot.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--speed",
        type=speed_percent,
        default=50.0,
        help="move_pose 轨迹速度百分比，默认50",
    )
    parser.add_argument(
        "--timeout",
        type=positive_duration_s,
        default=20.0,
        help="每个顶点的反馈到位超时秒数，默认20",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
