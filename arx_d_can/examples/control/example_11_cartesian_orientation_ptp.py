#!/usr/bin/env python3
"""控制示例 11（PV）：用 set_pose 演示 Pitch、Roll、Yaw。"""
from __future__ import annotations

import argparse
import math
import time
from typing import Sequence

from arx_d_can import ArxDCanDualArm
from arx_d_can.examples.common import speed_percent


Pose = tuple[float, float, float, float, float, float]

BASE_LEFT_POSE: Pose = (
    0.403537,
    0.231892,
    0.381638,
    0.0,
    -math.pi / 2.0,
    0.0,
)
BASE_RIGHT_POSE: Pose = (
    0.403537,
    -0.231889,
    0.381639,
    0.0,
    -math.pi / 2.0,
    0.0,
)

# 基准姿态的 pitch=-90° 是 RPY 奇异点。以下端点使用真实旋转矩阵语义，
# 分别绕 base_link 的 X/Y/Z 轴从约 -45° 摆到 +45°。Pitch 端点同步
# 调整 XYZ，以避开 J6 的 ±0.785 rad 产品限位；两端由 J6=±0.78 rad 的
# 已知可达关节姿态 FK 得到，总摆幅约 89.4°。
PITCH_NEGATIVE_LEFT: Pose = (
    0.354893, 0.231892, 0.507978, -math.pi, -0.790796, math.pi,
)
PITCH_NEGATIVE_RIGHT: Pose = (
    0.354893, -0.231889, 0.507979, -math.pi, -0.790796, math.pi,
)
PITCH_POSITIVE_LEFT: Pose = (
    0.349267, 0.231892, 0.257611, 0.0, -0.790796, 0.0,
)
PITCH_POSITIVE_RIGHT: Pose = (
    0.349267, -0.231889, 0.257611, 0.0, -0.790796, 0.0,
)

ROLL_NEGATIVE_LEFT: Pose = (
    *BASE_LEFT_POSE[:3], -math.pi / 2.0, -math.pi / 4.0, math.pi / 2.0,
)
ROLL_NEGATIVE_RIGHT: Pose = (
    *BASE_RIGHT_POSE[:3], -math.pi / 2.0, -math.pi / 4.0, math.pi / 2.0,
)
ROLL_POSITIVE_LEFT: Pose = (
    *BASE_LEFT_POSE[:3], math.pi / 2.0, -math.pi / 4.0, -math.pi / 2.0,
)
ROLL_POSITIVE_RIGHT: Pose = (
    *BASE_RIGHT_POSE[:3], math.pi / 2.0, -math.pi / 4.0, -math.pi / 2.0,
)

YAW_NEGATIVE_LEFT: Pose = (
    *BASE_LEFT_POSE[:3], 0.0, -math.pi / 2.0, -math.pi / 4.0,
)
YAW_NEGATIVE_RIGHT: Pose = (
    *BASE_RIGHT_POSE[:3], 0.0, -math.pi / 2.0, -math.pi / 4.0,
)
YAW_POSITIVE_LEFT: Pose = (
    *BASE_LEFT_POSE[:3], 0.0, -math.pi / 2.0, math.pi / 4.0,
)
YAW_POSITIVE_RIGHT: Pose = (
    *BASE_RIGHT_POSE[:3], 0.0, -math.pi / 2.0, math.pi / 4.0,
)

AXIS_SWEEPS = (
    (
        "Pitch",
        PITCH_NEGATIVE_LEFT,
        PITCH_NEGATIVE_RIGHT,
        PITCH_POSITIVE_LEFT,
        PITCH_POSITIVE_RIGHT,
    ),
    (
        "Roll",
        ROLL_NEGATIVE_LEFT,
        ROLL_NEGATIVE_RIGHT,
        ROLL_POSITIVE_LEFT,
        ROLL_POSITIVE_RIGHT,
    ),
    (
        "Yaw",
        YAW_NEGATIVE_LEFT,
        YAW_NEGATIVE_RIGHT,
        YAW_POSITIVE_LEFT,
        YAW_POSITIVE_RIGHT,
    ),
)


def _rotation_from_rpy(pose: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = pose[3:]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _orientation_error(actual: Sequence[float], target: Sequence[float]) -> float:
    actual_rotation = _rotation_from_rpy(actual)
    target_rotation = _rotation_from_rpy(target)
    relative_trace = sum(
        target_rotation[row][column] * actual_rotation[row][column]
        for row in range(3)
        for column in range(3)
    )
    cosine = max(-1.0, min(1.0, (relative_trace - 1.0) / 2.0))
    return math.acos(cosine)


def _wait_dual_pose(
    robot: ArxDCanDualArm,
    left_target: Pose,
    right_target: Pose,
    *,
    timeout_s: float = 30.0,
) -> None:
    """set_pose 无 motion ID，使用真实位姿和关节速度确认双臂稳定到位。"""
    deadline = time.monotonic() + timeout_s
    stable_since: float | None = None
    while time.monotonic() < deadline:
        left_pose = robot.get_pose("left")
        right_pose = robot.get_pose("right")
        state = robot.read_state()
        position_error = max(
            math.dist(left_pose[:3], left_target[:3]),
            math.dist(right_pose[:3], right_target[:3]),
        )
        orientation_error = max(
            _orientation_error(left_pose, left_target),
            _orientation_error(right_pose, right_target),
        )
        maximum_velocity = max(
            *(abs(value) for value in state.left.arm.velocities),
            *(abs(value) for value in state.right.arm.velocities),
        )
        if (
            position_error <= 0.006
            and orientation_error <= 0.035
            and maximum_velocity <= 0.05
        ):
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= 0.5:
                print(
                    f"已到位：位置误差={position_error * 1000:.2f} mm，"
                    f"姿态误差={math.degrees(orientation_error):.2f}°"
                )
                return
        else:
            stable_since = None
        time.sleep(0.01)
    raise TimeoutError(f"双臂 set_pose 在 {timeout_s:.0f} 秒内未稳定到位")


def _move_and_wait(
    robot: ArxDCanDualArm,
    *,
    left: Pose,
    right: Pose,
    speed: float,
    label: str,
) -> None:
    print(f"提交：{label}")
    robot.set_pose(
        left_target_pose=left,
        right_target_pose=right,
        speed_percent=speed,
    )
    _wait_dual_pose(robot, left, right)


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    try:
        input(
            "双臂将先到 J4=90°、其余关节为0°，随后演示三个姿态轴；"
            "确认周围安全后按回车继续..."
        )
        robot.connect()
        robot.enable()
        _move_and_wait(
            robot,
            left=BASE_LEFT_POSE,
            right=BASE_RIGHT_POSE,
            speed=args.speed,
            label="双臂基准姿态",
        )
        for axis, negative_left, negative_right, positive_left, positive_right in AXIS_SWEEPS:
            input(f"按回车开始 {axis} 约90°往返演示...")
            _move_and_wait(
                robot,
                left=negative_left,
                right=negative_right,
                speed=args.speed,
                label=f"{axis} 负方向端点",
            )
            _move_and_wait(
                robot,
                left=positive_left,
                right=positive_right,
                speed=args.speed,
                label=f"{axis} 正方向端点",
            )
            _move_and_wait(
                robot,
                left=BASE_LEFT_POSE,
                right=BASE_RIGHT_POSE,
                speed=args.speed,
                label=f"{axis} 返回基准姿态",
            )
        input("Pitch、Roll、Yaw 演示完成，按回车失能并退出...")
        robot.disable()
        print("双臂已失能")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.disconnect()
        print("已断开连接")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--speed",
        type=speed_percent,
        default=50.0,
        help="set_pose 普通 PV 速度百分比，范围 [1, 100]，默认 50",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
