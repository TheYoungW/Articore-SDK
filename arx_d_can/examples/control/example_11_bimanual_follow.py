#!/usr/bin/env python3
"""控制示例 11：普通 PV/MIT 控制一侧，另一侧保持启动时相对关系。"""
from __future__ import annotations

import argparse
import math
import time

from arx_d_can import ArxDCanDualArm, BimanualFollowPhase


SAFE_POSE = (0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0)


def _send(
    robot: ArxDCanDualArm,
    mode: str,
    leader: str,
    leader_target: tuple[float, ...],
    speed: float,
) -> None:
    # 跟随启动后，Runtime 只采用主臂目标；这里传入的从臂 SAFE_POSE
    # 仅用于保持普通双臂 API 形状，不会覆盖底层生成的从臂跟随目标。
    left = leader_target if leader == "left" else SAFE_POSE
    right = leader_target if leader == "right" else SAFE_POSE
    if mode == "pv":
        robot.set_joint_pv(left=left, right=right)
    else:
        robot.set_joint_mit(left=left, right=right, velocity=speed)


def _wait_target(
    robot: ArxDCanDualArm,
    leader: str,
    target: tuple[float, ...],
    position_tolerance: float,
    timeout_s: float = 20.0,
) -> None:
    deadline = time.monotonic() + timeout_s
    stable_since: float | None = None
    while time.monotonic() < deadline:
        state = robot.read_state()
        arm = state.left.arm if leader == "left" else state.right.arm
        arrived = (
            max(abs(a - b) for a, b in zip(arm.positions, target))
            <= position_tolerance
            and max(abs(value) for value in arm.velocities) <= 0.05
        )
        if arrived:
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= 0.25:
                return
        else:
            stable_since = None
        health = robot.get_health()
        if health.safe_stopped or health.fault_reason:
            raise RuntimeError(
                health.safety_reason or health.fault_reason or
                "双臂协同进入安全停机"
            )
        time.sleep(0.01)
    raise RuntimeError("主臂未在超时内稳定到达目标")


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode=args.mode)
    started = False
    try:
        robot.connect()
        robot.enable()
        if args.mode == "pv":
            robot.set_max_speed(args.speed)

        if args.mode == "pv":
            robot.set_joint_pv(left=SAFE_POSE, right=SAFE_POSE)
        else:
            robot.set_joint_mit(
                left=SAFE_POSE, right=SAFE_POSE, velocity=args.speed
            )
        tolerance = 0.05 if args.mode == "pv" else 0.06
        _wait_target(robot, args.leader, SAFE_POSE, tolerance)

        # 此刻的左右相对关节位置由 Runtime 原子记录。
        robot.start_bimanual_follow(leader=args.leader)
        started = True
        if robot.bimanual_follow_status.phase is not BimanualFollowPhase.ACTIVE:
            raise RuntimeError("双臂协同未立即进入 ACTIVE")

        target = list(SAFE_POSE)
        target[0] += math.radians(args.delta_deg)
        leader_target = tuple(target)
        print(
            f"{args.mode.upper()} 普通命令控制 {args.leader} 主臂 J1 "
            f"移动 {args.delta_deg:.1f}°，从臂由 Runtime 自动跟随"
        )
        _send(robot, args.mode, args.leader, leader_target, args.speed)
        _wait_target(robot, args.leader, leader_target, tolerance)
        print(
            "到达；从臂最大跟踪误差 "
            f"{robot.bimanual_follow_status.max_tracking_error:.4f} rad"
        )

        _send(robot, args.mode, args.leader, SAFE_POSE, args.speed)
        _wait_target(robot, args.leader, SAFE_POSE, tolerance)
        robot.stop_bimanual_follow()
        started = False
        print("已回到起始关系并退出双臂协同")
    finally:
        if robot.connected and started:
            try:
                robot.stop_bimanual_follow()
            except Exception:
                pass
        if robot.connected:
            robot.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("pv", "mit"), default="pv")
    parser.add_argument("--leader", choices=("left", "right"), default="right")
    parser.add_argument("--speed", type=float, default=30.0)
    parser.add_argument("--delta-deg", type=float, default=8.0)
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
