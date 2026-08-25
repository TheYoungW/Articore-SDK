#!/usr/bin/env python3
"""Real Yunyi tool0 topology and small PV Cartesian motion validation."""
from __future__ import annotations

import argparse
import json
import math
import time

from arx_d_can import ArxDCanDualArm, CartesianMotionState, SafetyState


TOOL_OFFSET = (-0.004, 0.0, -0.178)


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, ...], ...]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def expected_tool_pose(flange: list[float]) -> list[float]:
    rotation = rotation_from_rpy(*flange[3:])
    translation = [
        sum(rotation[row][column] * TOOL_OFFSET[column] for column in range(3))
        for row in range(3)
    ]
    return [
        *(flange[index] + translation[index] for index in range(3)),
        *flange[3:],
    ]


def require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    if health.state in {
        SafetyState.FAULT,
        SafetyState.SAFE_STOP,
        SafetyState.SAFE_HOLD,
    }:
        raise RuntimeError(
            f"unsafe Runtime state={health.state.name}: "
            f"{health.fault_reason or health.safety_reason}"
        )


def read_topology(with_grippers: bool) -> dict[str, object]:
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=with_grippers)
    try:
        robot.connect()
        require_healthy(robot)
        state = robot.read_state()
        return {
            "with_grippers": with_grippers,
            "state": robot.get_health().state.name,
            "left_q": list(state.left.arm.positions),
            "right_q": list(state.right.arm.positions),
            "left_pose": robot.get_pose("left"),
            "right_pose": robot.get_pose("right"),
        }
    finally:
        robot.disconnect()


def wait_motion(
    robot: ArxDCanDualArm,
    motion_id: int,
    timeout_s: float,
    *,
    allow_fault: bool = False,
) -> dict[str, object]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        require_healthy(robot)
        status = robot.cartesian_motion_status
        if status.motion_id != motion_id:
            raise RuntimeError(
                f"motion id changed: expected {motion_id}, got {status.motion_id}"
            )
        if status.state is CartesianMotionState.COMPLETED:
            return {
                "motion_id": motion_id,
                "state": status.state.value,
                "duration_s": status.duration_s,
                "elapsed_s": status.elapsed_s,
                "progress": status.progress,
            }
        if status.state in {
            CartesianMotionState.CANCELLED,
            CartesianMotionState.FAULT,
        }:
            if allow_fault:
                return {
                    "motion_id": motion_id,
                    "state": status.state.value,
                    "duration_s": status.duration_s,
                    "elapsed_s": status.elapsed_s,
                    "progress": status.progress,
                    "error": status.error,
                }
            raise RuntimeError(
                f"motion {motion_id} ended as {status.state.value}: {status.error}"
            )
        time.sleep(0.01)
    raise TimeoutError(f"motion {motion_id} did not complete within {timeout_s}s")


def settle_pose(robot: ArxDCanDualArm, side: str, seconds: float = 2.0) -> list[float]:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        require_healthy(robot)
        time.sleep(0.02)
    return robot.get_pose(side)


def wait_ptp_pose(
    robot: ArxDCanDualArm,
    side: str,
    target: list[float],
    timeout_s: float,
) -> dict[str, object]:
    """PTP has no motion status; verify pose and velocity feedback directly."""
    deadline = time.monotonic() + timeout_s
    stable_samples = 0
    while time.monotonic() < deadline:
        require_healthy(robot)
        pose = robot.get_pose(side)
        arm = getattr(robot.read_state(), side).arm
        position_error = math.dist(pose[:3], target[:3])
        orientation_error = math.dist(pose[3:], target[3:])
        maximum_velocity = max(abs(value) for value in arm.velocities)
        if (
            position_error <= 0.006
            and orientation_error <= 0.035
            and maximum_velocity <= 0.05
        ):
            stable_samples += 1
            if stable_samples >= 25:
                return {
                    "state": "feedback_settled",
                    "position_error_m": position_error,
                    "orientation_error_rad": orientation_error,
                    "maximum_velocity_rad_s": maximum_velocity,
                }
        else:
            stable_samples = 0
        time.sleep(0.002)
    raise TimeoutError(f"PTP did not settle within {timeout_s}s")


def collect_hold(
    robot: ArxDCanDualArm,
    motion_id: int | None,
    *,
    seconds: float = 5.0,
) -> dict[str, object]:
    minimum = [math.inf] * 7
    maximum = [-math.inf] * 7
    velocity_peak = [0.0] * 7
    samples = 0
    last_sequence = None
    status_counts: dict[str, int] = {}
    started = time.monotonic()
    deadline = started + seconds
    while time.monotonic() < deadline:
        require_healthy(robot)
        if motion_id is not None:
            status = robot.cartesian_motion_status
            if status.motion_id != motion_id:
                raise RuntimeError(
                    f"hold motion id changed: expected {motion_id}, "
                    f"got {status.motion_id}"
                )
            status_counts[status.state.value] = (
                status_counts.get(status.state.value, 0) + 1
            )
        state = robot.read_state()
        if state.sequence != last_sequence:
            last_sequence = state.sequence
            samples += 1
            for index, (position, velocity) in enumerate(
                zip(state.left.arm.positions, state.left.arm.velocities)
            ):
                minimum[index] = min(minimum[index], position)
                maximum[index] = max(maximum[index], position)
                velocity_peak[index] = max(
                    velocity_peak[index], abs(velocity)
                )
        time.sleep(0.001)
    return {
        "seconds": time.monotonic() - started,
        "samples": samples,
        "sample_hz": samples / max(time.monotonic() - started, 1e-9),
        "position_peak_to_peak_rad": [
            high - low for low, high in zip(minimum, maximum)
        ],
        "velocity_absolute_peak_rad_s": velocity_peak,
        "status_counts": status_counts,
        "final_status": (
            "not_available_for_ptp"
            if motion_id is None
            else robot.cartesian_motion_status.state.value
        ),
        "final_status_error": (
            None
            if motion_id is None
            else robot.cartesian_motion_status.error
        ),
    }


def bring_left_joint4_inside_limit(robot: ArxDCanDualArm) -> dict[str, object] | None:
    state = robot.read_state()
    actual = state.left.arm.positions[3]
    if actual >= -0.1744:
        return None
    left = list(state.left.arm.positions)
    right = list(state.right.arm.positions)
    left[3] = -0.15
    robot.set_max_speed(5.0)
    robot.set_joint_pv(left=left, right=right)
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        require_healthy(robot)
        current = robot.read_state().left.arm
        if abs(current.positions[3] - left[3]) <= 0.005 and abs(current.velocities[3]) <= 0.05:
            return {
                "initial_rad": actual,
                "target_rad": left[3],
                "reached_rad": current.positions[3],
            }
        time.sleep(0.01)
    raise TimeoutError("left joint4 did not return inside its product limit")


def restore_left_joint_pose(target: list[float]) -> dict[str, object]:
    if len(target) != 7 or any(not math.isfinite(value) for value in target):
        raise ValueError("--restore-left requires seven finite radians")
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    enabled = False
    try:
        robot.connect()
        require_healthy(robot)
        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")
        enabled = True
        current = robot.read_state()
        robot.set_max_speed(5.0)
        robot.set_joint_pv(
            left=target,
            right=current.right.arm.positions,
        )
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            require_healthy(robot)
            arm = robot.read_state().left.arm
            error = max(
                abs(actual - expected)
                for actual, expected in zip(arm.positions, target)
            )
            speed = max(abs(value) for value in arm.velocities)
            if error <= 0.005 and speed <= 0.05:
                return {
                    "target": target,
                    "actual": list(arm.positions),
                    "maximum_error_rad": error,
                    "maximum_velocity_rad_s": speed,
                }
            time.sleep(0.005)
        raise TimeoutError("left arm did not restore the recorded joint pose")
    finally:
        if enabled:
            try:
                robot.disable()
            except Exception:
                pass
        robot.disconnect()


def move_and_return(
    side: str,
    distance_m: float,
    *,
    speed_percent: float = 10.0,
    hold_seconds: float = 5.0,
) -> dict[str, object]:
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    enabled = False
    try:
        robot.connect()
        require_healthy(robot)
        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")
        enabled = True
        limit_recovery = bring_left_joint4_inside_limit(robot)
        start = robot.get_pose(side)
        target = list(start)
        target[2] += distance_m
        robot.move_pose(
            side=side, target_pose=target, speed_percent=speed_percent
        )
        outbound = wait_ptp_pose(robot, side, target, 15.0)
        outbound_hold = collect_hold(robot, None, seconds=hold_seconds)
        reached = robot.get_pose(side)
        return_id = robot.move_linear(
            side=side, start_pose=reached, end_pose=start,
            speed_percent=speed_percent,
        )
        returned = wait_motion(robot, return_id, 15.0, allow_fault=True)
        return_hold = collect_hold(
            robot, return_id, seconds=hold_seconds
        )
        final = robot.get_pose(side)
        require_healthy(robot)
        return {
            "start": start,
            "target": target,
            "reached": reached,
            "target_position_error_m": math.dist(reached[:3], target[:3]),
            "final": final,
            "return_position_error_m": math.dist(final[:3], start[:3]),
            "outbound": outbound,
            "outbound_hold": outbound_hold,
            "returned": returned,
            "return_hold": return_hold,
            "health": robot.get_health().state.name,
            "limit_recovery": limit_recovery,
        }
    finally:
        try:
            robot.cancel_cartesian_motion()
        except Exception:
            pass
        if enabled:
            try:
                robot.disable()
            except Exception:
                pass
        robot.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--move-mm", type=float, default=0.0)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--speed-percent", type=float, default=10.0)
    parser.add_argument("--hold-seconds", type=float, default=5.0)
    parser.add_argument(
        "--restore-left",
        type=lambda text: [float(value) for value in text.split(",")],
    )
    args = parser.parse_args()
    report: dict[str, object] = {}
    if args.restore_left is not None:
        report["restore"] = restore_left_joint_pose(args.restore_left)
    flange = read_topology(False)
    tool = read_topology(True)
    report["without_grippers"] = flange
    report["with_grippers"] = tool
    for side in ("left", "right"):
        q_delta = max(
            abs(left - right)
            for left, right in zip(flange[f"{side}_q"], tool[f"{side}_q"])
        )
        expected = expected_tool_pose(flange[f"{side}_pose"])
        measured = tool[f"{side}_pose"]
        position_error = math.dist(expected[:3], measured[:3])
        orientation_error = max(
            abs(expected[index] - measured[index]) for index in range(3, 6)
        )
        if q_delta > 0.01 or position_error > 0.003 or orientation_error > 0.01:
            raise RuntimeError(
                f"{side} tool0 mismatch: q_delta={q_delta}, "
                f"position_error={position_error}, orientation_error={orientation_error}"
            )
        report[f"{side}_topology_check"] = {
            "maximum_joint_delta_rad": q_delta,
            "expected_tool_pose": expected,
            "measured_tool_pose": measured,
            "position_error_m": position_error,
            "orientation_error_rad": orientation_error,
        }
    if args.move_mm:
        report["motion"] = move_and_return(
            args.side,
            args.move_mm / 1000.0,
            speed_percent=args.speed_percent,
            hold_seconds=args.hold_seconds,
        )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
