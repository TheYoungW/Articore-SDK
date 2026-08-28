#!/usr/bin/env python3
"""Validate explicit-start Linear and Circular paths on the left Yunyi arm."""
from __future__ import annotations

import json
import math
import time

from arx_d_can import ArxDCanDualArm, MotionState, SafetyState


JOINT_TARGET = [0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0]
SETUP_SPEED_PERCENT = 10.0
CARTESIAN_DURATION_S = 10.0
VERTICAL_DISTANCE_M = 0.070
RIGHT_OFFSET_M = 0.035


def require_healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    if health.state in {
        SafetyState.FAULT,
        SafetyState.SAFE_STOP,
        SafetyState.SAFE_HOLD,
    }:
        raise RuntimeError(
            f"unsafe Runtime state={health.state.name}: "
            f"{health.last_operation_error or health.fault_reason or health.safety_reason}"
        )


def wait_joint_target(
    robot: ArxDCanDualArm,
    target: list[float],
    *,
    timeout_s: float = 20.0,
) -> dict[str, object]:
    started = time.monotonic()
    deadline = started + timeout_s
    while time.monotonic() < deadline:
        require_healthy(robot)
        arm = robot.read_state().left.arm
        maximum_error = max(
            abs(actual - expected)
            for actual, expected in zip(arm.positions, target)
        )
        maximum_speed = max(abs(value) for value in arm.velocities)
        if maximum_error <= 0.005 and maximum_speed <= 0.05:
            return {
                "elapsed_s": time.monotonic() - started,
                "actual": list(arm.positions),
                "maximum_error_rad": maximum_error,
                "maximum_speed_rad_s": maximum_speed,
            }
        time.sleep(0.005)
    raise TimeoutError("left arm did not reach the requested joint target")


def wait_motion(
    robot: ArxDCanDualArm,
    motion_id: int,
    *,
    timeout_s: float = 20.0,
) -> dict[str, object]:
    started = time.monotonic()
    deadline = started + timeout_s
    samples = 0
    last_sequence: int | None = None
    while time.monotonic() < deadline:
        require_healthy(robot)
        state = robot.read_state()
        if state.sequence != last_sequence:
            samples += 1
            last_sequence = state.sequence
        status = robot.get_motion_status(motion_id)
        if status.motion_id != motion_id:
            raise RuntimeError(
                f"motion id changed: expected {motion_id}, got {status.motion_id}"
            )
        if status.state is MotionState.COMPLETED:
            elapsed = time.monotonic() - started
            return {
                "motion_id": motion_id,
                "elapsed_s": elapsed,
                "runtime_duration_s": status.duration_s,
                "samples": samples,
                "sample_hz": samples / max(elapsed, 1e-9),
                "progress": status.progress,
            }
        if status.state in {
            MotionState.CANCELLED,
            MotionState.FAULT,
        }:
            raise RuntimeError(
                f"motion {motion_id} ended as {status.state.value}: {status.error}"
            )
        time.sleep(0.002)
    raise TimeoutError(f"motion {motion_id} did not complete within {timeout_s}s")


def rotation_from_rpy(pose: list[float]) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = pose[3:]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def orientation_error(expected: list[float], actual: list[float]) -> float:
    expected_rotation = rotation_from_rpy(expected)
    actual_rotation = rotation_from_rpy(actual)
    trace = sum(
        expected_rotation[row][column] * actual_rotation[row][column]
        for row in range(3)
        for column in range(3)
    )
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.acos(cosine)


def pose_report(robot: ArxDCanDualArm, expected: list[float]) -> dict[str, object]:
    actual = robot.get_pose("left")
    return {
        "expected": expected,
        "actual": actual,
        "position_error_m": math.dist(actual[:3], expected[:3]),
        "orientation_error_rad": orientation_error(expected, actual),
    }


def main() -> None:
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    enabled = False
    report: dict[str, object] = {
        "joint_target_rad": JOINT_TARGET,
        "vertical_distance_m": VERTICAL_DISTANCE_M,
        "right_offset_m": RIGHT_OFFSET_M,
        "right_direction": "base -Y",
        "duration_s": CARTESIAN_DURATION_S,
    }
    try:
        robot.connect()
        require_healthy(robot)
        initial = robot.read_state()
        initial_left = list(initial.left.arm.positions)
        initial_right = list(initial.right.arm.positions)
        report["initial_left_rad"] = initial_left
        report["initial_right_rad"] = initial_right

        if not robot.enable():
            raise RuntimeError("whole-product enable was not confirmed")
        enabled = True
        robot.set_joint_pv(
            left=JOINT_TARGET,
            right=initial_right,
            velocity=SETUP_SPEED_PERCENT,
        )
        report["joint_setup"] = wait_joint_target(robot, JOINT_TARGET)
        time.sleep(0.25)

        start = robot.get_pose("left")
        end = list(start)
        end[2] += VERTICAL_DISTANCE_M
        via = [
            (start[index] + end[index]) / 2.0
            for index in range(6)
        ]
        via[1] -= RIGHT_OFFSET_M
        report["start_pose"] = start
        report["end_pose"] = end
        report["via_pose"] = via

        motion_id = robot.move_linear_trajectory(
            side="left",
            start_pose=start,
            end_pose=end,
            duration_s=CARTESIAN_DURATION_S,
        )
        report["linear_up"] = wait_motion(robot, motion_id)
        report["linear_up_pose"] = pose_report(robot, end)

        motion_id = robot.move_linear_trajectory(
            side="left",
            start_pose=end,
            end_pose=start,
            duration_s=CARTESIAN_DURATION_S,
        )
        report["linear_return"] = wait_motion(robot, motion_id)
        report["linear_return_pose"] = pose_report(robot, start)

        motion_id = robot.move_circular_trajectory(
            side="left",
            start_pose=start,
            via_pose=via,
            end_pose=end,
            duration_s=CARTESIAN_DURATION_S,
        )
        report["circular_up"] = wait_motion(robot, motion_id)
        report["circular_up_pose"] = pose_report(robot, end)

        motion_id = robot.move_linear_trajectory(
            side="left",
            start_pose=end,
            end_pose=start,
            duration_s=CARTESIAN_DURATION_S,
        )
        report["final_linear_return"] = wait_motion(robot, motion_id)
        report["final_linear_return_pose"] = pose_report(robot, start)
        print(
            json.dumps({"cartesian_results": report}, ensure_ascii=False),
            flush=True,
        )

        robot.set_joint_pv(
            left=initial_left,
            right=initial_right,
            velocity=SETUP_SPEED_PERCENT,
        )
        report["restore"] = wait_joint_target(robot, initial_left)
        report["health"] = robot.get_health().state.name
        print(json.dumps(report, indent=2, ensure_ascii=False))
    finally:
        try:
            robot.cancel_all_motions()
        except Exception:
            pass
        if enabled:
            try:
                robot.disable()
            except Exception:
                pass
        robot.disconnect()


if __name__ == "__main__":
    main()
