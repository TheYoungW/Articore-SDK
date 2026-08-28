#!/usr/bin/env python3
"""Real-hardware check for native TCP FK/IK across set_pose, Linear and Circular."""
from __future__ import annotations

import json
import math
import time

from arx_d_can import ArxDCanDualArm, MotionState, SafetyState


SAFE_Q = [0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0]
CUSTOM_TCP = [-0.004, 0.0, -0.128, 0.0, 0.0, 0.0]


def healthy(robot: ArxDCanDualArm) -> None:
    health = robot.get_health()
    if health.state in {SafetyState.FAULT, SafetyState.SAFE_STOP}:
        raise RuntimeError(
            health.last_operation_error or health.safety_reason or health.fault_reason
        )


def wait_joints(robot: ArxDCanDualArm, left, right, timeout: float = 30.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        healthy(robot)
        state = robot.read_state()
        error = max(
            *(abs(a - b) for a, b in zip(state.left.arm.positions, left)),
            *(abs(a - b) for a, b in zip(state.right.arm.positions, right)),
        )
        speed = max(
            *(abs(value) for value in state.left.arm.velocities),
            *(abs(value) for value in state.right.arm.velocities),
        )
        if error <= 0.01 and speed <= 0.05:
            return
        time.sleep(0.005)
    raise TimeoutError("joint target did not settle")


def wait_pose(robot: ArxDCanDualArm, target, timeout: float = 15.0) -> dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        healthy(robot)
        actual = robot.get_pose("left")
        speed = max(abs(value) for value in robot.read_state().left.arm.velocities)
        position_error = math.dist(actual[:3], target[:3])
        if position_error <= 0.005 and speed <= 0.05:
            return {"actual": actual, "position_error_m": position_error}
        time.sleep(0.005)
    raise TimeoutError("set_pose target did not settle")


def wait_motion(robot: ArxDCanDualArm, motion_id: int, timeout: float = 20.0) -> dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        healthy(robot)
        status = robot.get_motion_status(motion_id)
        if status.state is MotionState.COMPLETED:
            return {"duration_s": status.duration_s, "progress": status.progress}
        if status.state in {MotionState.CANCELLED, MotionState.FAULT}:
            raise RuntimeError(status.error or status.state.value)
        time.sleep(0.002)
    raise TimeoutError(f"motion {motion_id} did not complete")


def main() -> None:
    robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
    enabled = False
    report: dict[str, object] = {}
    try:
        robot.connect()
        initial = robot.read_state()
        initial_left = list(initial.left.arm.positions)
        initial_right = list(initial.right.arm.positions)
        report["default_tcp"] = robot.get_tcp_offset(side="left")
        default_pose = robot.get_pose("left")
        robot.set_tcp_offset(side="left", offset=CUSTOM_TCP)
        report["custom_tcp"] = robot.get_tcp_offset(side="left")
        custom_pose = robot.get_pose("left")
        report["stationary_pose_shift_m"] = math.dist(
            default_pose[:3], custom_pose[:3]
        )

        if not robot.enable():
            raise RuntimeError("enable was not confirmed")
        enabled = True
        robot.set_joint_pv(left=SAFE_Q, right=SAFE_Q, velocity=20.0)
        wait_joints(robot, SAFE_Q, SAFE_Q)

        start = robot.get_pose("left")
        up = list(start)
        up[2] += 0.01
        robot.set_pose(
            left_target_pose=up,
            right_target_pose=robot.get_pose("right"),
            speed_percent=20.0,
        )
        report["ptp"] = wait_pose(robot, up)

        linear_id = robot.move_linear_trajectory(
            side="left", start_pose=up, end_pose=start, duration_s=10.0
        )
        report["linear"] = wait_motion(robot, linear_id)
        report["linear_end"] = wait_pose(robot, start)

        via = list(start)
        end = list(start)
        via[1] += 0.005
        via[2] += 0.005
        end[2] += 0.01
        circular_id = robot.move_circular_trajectory(
            side="left", start_pose=start, via_pose=via, end_pose=end,
            duration_s=10.0,
        )
        report["circular"] = wait_motion(robot, circular_id)
        report["circular_end"] = wait_pose(robot, end)

        return_id = robot.move_linear_trajectory(
            side="left", start_pose=end, end_pose=start, duration_s=10.0
        )
        report["return"] = wait_motion(robot, return_id)
        wait_pose(robot, start)

        robot.set_joint_pv(
            left=initial_left,
            right=initial_right,
            velocity=20.0,
        )
        wait_joints(robot, initial_left, initial_right)
        robot.disable()
        enabled = False
        robot.reset_tcp_offset(side="left")
        report["reset_tcp"] = robot.get_tcp_offset(side="left")
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
        if robot.connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
