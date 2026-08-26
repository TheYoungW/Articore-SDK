#!/usr/bin/env python3
"""Real-hardware check of the public SDK's native 14-joint trajectory path."""
from __future__ import annotations

import argparse
import json
import math
import time

from arx_d_can import ArxDCanDualArm, TrajectoryState
from arx_d_can.service_tools.dual_trajectory_recording import (
    DualArmTrajectorySample,
    replay,
)


MIT_KP = (190.0, 190.0, 70.0, 125.0, 10.0, 22.0, 28.0)
MIT_KD = (4.55, 4.5, 2.0, 2.9, 0.7, 0.89, 0.84)


def _toward_zero(value: float, distance: float, zero_direction: float) -> float:
    if abs(value) < 0.05:
        return value + zero_direction * distance
    return value - math.copysign(min(abs(value), distance), value)


def _wait(robot: ArxDCanDualArm, timeout: float) -> tuple[object, list[float]]:
    deadline = time.monotonic() + timeout
    sample_times: list[float] = []
    while time.monotonic() < deadline:
        sample_times.append(time.monotonic())
        robot.read_state()
        status = robot.trajectory_status
        if status.state in {
            TrajectoryState.COMPLETED,
            TrajectoryState.CANCELLED,
            TrajectoryState.FAULT,
        }:
            return status, sample_times
        time.sleep(0.002)
    raise TimeoutError("trajectory did not reach a terminal state")


def _start(
    robot: ArxDCanDualArm,
    mode: str,
    start_left: tuple[float, ...],
    start_right: tuple[float, ...],
    end_left: tuple[float, ...],
    end_right: tuple[float, ...],
    duration: float,
) -> int:
    arguments = dict(
        timestamps=[0.0, duration],
        left_positions=[start_left, end_left],
        right_positions=[start_right, end_right],
    )
    if mode == "pv":
        arguments["pv_velocity_limits"] = 2.5
    else:
        arguments.update(
            kp=MIT_KP,
            kd=MIT_KD,
            feedforward_torque=(0.0,) * 7,
        )
    return robot.start_trajectory(**arguments)


def run(mode: str, cancel: bool, replay_service: bool) -> dict[str, object]:
    robot = ArxDCanDualArm(control_mode=mode, with_grippers=True)
    result: dict[str, object] = {
        "mode": mode,
        "cancel_requested": cancel,
        "submission": "recording.replay" if replay_service else "start_trajectory",
    }
    robot.connect()
    try:
        before = robot.read_state()
        start_left = tuple(before.left.arm.positions)
        start_right = tuple(before.right.arm.positions)
        target_right = list(start_right)
        target_right[0] = _toward_zero(target_right[0], 0.15, 1.0)
        target_right[1] = _toward_zero(target_right[1], 0.12, -1.0)
        target_right_tuple = tuple(target_right)

        robot.enable()
        if replay_service:
            status = replay(
                robot,
                timestamps=[0.0, 2.0],
                samples=[
                    DualArmTrajectorySample(start_left, start_right, None, None),
                    DualArmTrajectorySample(
                        start_left, target_right_tuple, None, None
                    ),
                ],
            )
            trajectory_id = status.trajectory_id
        else:
            trajectory_id = _start(
                robot,
                mode,
                start_left,
                start_right,
                start_left,
                target_right_tuple,
                4.0 if cancel else 2.0,
            )
        if cancel:
            time.sleep(0.5)
            robot.cancel_trajectory()
        status, sample_times = (
            (status, []) if replay_service else _wait(robot, 10.0)
        )
        result.update(
            trajectory_id=trajectory_id,
            terminal_state=status.state.value,
            progress=status.progress,
            status_error=status.error,
            measured_read_hz=(
                (len(sample_times) - 1) / (sample_times[-1] - sample_times[0])
                if len(sample_times) > 1
                else 0.0
            ),
        )
        if cancel:
            if status.state is not TrajectoryState.CANCELLED:
                raise RuntimeError(f"expected CANCELLED, got {status.state.value}")
        elif status.state is not TrajectoryState.COMPLETED:
            raise RuntimeError(status.error or f"trajectory ended as {status.state.value}")

        current = robot.read_state()
        return_id = _start(
            robot,
            mode,
            tuple(current.left.arm.positions),
            tuple(current.right.arm.positions),
            start_left,
            start_right,
            2.0,
        )
        return_status, _ = _wait(robot, 10.0)
        if return_status.state is not TrajectoryState.COMPLETED:
            raise RuntimeError(
                return_status.error or f"return ended as {return_status.state.value}"
            )
        result["return_trajectory_id"] = return_id
        result["health_before_disconnect"] = robot.get_health().state.name
    finally:
        robot.disconnect()

    verifier = ArxDCanDualArm(control_mode=mode, with_grippers=True)
    verifier.connect()
    try:
        final_state = verifier.read_state()
        enabled = final_state.left.arm.enabled + final_state.right.arm.enabled
        health = verifier.get_health()
        result.update(
            all_arms_disabled=all(value is False for value in enabled),
            final_health=health.state.name,
            disable_confirmed=health.disable_confirmed,
            fault_reason=health.fault_reason,
        )
    finally:
        verifier.disconnect()
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("pv", "mit"), required=True)
    parser.add_argument("--cancel", action="store_true")
    parser.add_argument("--replay-service", action="store_true")
    args = parser.parse_args()
    if args.cancel and args.replay_service:
        parser.error("--cancel and --replay-service cannot be combined")
    print(
        json.dumps(
            run(args.mode, args.cancel, args.replay_service),
            indent=2,
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
