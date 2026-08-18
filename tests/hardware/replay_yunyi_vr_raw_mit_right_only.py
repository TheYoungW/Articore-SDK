"""Single-right-arm A/B replay for the Yunyi VR feedback-loss diagnosis."""
from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass, replace
import json
from pathlib import Path
import time
from typing import Any

from arx_d_can import ArxDCanArm

from replay_yunyi_vr_raw_mit import (
    CanMemoryTrace,
    DEFAULT_TRACE,
    _load_frames,
)


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "name") and hasattr(value, "value"):
        return value.name
    return value


def _wait_at_start(robot: ArxDCanArm, target, velocity: float, timeout_s: float) -> None:
    robot.set_joint_mit(target, velocity=velocity)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = robot.read_cached_state()
        if max(abs(a - b) for a, b in zip(state.arm.positions, target)) < 0.05:
            return
        health = robot.safety_health
        if health.fault_reason:
            raise RuntimeError(f"fault while moving to replay start: {health.fault_reason}")
        time.sleep(0.1)
    raise RuntimeError("right arm did not reach the replay start pose")


def _submit_raw(robot: ArxDCanArm, positions) -> None:
    # Match the public dual-arm API's resultant-torque limiter. The single-arm
    # SDK does not yet expose a public raw MIT method.
    state = robot.read_cached_state()
    velocity, kp, kd, torque = robot._limit_raw_mit_resultant_torque(
        positions=positions,
        velocities=None,
        kp=None,
        kd=None,
        feedforward_torques=None,
        current_positions=state.arm.positions,
        current_velocities=state.arm.velocities,
    )
    robot._submit_joint_positions(
        positions,
        velocities=velocity,
        torques=torque,
        mit_kp=kp,
        mit_kd=kd,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay only the right arm of a recorded Yunyi VR command stream."
    )
    parser.add_argument("--trace", type=Path, default=DEFAULT_TRACE)
    parser.add_argument("--control-hz", type=int, choices=(400, 500), default=500)
    parser.add_argument("--start-velocity", type=float, default=0.15)
    parser.add_argument("--start-timeout-s", type=float, default=25.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/dev/shm/yunyi_vr_right_only.jsonl"),
    )
    args = parser.parse_args()

    targets, frames = _load_frames(args.trace, args.control_hz)
    capture = CanMemoryTrace(("can1",), 5.0)
    robot = ArxDCanArm(model="yunyi_v1_0_right", control_mode="mit")
    robot.config = replace(robot.config, control_hz=float(args.control_hz))
    result: dict[str, Any] = {
        "right_only": True,
        "source_targets": len(targets),
        "generated_frames": len(frames),
        "requested_control_hz": args.control_hz,
    }
    failure: BaseException | None = None
    submitted = 0
    raw_started: float | None = None
    capture.start()
    try:
        robot.connect()
        robot.enable()
        _wait_at_start(
            robot,
            targets[0][1],
            args.start_velocity,
            args.start_timeout_s,
        )
        raw_started = time.perf_counter()
        period = 1.0 / args.control_hz
        for index, frame in enumerate(frames):
            remaining = raw_started + index * period - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            _submit_raw(robot, frame.right)
            submitted += 1
        elapsed = time.perf_counter() - raw_started
        result.update(
            submitted_frames=submitted,
            elapsed_s=elapsed,
            submit_hz=submitted / elapsed,
            runtime_health=_jsonable(robot.safety_health),
        )
    except BaseException as exc:
        failure = exc
        elapsed = 0.0 if raw_started is None else time.perf_counter() - raw_started
        result.update(
            submitted_frames=submitted,
            elapsed_s=elapsed,
            submit_hz=0.0 if elapsed <= 0 else submitted / elapsed,
            failure=f"{type(exc).__name__}: {exc}",
        )
        try:
            result["runtime_health"] = _jsonable(robot.safety_health)
        except Exception as diagnostic_exc:
            result["diagnostic_error"] = f"{type(diagnostic_exc).__name__}: {diagnostic_exc}"
    finally:
        try:
            if robot.connected:
                robot.disable()
                result["disable_report"] = _jsonable(robot.last_disable_report)
        except Exception as exc:
            result["disable_error"] = f"{type(exc).__name__}: {exc}"
        try:
            robot.close()
        except Exception as exc:
            result["close_error"] = f"{type(exc).__name__}: {exc}"
        capture.stop()
        result["can_summary"] = capture.summary()
        capture.dump(args.output, result)

    health = result.get("runtime_health", {})
    print(
        json.dumps(
            {
                "submitted_frames": result.get("submitted_frames"),
                "elapsed_s": result.get("elapsed_s"),
                "submit_hz": result.get("submit_hz"),
                "failure": result.get("failure"),
                "runtime_state": health.get("state"),
                "fault_reason": health.get("fault_reason"),
                "disable_report": result.get("disable_report"),
                "disable_error": result.get("disable_error"),
                "close_error": result.get("close_error"),
                "can_summary": result["can_summary"],
                "diagnostic": str(args.output),
            },
            indent=2,
        )
    )
    if failure is not None:
        raise failure
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
