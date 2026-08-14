"""Yunyi 双臂 raw MIT 按 Runtime 实际频率发送与反馈频率真机测试。"""
from __future__ import annotations

import math
import os
import statistics
import time

import pytest

from arx_d_can import ArxDCanDualArm
from arx_d_can.driver import damiao_model_limits


EXPECTED_DUAL_CONTROL_HZ = 400.0
TARGET_SPEED = math.radians(30.0)
SETTLE_SECONDS = 1.0


def _soft_clamp(arm, positions) -> tuple[float, ...]:
    margin = arm.config.soft_limit_margin
    output = []
    for value, joint in zip(positions, arm.config.arm_joints):
        model_limit, _, _ = damiao_model_limits(joint.model)
        lower = (
            -model_limit if joint.lower_limit is None else joint.lower_limit
        ) + margin
        upper = (
            model_limit if joint.upper_limit is None else joint.upper_limit
        ) - margin
        output.append(max(lower, min(upper, float(value))))
    return tuple(output)


def _target(joint_count: int) -> tuple[float, ...]:
    values = [0.0] * joint_count
    values[3] = math.radians(90.0)
    return tuple(values)


def _blend(start, target, progress: float) -> tuple[float, ...]:
    u = max(0.0, min(1.0, progress))
    alpha = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    return tuple(
        initial + (final - initial) * alpha
        for initial, final in zip(start, target)
    )


def _feedback_stats(robot: ArxDCanDualArm):
    return {
        **robot.left.robot.get_feedback_stats(
            joint_names=list(robot.left.joint_names)
        ),
        **robot.right.robot.get_feedback_stats(
            joint_names=list(robot.right.joint_names)
        ),
    }


def _submit(robot: ArxDCanDualArm, left, right) -> None:
    # 不提供高级字段：SDK按YAML发送Kp/Kd，dq=0，tau_ff=0。
    robot._submit_joint_positions(left=left, right=right)


def _run_raw_mit_motion(
    robot: ArxDCanDualArm,
    *,
    left_start: tuple[float, ...],
    right_start: tuple[float, ...],
    left_target: tuple[float, ...],
    right_target: tuple[float, ...],
    control_hz: float,
) -> tuple[int, int, float]:
    largest_move = max(
        *(abs(end - start) for start, end in zip(left_start, left_target)),
        *(abs(end - start) for start, end in zip(right_start, right_target)),
    )
    # 五次smoothstep的最大归一化速度为1.875。
    move_seconds = 1.875 * largest_move / TARGET_SPEED
    total_seconds = move_seconds + SETTLE_SECONDS
    period = 1.0 / control_hz
    started = time.perf_counter()
    tick = 0
    skipped = 0
    submitted = 0

    while True:
        scheduled_at = started + tick * period
        remaining = scheduled_at - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        elapsed = time.perf_counter() - started
        if elapsed < move_seconds:
            progress = elapsed / move_seconds
            left = _blend(left_start, left_target, progress)
            right = _blend(right_start, right_target, progress)
        else:
            left = left_target
            right = right_target
        _submit(robot, left, right)
        submitted += 1
        if elapsed >= total_seconds:
            return submitted, skipped, elapsed

        next_tick = tick + 1
        expected_tick = math.floor((time.perf_counter() - started) * control_hz) + 1
        if expected_tick > next_tick:
            skipped += expected_tick - next_tick
            next_tick = expected_tick
        tick = next_tick


def test_dual_raw_mit_uses_runtime_effective_control_rate() -> None:
    if os.environ.get("ARX_D_CAN_RUN_HARDWARE_TEST") != "1":
        pytest.skip("set ARX_D_CAN_RUN_HARDWARE_TEST=1 to move real hardware")

    robot = ArxDCanDualArm(control_mode="mit")
    robot.connect()
    control_hz = robot._effective_control_hz
    print("\n机器人连接成功：即将以 raw MIT 控制双臂")
    try:
        assert control_hz == pytest.approx(EXPECTED_DUAL_CONTROL_HZ)
        robot.enable()
        initial = robot.read_cached_state()
        left_start = _soft_clamp(robot.left, initial.left.arm.positions)
        right_start = _soft_clamp(robot.right, initial.right.arm.positions)
        left_target = _target(len(left_start))
        right_target = _target(len(right_start))

        before = _feedback_stats(robot)
        submitted, skipped, elapsed = _run_raw_mit_motion(
            robot,
            left_start=left_start,
            right_start=right_start,
            left_target=left_target,
            right_target=right_target,
            control_hz=control_hz,
        )
        after = _feedback_stats(robot)
        final = robot.read_cached_state()

        rates = {}
        for name, first in before.items():
            last = after[name]
            assert first.has_feedback and last.has_feedback
            updates = last.update_count - first.update_count
            rates[name] = updates / elapsed

        left_error = [
            math.degrees(actual - expected)
            for actual, expected in zip(final.left.arm.positions, left_target)
        ]
        right_error = [
            math.degrees(actual - expected)
            for actual, expected in zip(final.right.arm.positions, right_target)
        ]
        print(f"raw MIT提交：{submitted}帧，{submitted / elapsed:.2f} Hz")
        print(f"主机调度跳过：{skipped}帧")
        print(
            "自然反馈频率："
            f"min={min(rates.values()):.2f} Hz，"
            f"median={statistics.median(rates.values()):.2f} Hz，"
            f"max={max(rates.values()):.2f} Hz"
        )
        for name in (*robot.left.joint_names, *robot.right.joint_names):
            print(f"  {name}: {rates[name]:.2f} Hz")
        print("左臂最终误差(°)：", [round(value, 3) for value in left_error])
        print("右臂最终误差(°)：", [round(value, 3) for value in right_error])

        assert submitted / elapsed >= control_hz * 0.95
        assert min(rates.values()) >= control_hz * 0.90
        assert max(rates.values()) <= control_hz * 1.10
    finally:
        robot.close()
        print("双臂已失能并断开连接")
