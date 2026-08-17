"""Yunyi 双臂 raw MIT 按 Runtime 实际频率发送与反馈频率真机测试。"""
from __future__ import annotations

import math
import os
import statistics
import time
from collections import deque

import pytest

from arx_d_can import ArxDCanDualArm
from arx_d_can.driver import damiao_model_limits


EXPECTED_DUAL_CONTROL_HZ = 400.0
TARGET_SPEED = math.radians(30.0)
SETTLE_SECONDS = 1.0
CACHED_FEEDBACK_SAMPLE_SECONDS = float(
    os.environ.get("ARX_D_CAN_FEEDBACK_SAMPLE_SECONDS", "30.0")
)
CACHED_FEEDBACK_JUMP_THRESHOLD = math.radians(5.0)
CACHED_FEEDBACK_CROSS_MATCH_TOLERANCE = math.radians(0.05)
CACHED_FEEDBACK_CROSS_HISTORY_SECONDS = 0.05


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


def _feedback_integrity_stats(robot: ArxDCanDualArm):
    return {
        name: motor.get_feedback_integrity_stats()
        for arm in (robot.left, robot.right)
        for name, motor in arm.robot._motor_map.items()
    }


def _submit(robot: ArxDCanDualArm, left, right) -> None:
    # 不提供高级字段：SDK按YAML发送Kp/Kd，dq=0，tau_ff=0。
    robot._submit_joint_positions(left=left, right=right)


def _submit_public_raw_mit(robot: ArxDCanDualArm, left, right) -> None:
    """使用公开 raw MIT 接口保持当前位置，不添加速度或前馈力矩。"""
    robot.submit_raw_mit(
        left_positions=left,
        right_positions=right,
    )


def _cross_channel_value(
    value: float,
    *,
    source_joint,
    destination_joint,
) -> float:
    """把另一通道的逻辑位置还原后，按目标关节 direction 重新解释。"""
    return float(value) * destination_joint.direction / source_joint.direction


def _find_recent_cross_channel_match(
    *,
    value: float,
    joint_index: int,
    destination_arm,
    source_arm,
    source_history,
    now: float,
) -> tuple[float, float, object] | None:
    source_joints = list(source_arm.config.arm_joints)
    if source_arm.config.gripper is not None:
        source_joints.append(source_arm.config.gripper)
    for sampled_at, positions in reversed(source_history):
        age = now - sampled_at
        if age > CACHED_FEEDBACK_CROSS_HISTORY_SECONDS:
            break
        for source_joint, source_value in zip(source_joints, positions):
            candidate = _cross_channel_value(
                source_value,
                source_joint=source_joint,
                destination_joint=destination_arm.config.arm_joints[joint_index],
            )
            if abs(value - candidate) <= CACHED_FEEDBACK_CROSS_MATCH_TOLERANCE:
                return candidate, age, source_joint
    return None


def _cached_feedback_jump(
    previous: float,
    current: float,
    *,
    previous_velocity: float,
    current_velocity: float,
    dt: float,
) -> bool:
    """识别与同帧反馈速度不相容的单次位置跳变。"""
    reported_travel = max(abs(previous_velocity), abs(current_velocity)) * dt
    allowed = max(
        CACHED_FEEDBACK_JUMP_THRESHOLD,
        4.0 * reported_travel + math.radians(0.5),
    )
    return abs(current - previous) > allowed


def _cached_motor_positions(state) -> tuple[float, ...]:
    values = tuple(state.arm.positions)
    if state.gripper is not None:
        values += (state.gripper.motor_position,)
    return values


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


def test_dual_raw_mit_cached_feedback_isolated_by_channel() -> None:
    """持续采样缓存，检查相同 CAN ID 的 CH0/CH1 反馈是否发生串线。"""
    if os.environ.get("ARX_D_CAN_RUN_HARDWARE_TEST") != "1":
        pytest.skip("set ARX_D_CAN_RUN_HARDWARE_TEST=1 to use real hardware")
    if CACHED_FEEDBACK_SAMPLE_SECONDS <= 0.0:
        raise ValueError("ARX_D_CAN_FEEDBACK_SAMPLE_SECONDS must be positive")

    robot = ArxDCanDualArm(control_mode="mit")
    robot.connect()
    control_hz = robot._effective_control_hz
    period = 1.0 / control_hz
    print(
        "\n机器人连接成功：保持当前位置并持续采样 read_cached_state() "
        f"{CACHED_FEEDBACK_SAMPLE_SECONDS:.1f} 秒"
    )
    try:
        robot.enable()
        initial = robot.read_cached_state()
        left_target = _soft_clamp(robot.left, initial.left.arm.positions)
        right_target = _soft_clamp(robot.right, initial.right.arm.positions)
        integrity_before = _feedback_integrity_stats(robot)

        history_capacity = max(
            4,
            math.ceil(control_hz * CACHED_FEEDBACK_CROSS_HISTORY_SECONDS) + 2,
        )
        left_history = deque(maxlen=history_capacity)
        right_history = deque(maxlen=history_capacity)
        previous = None
        anomalies: list[str] = []
        active_anomalies: set[tuple[str, int]] = set()
        samples = 0
        skipped = 0
        started = time.perf_counter()
        tick = 0

        while True:
            scheduled_at = started + tick * period
            remaining = scheduled_at - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)

            _submit_public_raw_mit(robot, left_target, right_target)
            sampled_at = time.perf_counter()
            state = robot.read_cached_state()
            samples += 1
            left_positions = tuple(state.left.arm.positions)
            right_positions = tuple(state.right.arm.positions)
            left_history.append((sampled_at, _cached_motor_positions(state.left)))
            right_history.append((sampled_at, _cached_motor_positions(state.right)))

            if previous is not None:
                previous_at, previous_state = previous
                dt = max(sampled_at - previous_at, 1e-9)
                sides = (
                    (
                        "CH0/left",
                        robot.left,
                        robot.right,
                        left_positions,
                        previous_state.left.arm.positions,
                        state.left.arm.velocities,
                        previous_state.left.arm.velocities,
                        right_history,
                    ),
                    (
                        "CH1/right",
                        robot.right,
                        robot.left,
                        right_positions,
                        previous_state.right.arm.positions,
                        state.right.arm.velocities,
                        previous_state.right.arm.velocities,
                        left_history,
                    ),
                )
                for (
                    side_name,
                    destination_arm,
                    source_arm,
                    positions,
                    previous_positions,
                    velocities,
                    previous_velocities,
                    source_history,
                ) in sides:
                    for index, (old, new, old_velocity, new_velocity) in enumerate(
                        zip(
                            previous_positions,
                            positions,
                            previous_velocities,
                            velocities,
                        )
                    ):
                        if not _cached_feedback_jump(
                            old,
                            new,
                            previous_velocity=old_velocity,
                            current_velocity=new_velocity,
                            dt=dt,
                        ):
                            continue
                        match = _find_recent_cross_channel_match(
                            value=new,
                            joint_index=index,
                            destination_arm=destination_arm,
                            source_arm=source_arm,
                            source_history=source_history,
                            now=sampled_at,
                        )
                        anomaly_key = (side_name, index)
                        if match is None and anomaly_key in active_anomalies:
                            # 串入值后的下一次跳变通常只是缓存恢复，不重复计数。
                            active_anomalies.remove(anomaly_key)
                            continue
                        match_text = "no opposite-channel match"
                        if match is not None:
                            active_anomalies.add(anomaly_key)
                            candidate, age, source_joint = match
                            match_text = (
                                f"matches opposite {source_joint.name} "
                                f"(CAN ID 0x{source_joint.motor_id:02X}, "
                                f"feedback 0x{source_joint.feedback_id:02X}) value "
                                f"{math.degrees(candidate):+.4f} deg "
                                f"from {age * 1000.0:.2f} ms ago"
                            )
                        destination_joint = destination_arm.config.arm_joints[index]
                        raw_state = destination_arm.robot._motor_map[
                            destination_joint.name
                        ].get_state()
                        raw_identity = "raw cache unavailable"
                        if raw_state is not None:
                            raw_identity = (
                                f"raw can_id=0x{raw_state.can_id:02X}, "
                                f"arbitration_id=0x{raw_state.arbitration_id:X}"
                            )
                        anomalies.append(
                            f"sample={samples} {side_name} "
                            f"joint{index + 1}: {math.degrees(old):+.4f} -> "
                            f"{math.degrees(new):+.4f} deg, dt={dt * 1000.0:.3f} ms, "
                            f"feedback_velocity={math.degrees(new_velocity):+.3f} deg/s; "
                            f"{match_text}; {raw_identity}"
                        )
            previous = (sampled_at, state)

            elapsed = sampled_at - started
            if elapsed >= CACHED_FEEDBACK_SAMPLE_SECONDS:
                break
            next_tick = tick + 1
            expected_tick = math.floor(elapsed * control_hz) + 1
            if expected_tick > next_tick:
                skipped += expected_tick - next_tick
                next_tick = expected_tick
            tick = next_tick

        elapsed = time.perf_counter() - started
        integrity_after = _feedback_integrity_stats(robot)
        rejected = {}
        for name, first in integrity_before.items():
            last = integrity_after[name]
            delta = last.rejected_frame_count - first.rejected_frame_count
            assert delta >= 0
            if delta:
                rejected[name] = (
                    delta,
                    last.short_frame_count - first.short_frame_count,
                    last.identity_mismatch_count - first.identity_mismatch_count,
                    last.implausible_position_jump_count
                    - first.implausible_position_jump_count,
                )
        print(
            f"缓存采样：{samples} 帧，{samples / elapsed:.2f} Hz；"
            f"主机调度跳过：{skipped} 帧；缓存异常：{len(anomalies)}；"
            f"底层拒绝帧：{sum(value[0] for value in rejected.values())}"
        )
        for name, (total, short, identity, jump) in rejected.items():
            print(
                f"  {name}: rejected={total}, short={short}, "
                f"identity={identity}, position_jump={jump}"
            )
        for anomaly in anomalies[:20]:
            print("  ", anomaly)
        assert not anomalies, "cached feedback discontinuities:\n" + "\n".join(
            anomalies[:20]
        )
    finally:
        robot.close()
        print("双臂已失能并断开连接")
