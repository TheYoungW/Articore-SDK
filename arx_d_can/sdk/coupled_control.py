"""耦合关节坐标变换与 MIT 内环支持。"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import replace
from typing import Sequence

import numpy as np

from ..errors import CommunicationError, StaleFeedbackError
from .state import (
    CoupledControlStats,
    CoupledTorqueSaturation,
    CoupledTorqueTelemetry,
    MitCommand,
)


_LOG = logging.getLogger(__name__)


class _CoupledControlMixin:
    def _clamp_transformed_virtual_positions(
        self,
        positions: Sequence[float],
    ) -> np.ndarray:
        result = np.asarray(positions, dtype=np.float64).reshape(-1).copy()
        if self._joint_transform is None:
            return result
        for index in self._joint_transform.transformed_indices:
            joint = self.config.arm_joints[index]
            if joint.lower_limit is not None:
                result[index] = max(result[index], joint.lower_limit)
            if joint.upper_limit is not None:
                result[index] = min(result[index], joint.upper_limit)
        return result

    def _transform_command_vectors(
        self,
        positions: Sequence[float],
        *,
        velocities: Sequence[float] | None = None,
        torques: Sequence[float] | None = None,
        velocity_limits: Sequence[float] | None = None,
    ) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        virtual_positions = self._clamp_transformed_virtual_positions(positions)
        if self._joint_transform is None:
            return (
                virtual_positions,
                None if velocities is None else np.asarray(velocities, dtype=np.float64),
                None if torques is None else np.asarray(torques, dtype=np.float64),
                None
                if velocity_limits is None
                else np.asarray(velocity_limits, dtype=np.float64),
            )
        motor_positions = self._joint_transform.virtual_positions_to_motor(
            virtual_positions
        )
        motor_velocities = (
            None
            if velocities is None
            else self._joint_transform.virtual_velocities_to_motor(
                virtual_positions,
                velocities,
            )
        )
        motor_torques = (
            None
            if torques is None
            else self._joint_transform.virtual_torques_to_motor(
                virtual_positions,
                torques,
            )
        )
        motor_velocity_limits = (
            None
            if velocity_limits is None
            else self._joint_transform.virtual_velocity_limits_to_motor(
                virtual_positions,
                velocity_limits,
            )
        )
        return (
            motor_positions,
            motor_velocities,
            motor_torques,
            motor_velocity_limits,
        )

    def _transform_feedback_vectors(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        torques: Sequence[float],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        motor_positions = np.asarray(positions, dtype=np.float64)
        motor_velocities = np.asarray(velocities, dtype=np.float64)
        motor_torques = np.asarray(torques, dtype=np.float64)
        if self._joint_transform is None:
            return motor_positions, motor_velocities, motor_torques
        return (
            self._joint_transform.motor_positions_to_virtual(motor_positions),
            self._joint_transform.motor_velocities_to_virtual(
                motor_positions,
                motor_velocities,
            ),
            self._joint_transform.motor_torques_to_virtual(
                motor_positions,
                motor_torques,
            ),
        )

    def _resolved_mit_gains(
        self,
        values: np.ndarray | None,
        *,
        gain: str,
    ) -> np.ndarray:
        if values is not None:
            return np.asarray(values, dtype=np.float64).reshape(-1).copy()
        attribute = "mit_kp" if gain == "kp" else "mit_kd"
        return np.asarray(
            [getattr(joint, attribute) for joint in self.config.arm_joints],
            dtype=np.float64,
        )

    def _make_mit_command(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        kp: Sequence[float],
        kd: Sequence[float],
        feedforward_torques: Sequence[float],
    ) -> MitCommand:
        logical_positions = self._clamp_transformed_virtual_positions(positions)
        return MitCommand(
            positions=tuple(float(value) for value in logical_positions),
            velocities=tuple(float(value) for value in velocities),
            kp=tuple(float(value) for value in kp),
            kd=tuple(float(value) for value in kd),
            feedforward_torques=tuple(
                float(value) for value in feedforward_torques
            ),
            timestamp=time.monotonic(),
        )

    def _compose_mit_motor_command(
        self,
        command: MitCommand,
        *,
        motor_positions: Sequence[float] | None = None,
        motor_velocities: Sequence[float] | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """将一条逻辑 MIT 命令解析到物理电机坐标。"""
        logical_position = np.asarray(command.positions, dtype=np.float64)
        logical_velocity = np.asarray(command.velocities, dtype=np.float64)
        logical_kp = np.asarray(command.kp, dtype=np.float64)
        logical_kd = np.asarray(command.kd, dtype=np.float64)
        logical_tau = np.asarray(command.feedforward_torques, dtype=np.float64)
        if self._joint_transform is None:
            return (
                logical_position.copy(),
                logical_velocity.copy(),
                logical_kp.copy(),
                logical_kd.copy(),
                logical_tau.copy(),
            )
        if motor_positions is None or motor_velocities is None:
            raise RuntimeError("coupled MIT control requires cached motor feedback")

        physical_position = np.asarray(
            motor_positions, dtype=np.float64
        ).reshape(-1)
        physical_velocity = np.asarray(
            motor_velocities, dtype=np.float64
        ).reshape(-1)
        joint_count = len(self.config.arm_joints)
        if (
            physical_position.size != joint_count
            or physical_velocity.size != joint_count
        ):
            raise RuntimeError("coupled MIT feedback does not cover every arm motor")

        actual_position = self._joint_transform.motor_positions_to_virtual(
            physical_position
        )
        actual_velocity = self._joint_transform.motor_velocities_to_virtual(
            physical_position,
            physical_velocity,
        )
        filtered_actual_velocity = self._filter_coupled_virtual_velocities(
            actual_velocity
        )
        transformed_indices = sorted(self._joint_transform.transformed_indices)
        virtual_tau = logical_tau.copy()
        virtual_tau[transformed_indices] += (
            logical_kp[transformed_indices]
            * (
                logical_position[transformed_indices]
                - actual_position[transformed_indices]
            )
            + logical_kd[transformed_indices]
            * (
                logical_velocity[transformed_indices]
                - filtered_actual_velocity[transformed_indices]
            )
        )

        motor_position_target = self._joint_transform.virtual_positions_to_motor(
            logical_position
        )
        motor_velocity_target = self._joint_transform.virtual_velocities_to_motor(
            logical_position,
            logical_velocity,
        )
        transformed_motor_tau = self._joint_transform.virtual_torques_to_motor(
            actual_position,
            virtual_tau,
        )
        motor_kp = logical_kp.copy()
        motor_kd = logical_kd.copy()
        motor_kp[transformed_indices] = 0.0
        motor_velocity_command = motor_velocity_target.copy()
        motor_velocity_command[transformed_indices] = 0.0

        requested_motor_tau = transformed_motor_tau
        limited_motor_tau = self._limit_coupled_motor_torques(
            requested_motor_tau
        )
        now = time.monotonic()
        hold_pairs_set: set[tuple[int, int]] = set()
        for pair in self._joint_transform.transformed_pairs:
            hold_candidate = (
                all(
                    self.config.arm_joints[index].coupled_hold_torque_rise_rate
                    is not None
                    for index in pair
                )
                and max(abs(float(logical_velocity[index])) for index in pair)
                <= math.radians(1.0)
                and max(
                    abs(float(logical_position[index] - actual_position[index]))
                    for index in pair
                )
                <= math.radians(1.5)
            )
            if not hold_candidate:
                self._coupled_hold_candidate_since.pop(pair, None)
                continue
            started = self._coupled_hold_candidate_since.setdefault(pair, now)
            if now - started >= 0.15:
                hold_pairs_set.add(pair)
        hold_pairs = frozenset(hold_pairs_set)
        motor_tau = self._slew_coupled_motor_torques(
            limited_motor_tau,
            hold_pairs=hold_pairs,
        )

        # 物理电机侧增益刻意采用被动形式：目标速度为零，因此只会阻碍实测运动。
        # 必要时自适应调整增益，确保前馈力矩与估算阻尼之和不超过电机硬力矩限制。
        damping_tau = np.zeros(joint_count, dtype=np.float64)
        for index in transformed_indices:
            joint = self.config.arm_joints[index]
            gain = joint.coupled_motor_kd
            speed = abs(float(physical_velocity[index]))
            if joint.effort_limit is not None and speed > 1e-12:
                remaining = max(
                    0.0,
                    joint.effort_limit - abs(float(motor_tau[index])),
                )
                gain = min(gain, remaining / speed)
            motor_kd[index] = gain
            damping_tau[index] = -gain * physical_velocity[index]
        estimated_total_tau = motor_tau + damping_tau
        self._record_coupled_torque_command(
            motor_positions=physical_position,
            motor_velocities=physical_velocity,
            transformed_torques=transformed_motor_tau,
            motor_kd_gains=motor_kd,
            damping_torques=damping_tau,
            requested_torques=requested_motor_tau,
            limited_torques=limited_motor_tau,
            applied_torques=motor_tau,
            estimated_total_torques=estimated_total_tau,
        )
        return (
            motor_position_target,
            motor_velocity_command,
            motor_kp,
            motor_kd,
            motor_tau,
        )

    def _limit_coupled_motor_torques(self, requested: np.ndarray) -> np.ndarray:
        applied = np.asarray(requested, dtype=np.float64).copy()
        transform = self._joint_transform
        if transform is None:
            return applied
        for pair in transform.transformed_pairs:
            scale = 1.0
            for index in pair:
                joint = self.config.arm_joints[index]
                limits = [
                    value
                    for value in (joint.effort_limit, joint.coupled_effort_limit)
                    if value is not None
                ]
                if limits and abs(float(applied[index])) > 0.0:
                    scale = min(
                        scale,
                        min(limits) / abs(float(applied[index])),
                    )
            if scale < 1.0:
                applied[list(pair)] *= scale
        return applied

    def _slew_coupled_motor_torques(
        self,
        requested: np.ndarray,
        *,
        hold_pairs: frozenset[tuple[int, int]] = frozenset(),
    ) -> np.ndarray:
        desired = np.asarray(requested, dtype=np.float64).copy()
        transform = self._joint_transform
        if transform is None:
            return desired
        if not self._enabled:
            self._coupled_previous_motor_tau = desired.copy()
            return desired

        previous = self._coupled_previous_motor_tau
        applied = desired.copy()
        period = 1.0 / self.config.control_hz
        for pair in transform.transformed_pairs:
            joints = [self.config.arm_joints[index] for index in pair]
            if any(
                joint.coupled_torque_rise_rate is None
                or joint.coupled_torque_brake_rate is None
                for joint in joints
            ):
                continue
            rise_rate = min(
                float(joint.coupled_torque_rise_rate) for joint in joints
            )
            if pair in hold_pairs:
                rise_rate = min(
                    float(joint.coupled_hold_torque_rise_rate)
                    for joint in joints
                    if joint.coupled_hold_torque_rise_rate is not None
                )
            brake_rate = min(
                float(joint.coupled_torque_brake_rate) for joint in joints
            )
            pair_indices = list(pair)
            target = desired[pair_indices]
            prior = previous[pair_indices]
            target_peak = float(np.max(np.abs(target)))
            prior_peak = float(np.max(np.abs(prior)))
            epsilon = 1e-12

            if target_peak <= epsilon:
                next_peak = max(0.0, prior_peak - brake_rate * period)
                applied[pair_indices] = (
                    np.zeros(2)
                    if prior_peak <= epsilon
                    else prior * (next_peak / prior_peak)
                )
                continue
            if prior_peak > epsilon and float(np.dot(prior, target)) <= 0.0:
                # 方向反转前必须先释放已积累的力。沿旧向量制动可避免在同一周期混用
                # 相反的 A/B 比例；新方向的向量从零开始建立。
                next_peak = max(0.0, prior_peak - brake_rate * period)
                applied[pair_indices] = prior * (next_peak / prior_peak)
                continue

            rate = rise_rate if target_peak > prior_peak else brake_rate
            maximum_delta = rate * period
            next_peak = float(
                np.clip(
                    target_peak,
                    max(0.0, prior_peak - maximum_delta),
                    prior_peak + maximum_delta,
                )
            )
            # 对电机对使用同一个缩放系数，使请求的 A/B 力矩方向在正常上升和衰减
            # 过程中保持不变。
            applied[pair_indices] = target * (next_peak / target_peak)
        self._coupled_previous_motor_tau = applied.copy()
        return applied

    def _reset_coupled_motor_torque_state(self) -> None:
        self._coupled_previous_motor_tau = np.zeros(
            len(self.config.arm_joints),
            dtype=np.float64,
        )
        self._coupled_filtered_virtual_velocity.fill(0.0)
        self._coupled_velocity_filter_initialized.fill(False)
        self._coupled_hold_candidate_since.clear()

    def _filter_coupled_virtual_velocities(
        self,
        measured: np.ndarray,
    ) -> np.ndarray:
        filtered = np.asarray(measured, dtype=np.float64).copy()
        transform = self._joint_transform
        if transform is None:
            return filtered
        period = 1.0 / self.config.control_hz
        for pair in transform.transformed_pairs:
            pair_indices = list(pair)
            time_constant = max(
                self.config.arm_joints[index].coupled_velocity_filter_s
                for index in pair
            )
            if time_constant <= 0.0:
                self._coupled_filtered_virtual_velocity[pair_indices] = filtered[
                    pair_indices
                ]
                self._coupled_velocity_filter_initialized[pair_indices] = True
                continue
            if not np.all(
                self._coupled_velocity_filter_initialized[pair_indices]
            ):
                self._coupled_filtered_virtual_velocity[pair_indices] = filtered[
                    pair_indices
                ]
                self._coupled_velocity_filter_initialized[pair_indices] = True
            else:
                alpha = period / (time_constant + period)
                self._coupled_filtered_virtual_velocity[pair_indices] += alpha * (
                    filtered[pair_indices]
                    - self._coupled_filtered_virtual_velocity[pair_indices]
                )
            filtered[pair_indices] = self._coupled_filtered_virtual_velocity[
                pair_indices
            ]
        return filtered

    def _record_coupled_torque_command(
        self,
        *,
        motor_positions: np.ndarray,
        motor_velocities: np.ndarray,
        transformed_torques: np.ndarray,
        motor_kd_gains: np.ndarray,
        damping_torques: np.ndarray,
        requested_torques: np.ndarray,
        limited_torques: np.ndarray,
        applied_torques: np.ndarray,
        estimated_total_torques: np.ndarray,
    ) -> None:
        transform = self._joint_transform
        if transform is None:
            return
        indices = sorted(transform.transformed_indices)
        saturated = [
            index
            for index in indices
            if not math.isclose(
                float(requested_torques[index]),
                float(limited_torques[index]),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ]
        scales = [
            abs(float(limited_torques[index]))
            / abs(float(requested_torques[index]))
            for index in indices
            if abs(float(requested_torques[index])) > 1e-12
        ]
        saturation_scale = min(scales, default=1.0)
        now = time.monotonic()
        status = CoupledTorqueSaturation(
            active=bool(saturated),
            motor_names=tuple(
                self.config.arm_joints[index].name for index in saturated
            ),
            requested_torques=tuple(
                float(requested_torques[index]) for index in indices
            ),
            limited_torques=tuple(
                float(limited_torques[index]) for index in indices
            ),
            applied_torques=tuple(
                float(applied_torques[index]) for index in indices
            ),
            saturation_scale=saturation_scale,
            timestamp=now,
        )
        telemetry = CoupledTorqueTelemetry(
            motor_names=tuple(
                self.config.arm_joints[index].name for index in indices
            ),
            motor_positions=tuple(float(motor_positions[index]) for index in indices),
            motor_velocities=tuple(
                float(motor_velocities[index]) for index in indices
            ),
            transformed_torques=tuple(
                float(transformed_torques[index]) for index in indices
            ),
            motor_kd_gains=tuple(
                float(motor_kd_gains[index]) for index in indices
            ),
            damping_torques=tuple(
                float(damping_torques[index]) for index in indices
            ),
            requested_torques=status.requested_torques,
            limited_torques=status.limited_torques,
            applied_torques=status.applied_torques,
            estimated_total_torques=tuple(
                float(estimated_total_torques[index]) for index in indices
            ),
            saturation_scale=saturation_scale,
            timestamp=now,
        )
        with self._state_lock:
            previous = self._coupled_torque_saturation
            self._coupled_torque_saturation = status
            self._coupled_torque_telemetry = telemetry
            self._coupled_control_stats = replace(
                self._coupled_control_stats,
                torque_command_count=(
                    self._coupled_control_stats.torque_command_count + 1
                ),
                torque_saturation_count=(
                    self._coupled_control_stats.torque_saturation_count
                    + int(status.active)
                ),
            )
        if status.active and (
            not previous.active or previous.motor_names != status.motor_names
        ):
            _LOG.debug(
                "coupled motor torque saturated: %s",
                ", ".join(status.motor_names),
            )
        elif previous.active and not status.active:
            _LOG.debug("coupled motor torque saturation cleared")

    def _read_cached_arm_motor_state(self) -> tuple[np.ndarray, np.ndarray]:
        position, velocity, _ = self.robot.get_state(
            request_feedback=False,
            require_complete=True,
            joint_names=list(self.config.joint_names),
        )
        statuses = self.robot.get_status_codes(
            joint_names=list(self.config.joint_names),
        )
        disabled = [name for name, status in statuses.items() if status == 0]
        if disabled:
            raise RuntimeError(
                "coupled MIT motor unexpectedly disabled: " + ", ".join(disabled)
            )
        transform = self._joint_transform
        assert transform is not None
        coupled_names = [
            self.config.arm_joints[index].name
            for index in sorted(transform.transformed_indices)
        ]
        feedback_stats = self.robot.get_feedback_stats(
            joint_names=coupled_names,
        )
        ages_s = {
            name: float(stats.age_ns) * 1e-9
            for name, stats in feedback_stats.items()
        }
        stale = [
            name
            for name, stats in feedback_stats.items()
            if (
                not stats.has_feedback
                or ages_s[name] > self.config.max_cached_feedback_age_s
            )
        ]
        counts = {
            name: int(stats.update_count)
            for name, stats in feedback_stats.items()
        }
        with self._state_lock:
            previous_counts = self._coupled_feedback_update_counts
            stalled = bool(previous_counts) and any(
                counts.get(name, -1) <= previous_counts.get(name, -1)
                for name in coupled_names
            )
            self._coupled_feedback_update_counts = counts
            self._coupled_control_stats = replace(
                self._coupled_control_stats,
                feedback_stall_cycles=(
                    self._coupled_control_stats.feedback_stall_cycles
                    + int(stalled)
                ),
                stale_feedback_faults=(
                    self._coupled_control_stats.stale_feedback_faults
                    + int(bool(stale))
                ),
                maximum_feedback_age_s=max(
                    self._coupled_control_stats.maximum_feedback_age_s,
                    max(ages_s.values(), default=0.0),
                ),
            )
        if stale:
            details = ", ".join(
                f"{name}={ages_s[name] * 1000.0:.1f}ms"
                for name in stale
            )
            raise StaleFeedbackError(
                "coupled MIT feedback is stale: "
                f"{details}; limit={self.config.max_cached_feedback_age_s * 1000.0:.1f}ms",
                operation="read_cached_feedback",
                motor_names=tuple(stale),
                retryable=False,
                feedback_ages_s={name: ages_s[name] for name in stale},
                age_limit_s=self.config.max_cached_feedback_age_s,
            )
        return np.asarray(position, dtype=np.float64), np.asarray(
            velocity, dtype=np.float64
        )

    def _send_mit_command(self, command: MitCommand, *, strict: bool) -> None:
        with self._io_lock:
            if self._joint_transform is None:
                vectors = self._compose_mit_motor_command(command)
            else:
                motor_position, motor_velocity = self._read_cached_arm_motor_state()
                vectors = self._compose_mit_motor_command(
                    command,
                    motor_positions=motor_position,
                    motor_velocities=motor_velocity,
                )
            position, velocity, kp, kd, torque = vectors
            self.robot.arm.send_mit(
                position,
                vel=velocity,
                kp=kp,
                kd=kd,
                tau=torque,
                strict=strict,
            )

    def _start_coupled_control(self) -> None:
        if self._joint_transform is None or self._mode != "mit" or not self._enabled:
            return
        thread = self._coupled_control_thread
        if thread is not None and thread.is_alive():
            return
        self._coupled_control_stop.clear()
        self._coupled_control_wakeup.clear()
        with self._state_lock:
            self._coupled_feedback_update_counts = {}
            self._coupled_control_stats = CoupledControlStats(
                target_hz=self.config.control_hz,
                achieved_hz=0.0,
                cycle_count=0,
                overrun_count=0,
                feedback_stall_cycles=0,
                stale_feedback_faults=0,
                maximum_feedback_age_s=0.0,
                torque_command_count=0,
                torque_saturation_count=0,
            )
        self._coupled_control_thread = threading.Thread(
            target=self._coupled_control_loop,
            name="arx-d-can-coupled-mit-control",
            daemon=True,
        )
        self._coupled_control_thread.start()

    def _stop_coupled_control(self) -> None:
        self._coupled_control_stop.set()
        self._coupled_control_wakeup.set()
        thread = self._coupled_control_thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=1.0)
        if thread is not threading.current_thread():
            self._coupled_control_thread = None

    def _coupled_control_loop(self) -> None:
        period = 1.0 / self.config.control_hz
        next_tick = time.perf_counter()
        first_cycle_started: float | None = None
        while not self._coupled_control_stop.is_set():
            with self._state_lock:
                command = self._last_mit_command
                enabled = self._enabled
                faulted = self._faulted
                safe_holding = self._safe_holding
            if not enabled or (faulted and not safe_holding):
                return
            if command is None or safe_holding:
                self._coupled_control_wakeup.wait(period)
                self._coupled_control_wakeup.clear()
                next_tick = time.perf_counter()
                continue
            try:
                cycle_started = time.perf_counter()
                if first_cycle_started is None:
                    first_cycle_started = cycle_started
                self._send_mit_command(command, strict=True)
            except StaleFeedbackError as exc:
                self._record_communication_error(exc, using_fallback=False)
                self._trip_fault(str(exc))
            except Exception as exc:
                if isinstance(exc, CommunicationError):
                    self._record_communication_error(exc, using_fallback=None)
                self._begin_safe_hold(f"coupled MIT control failed: {exc}")
            cycle_finished = time.perf_counter()
            with self._state_lock:
                cycles = self._coupled_control_stats.cycle_count + 1
                elapsed = max(
                    cycle_finished
                    - (first_cycle_started or cycle_started)
                    + period,
                    period,
                )
                self._coupled_control_stats = replace(
                    self._coupled_control_stats,
                    achieved_hz=cycles / elapsed,
                    cycle_count=cycles,
                    overrun_count=(
                        self._coupled_control_stats.overrun_count
                        + int(cycle_finished - cycle_started > period)
                    ),
                )
            next_tick += period
            delay = next_tick - time.perf_counter()
            if delay <= 0.0:
                next_tick = time.perf_counter()
                continue
            self._coupled_control_stop.wait(delay)


