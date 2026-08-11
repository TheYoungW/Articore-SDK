"""适用于 :class:`ArxDCanArm` 的安全同步重力补偿模式。"""
from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
import threading
import time

import numpy as np

from ..sdk import ArxDCanArm


GravityProvider = Callable[[np.ndarray], np.ndarray]


@dataclass(slots=True, frozen=True)
class GravityCompensationSample:
    """重力补偿模式下的一组反馈与命令样本。"""

    elapsed_s: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    commanded_torques: tuple[float, ...]


def _gain_vector(
    value: float | Sequence[float],
    *,
    joint_count: int,
    name: str,
    allow_negative: bool = False,
) -> np.ndarray:
    values = np.asarray(value, dtype=np.float64)
    if values.ndim == 0:
        values = np.full(joint_count, float(values), dtype=np.float64)
    else:
        values = values.reshape(-1)
    if len(values) != joint_count:
        raise ValueError(f"expected {joint_count} {name} values, got {len(values)}")
    if np.any(~np.isfinite(values)):
        raise ValueError(f"{name} values must be finite")
    if not allow_negative and np.any(values < 0.0):
        raise ValueError(f"{name} values must be finite and non-negative")
    return values


class GravityCompensationMode:
    """以零位置刚度运行机械臂 MIT 重力补偿。

    此模式从当前位置的 MIT 保持开始，逐渐用重力前馈替代配置的 Kp/Kd，并在失能前
    执行反向过渡。``damping=0`` 时 Kp 和 Kd 均发送零；设置较小的正阻尼值时，
    Kp 保持为零，同时加入速度阻尼。
    """

    def __init__(
        self,
        arm: ArxDCanArm,
        *,
        hz: float = 100.0,
        transition_seconds: float = 0.0,
        settle_seconds: float = 0.0,
        gravity_scale: float = 1.0,
        joint_scales: Sequence[float] | None = None,
        damping: float | Sequence[float] = 0.0,
        feedback_check_hz: float = 10.0,
        gravity_provider: GravityProvider | None = None,
    ) -> None:
        self.arm = arm
        self._joint_count = len(arm.joint_names)
        if self._joint_count == 0:
            raise ValueError("gravity compensation requires at least one arm joint")
        if not math.isfinite(hz) or hz <= 0.0:
            raise ValueError("hz must be finite and positive")
        if not math.isfinite(transition_seconds) or transition_seconds < 0.0:
            raise ValueError("transition_seconds must be finite and non-negative")
        if not math.isfinite(settle_seconds) or settle_seconds < 0.0:
            raise ValueError("settle_seconds must be finite and non-negative")
        if not math.isfinite(gravity_scale) or gravity_scale < 0.0:
            raise ValueError("gravity_scale must be finite and non-negative")
        if not math.isfinite(feedback_check_hz) or feedback_check_hz < 0.0:
            raise ValueError("feedback_check_hz must be finite and non-negative")
        self.hz = float(hz)
        self.transition_seconds = float(transition_seconds)
        self.settle_seconds = float(settle_seconds)
        self.gravity_scale = float(gravity_scale)
        self.feedback_check_hz = float(feedback_check_hz)
        self._period = 1.0 / self.hz
        self._zeros = np.zeros(self._joint_count, dtype=np.float64)
        self._damping = _gain_vector(
            damping,
            joint_count=self._joint_count,
            name="damping",
        )
        self._joint_scales = _gain_vector(
            1.0 if joint_scales is None else joint_scales,
            joint_count=self._joint_count,
            name="joint_scales",
            allow_negative=True,
        )
        self._default_kp = np.asarray(
            [joint.mit_kp for joint in arm.config.arm_joints],
            dtype=np.float64,
        )
        self._default_kd = np.asarray(
            [joint.mit_kd for joint in arm.config.arm_joints],
            dtype=np.float64,
        )
        self._gravity_provider = gravity_provider or self._build_gravity_provider()
        self._started = False
        self._owns_connection = False
        self._active_started = 0.0
        self._last_sample: GravityCompensationSample | None = None
        self._transition_safe = True
        self._feedback_monitor_stop = threading.Event()
        self._feedback_monitor_thread: threading.Thread | None = None

    @property
    def active(self) -> bool:
        return self._started

    @property
    def last_sample(self) -> GravityCompensationSample | None:
        return self._last_sample

    def _build_gravity_provider(self) -> GravityProvider:
        if self.arm.config.urdf_path is None:
            raise ValueError("gravity compensation requires a URDF model")
        try:
            from ..dynamics import compute_generalized_gravity
            from ..kinematics import load_robot_model
        except (ImportError, ModuleNotFoundError) as exc:
            raise RuntimeError(
                "gravity compensation requires Pinocchio; install the "
                "'dynamics' extra. If a ROS environment shadows the active "
                "Python environment, run with PYTHONPATH unset."
            ) from exc

        model = load_robot_model(
            self.arm.config.urdf_path,
            controlled_joint_names=self.arm.joint_names,
        )
        data = model.createData()

        def compute(positions: np.ndarray) -> np.ndarray:
            return compute_generalized_gravity(model, positions, data)

        return compute

    def _gravity_torques(self, positions: np.ndarray) -> np.ndarray:
        raw = np.asarray(self._gravity_provider(positions), dtype=np.float64).reshape(-1)
        if len(raw) != self._joint_count:
            raise RuntimeError(
                f"gravity provider returned {len(raw)} torques, "
                f"expected {self._joint_count}"
            )
        torques = self.gravity_scale * self._joint_scales * raw
        if np.any(~np.isfinite(torques)):
            raise RuntimeError("gravity provider returned non-finite torques")
        return torques

    def _read_checked_state(self, *, request_feedback: bool = True):
        state = (
            self.arm.read_state()
            if request_feedback
            else self.arm.read_cached_state()
        )
        if self.arm.faulted or self.arm.safe_holding:
            raise RuntimeError(
                self.arm.fault_reason or "arm entered fault-safe holding"
            )
        positions = np.asarray(state.arm.positions, dtype=np.float64)
        velocities = np.asarray(state.arm.velocities, dtype=np.float64)
        if len(positions) != self._joint_count or len(velocities) != self._joint_count:
            raise RuntimeError("arm feedback joint count changed")
        if np.any(~np.isfinite(positions)) or np.any(~np.isfinite(velocities)):
            raise RuntimeError("arm feedback contains non-finite values")
        return state, positions, velocities

    def _feedback_monitor_loop(self) -> None:
        period = 1.0 / self.feedback_check_hz
        while not self._feedback_monitor_stop.wait(period):
            if not self._started or not self.arm.connected:
                return
            try:
                refresh = getattr(self.arm, "refresh_feedback_background", None)
                if refresh is None:
                    self.arm.read_state()
                else:
                    refresh()
            except Exception:
                # ArxDCanArm 会记录连续反馈失败，并在达到配置阈值时进入安全保持。
                # 实时循环无需等待本次请求，即可观察到该状态。
                continue

    def _start_feedback_monitor(self) -> None:
        if self.feedback_check_hz <= 0.0:
            return
        thread = self._feedback_monitor_thread
        if thread is not None and thread.is_alive():
            return
        self._feedback_monitor_stop.clear()
        self._feedback_monitor_thread = threading.Thread(
            target=self._feedback_monitor_loop,
            name="arx-d-can-feedback-monitor",
            daemon=True,
        )
        self._feedback_monitor_thread.start()

    def _stop_feedback_monitor(self) -> None:
        self._feedback_monitor_stop.set()
        thread = self._feedback_monitor_thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join()
        self._feedback_monitor_thread = None

    def _send(
        self,
        positions: np.ndarray,
        torques: np.ndarray,
        *,
        kp: np.ndarray,
        kd: np.ndarray,
        require_enabled: bool = True,
    ) -> None:
        self.arm.send_joint_positions(
            positions,
            velocities=self._zeros,
            torques=torques,
            mit_kp=kp,
            mit_kd=kd,
            require_enabled=require_enabled,
        )

    def _gripper_position(self, state) -> float | None:
        """机械臂控制夹爪时，返回有限的夹爪位置。"""
        if not getattr(self.arm, "enable_gripper", False):
            return None
        if state.gripper is None:
            raise RuntimeError("gripper feedback is unavailable")
        position = float(state.gripper.position)
        if not math.isfinite(position):
            raise RuntimeError("gripper feedback contains a non-finite position")
        return position

    def _refresh_gripper_command(self, state) -> None:
        """跟随反馈位置刷新命令，避免已使能夹爪超时。"""
        position = self._gripper_position(state)
        if position is not None:
            self.arm.set_gripper_motor_value(position)

    def _run_transition(self, hold_position: np.ndarray, *, entering: bool) -> None:
        if self.transition_seconds <= 0.0:
            state, positions, _ = self._read_checked_state()
            gravity = self._gravity_torques(positions)
            alpha = 1.0 if entering else 0.0
            self._send(
                hold_position,
                alpha * gravity,
                kp=(1.0 - alpha) * self._default_kp,
                kd=(1.0 - alpha) * self._default_kd + alpha * self._damping,
            )
            self._refresh_gripper_command(state)
            return

        steps = max(1, math.ceil(self.transition_seconds * self.hz))
        started = time.monotonic()
        for index in range(steps + 1):
            state, positions, _ = self._read_checked_state()
            gravity = self._gravity_torques(positions)
            progress = index / steps
            gravity_alpha = progress if entering else 1.0 - progress
            self._send(
                hold_position,
                gravity_alpha * gravity,
                kp=(1.0 - gravity_alpha) * self._default_kp,
                kd=(
                    (1.0 - gravity_alpha) * self._default_kd
                    + gravity_alpha * self._damping
                ),
            )
            self._refresh_gripper_command(state)
            deadline = started + (index + 1) * self._period
            time.sleep(max(0.0, deadline - time.monotonic()))

    def start(self) -> GravityCompensationSample:
        """连接、使能并过渡到重力补偿状态。"""
        if self._started:
            raise RuntimeError("gravity compensation is already active")
        self._owns_connection = not self.arm.connected
        try:
            if self._owns_connection:
                self.arm.connect()
            if self.arm.enabled:
                raise RuntimeError(
                    "arm must be disabled before entering gravity compensation"
                )
            state, initial_position, _ = self._read_checked_state()
            initial_gripper_position = self._gripper_position(state)
            self._gravity_torques(initial_position)
            self.arm.configure("mit")
            self._send(
                initial_position,
                self._zeros,
                kp=self._default_kp,
                kd=self._default_kd,
                require_enabled=False,
            )
            self.arm.enable()
            if initial_gripper_position is not None:
                # 夹爪与机械臂同时使能，因此立即写入首条 MIT 命令，避免它在第一个
                # 重力补偿反馈周期前超时。
                self.arm.set_gripper_motor_value(initial_gripper_position)

            settle_deadline = time.monotonic() + self.settle_seconds
            next_tick = time.monotonic()
            while time.monotonic() < settle_deadline:
                state, _, _ = self._read_checked_state()
                self._send(
                    initial_position,
                    self._zeros,
                    kp=self._default_kp,
                    kd=self._default_kd,
                )
                self._refresh_gripper_command(state)
                next_tick += self._period
                time.sleep(max(0.0, next_tick - time.monotonic()))

            self._run_transition(initial_position, entering=True)
            self._started = True
            self._transition_safe = True
            self._active_started = time.monotonic()
            self._start_feedback_monitor()
            return self.step()
        except Exception:
            self._stop_feedback_monitor()
            if self.arm.connected and self.arm.enabled:
                try:
                    self.arm.disable()
                except Exception:
                    pass
            if self._owns_connection and self.arm.connected:
                try:
                    self.arm.close()
                except Exception:
                    pass
            self._started = False
            raise

    def step(self) -> GravityCompensationSample:
        """发送一帧零刚度重力补偿命令。"""
        if not self._started:
            raise RuntimeError("gravity compensation is not active")
        try:
            state, positions, velocities = self._read_checked_state(
                request_feedback=False
            )
            torques = self._gravity_torques(positions)
            self._send(
                positions,
                torques,
                kp=self._zeros,
                kd=self._damping,
            )
            self._refresh_gripper_command(state)
        except Exception:
            self._transition_safe = False
            raise
        sample = GravityCompensationSample(
            elapsed_s=time.monotonic() - self._active_started,
            positions=tuple(float(value) for value in positions),
            velocities=tuple(float(value) for value in velocities),
            commanded_torques=tuple(float(value) for value in torques),
        )
        self._last_sample = sample
        return sample

    def run(
        self,
        *,
        seconds: float = 0.0,
        on_sample: Callable[[GravityCompensationSample], None] | None = None,
    ) -> None:
        """持续刷新重力补偿，直到超时或收到 ``KeyboardInterrupt``。"""
        if not self._started:
            raise RuntimeError("gravity compensation is not active")
        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError("seconds must be finite and non-negative")
        deadline = None if seconds == 0.0 else time.monotonic() + seconds
        next_tick = time.monotonic()
        while deadline is None or time.monotonic() < deadline:
            sample = self.step()
            if on_sample is not None:
                on_sample(sample)
            next_tick += self._period
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def stop(self) -> None:
        """恢复配置的 MIT 增益，然后失能机械臂。"""
        transition_error: Exception | None = None
        self._stop_feedback_monitor()
        try:
            if (
                self._started
                and self.arm.connected
                and self.arm.enabled
                and not self.arm.faulted
                and not self.arm.safe_holding
                and self._transition_safe
            ):
                _, hold_position, _ = self._read_checked_state()
                self._run_transition(hold_position, entering=False)
        except Exception as exc:
            transition_error = exc
        finally:
            if self.arm.connected and self.arm.enabled:
                try:
                    self.arm.disable()
                except Exception as exc:
                    if transition_error is None:
                        transition_error = exc
            self._started = False
            self._transition_safe = True
        if transition_error is not None:
            raise RuntimeError(
                f"gravity-compensation shutdown failed: {transition_error}"
            ) from transition_error

    def shutdown(self) -> None:
        """停止当前模式，并关闭由本对象建立的连接。"""
        error: Exception | None = None
        try:
            self.stop()
        except Exception as exc:
            error = exc
        if self._owns_connection and self.arm.connected:
            try:
                self.arm.close()
            except Exception as exc:
                if error is None:
                    error = exc
        if error is not None:
            raise error

    def __enter__(self) -> "GravityCompensationMode":
        self.start()
        return self

    def __exit__(self, *_args) -> None:
        self.shutdown()


__all__ = ["GravityCompensationMode", "GravityCompensationSample"]
