"""Standalone ARX-D-CAN arm SDK."""
from __future__ import annotations

import logging
import math
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Sequence

import numpy as np

from ..actuator import ArxDCan, JointCfg, load_cfg
from ..driver import build_scan_command, parse_scan_ids
from ..kinematics.coupled_joint_transform import CoupledJointTransform
from .gripper_force_control import (
    GripperControlState,
    GripperForceControlConfig,
    GripperForceController,
)


@dataclass(slots=True, frozen=True)
class MotorState:
    name: str
    motor_id: int
    feedback_id: int
    position: float
    velocity: float
    torque: float = 0.0


@dataclass(slots=True, frozen=True)
class JointState:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    torques: tuple[float, ...]


@dataclass(slots=True, frozen=True)
class ArxDCanState:
    arm: JointState
    gripper: MotorState | None = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.arm.names

    @property
    def positions(self) -> tuple[float, ...]:
        return self.arm.positions


@dataclass(slots=True, frozen=True)
class MitCommand:
    """One complete logical-joint MIT command retained by the driver."""

    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    feedforward_torques: tuple[float, ...]
    timestamp: float


@dataclass(slots=True, frozen=True)
class CoupledTorqueSaturation:
    """Latest physical-motor torque limiting result for coupled joints."""

    active: bool
    motor_names: tuple[str, ...]
    requested_torques: tuple[float, ...]
    limited_torques: tuple[float, ...]
    applied_torques: tuple[float, ...]
    saturation_scale: float
    timestamp: float


@dataclass(slots=True, frozen=True)
class CoupledTorqueTelemetry:
    """Latest coupled-motor command stages and measured motor state."""

    motor_names: tuple[str, ...]
    motor_positions: tuple[float, ...]
    motor_velocities: tuple[float, ...]
    transformed_torques: tuple[float, ...]
    motor_kd_gains: tuple[float, ...]
    damping_torques: tuple[float, ...]
    requested_torques: tuple[float, ...]
    limited_torques: tuple[float, ...]
    applied_torques: tuple[float, ...]
    estimated_total_torques: tuple[float, ...]
    saturation_scale: float
    timestamp: float


@dataclass(slots=True, frozen=True)
class CoupledControlStats:
    """Observed timing and feedback health for the coupled MIT inner loop."""

    target_hz: float
    achieved_hz: float
    cycle_count: int
    overrun_count: int
    feedback_stall_cycles: int
    stale_feedback_faults: int
    maximum_feedback_age_s: float
    torque_command_count: int
    torque_saturation_count: int

    @property
    def torque_saturation_rate(self) -> float:
        if self.torque_command_count <= 0:
            return 0.0
        return self.torque_saturation_count / self.torque_command_count


_LOG = logging.getLogger(__name__)


class _StaleCoupledFeedbackError(RuntimeError):
    """Cached A/B feedback is too old for safe virtual-PD control."""


@dataclass(slots=True, frozen=True)
class JointMotorConfig:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    mit_kp: float
    mit_kd: float
    pv_vel_kp: float
    pv_vel_ki: float
    pv_pos_kp: float
    pv_pos_ki: float
    pv_vlim: float
    direction: float = 1.0
    torque_range: float | None = None
    effort_limit: float | None = None
    coupled_effort_limit: float | None = None
    coupled_motor_kd: float = 0.0
    coupled_velocity_filter_s: float = 0.0
    coupled_torque_rise_rate: float | None = None
    coupled_hold_torque_rise_rate: float | None = None
    coupled_torque_brake_rate: float | None = None
    velocity_range: float | None = None
    lower_limit: float | None = None
    upper_limit: float | None = None


@dataclass(slots=True, frozen=True)
class ArxDCanConfig:
    port: str = "/dev/ttyACM0"
    transport: str = "auto"
    baud: int = 1_000_000
    control_hz: float = 100.0
    arm_control_mode: str = "posvel"
    arm_joints: tuple[JointMotorConfig, ...] = ()
    gripper: JointMotorConfig | None = None
    gripper_open_value: float = 2.64
    gripper_closed_value: float = 0.0
    gripper_force_control_enabled: bool = False
    gripper_force_control: GripperForceControlConfig = field(
        default_factory=GripperForceControlConfig
    )
    watchdog_enabled: bool = True
    command_timeout_s: float = 0.25
    enable_grace_s: float = 2.0
    watchdog_poll_s: float = 0.02
    watchdog_action: str = "safe_hold"
    safe_hold_hz: float = 100.0
    feedback_fault_threshold: int = 3
    max_cached_feedback_age_s: float = 0.02
    name: str = "ARX-D-CAN"
    model: str = "custom"
    hardware_config_path: str | None = None
    urdf_path: str | None = None
    joint_transform_path: str | None = None
    end_effector_frame: str = "gripper_end"

    @property
    def channel(self) -> str:
        """Transport channel; ``port`` is retained for API compatibility."""
        return self.port

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.arm_joints)


def _joint_from_yaml(joint: JointCfg) -> JointMotorConfig:
    return JointMotorConfig(
        name=joint.name,
        motor_id=joint.motor_id,
        feedback_id=joint.feedback_id,
        model=joint.model,
        mit_kp=joint.kp,
        mit_kd=joint.kd,
        pv_vel_kp=joint.vel_kp,
        pv_vel_ki=joint.vel_ki,
        pv_pos_kp=joint.pos_kp,
        pv_pos_ki=joint.pos_ki,
        pv_vlim=joint.vlim,
        direction=joint.direction,
        torque_range=joint.torque_range,
        effort_limit=joint.effort_limit,
        coupled_effort_limit=joint.coupled_effort_limit,
        coupled_motor_kd=joint.coupled_motor_kd,
        coupled_velocity_filter_s=joint.coupled_velocity_filter_s,
        coupled_torque_rise_rate=joint.coupled_torque_rise_rate,
        coupled_hold_torque_rise_rate=joint.coupled_hold_torque_rise_rate,
        coupled_torque_brake_rate=joint.coupled_torque_brake_rate,
        velocity_range=joint.velocity_range,
        lower_limit=joint.lower_limit,
        upper_limit=joint.upper_limit,
    )


def _config_bool(value: object, *, name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "on", "1"}:
            return True
        if normalized in {"false", "no", "off", "0"}:
            return False
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    raise ValueError(f"{name} must be a boolean")


def _connection_channel(port: str | None, channel: str | None) -> str | None:
    if port is not None and channel is not None and port != channel:
        raise ValueError("port and channel are aliases and cannot have different values")
    return channel if channel is not None else port


def _mit_gain_vector(
    value: float | Sequence[float] | None,
    *,
    joint_count: int,
    name: str,
) -> np.ndarray | None:
    if value is None:
        return None
    values = np.asarray(value, dtype=np.float64)
    if values.ndim == 0:
        values = np.full(joint_count, float(values), dtype=np.float64)
    else:
        values = values.reshape(-1)
        if len(values) != joint_count:
            raise ValueError(
                f"expected {joint_count} MIT {name} values, got {len(values)}"
            )
    if np.any(~np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError(f"MIT {name} values must be finite and non-negative")
    return values


def _config_from_loaded(
    data: dict,
    *,
    port: str | None = None,
    baud: int | None = None,
    transport: str | None = None,
    control_hz: float | None = None,
    arm_control_mode: str = "posvel",
) -> ArxDCanConfig:
    joints_by_name = {
        joint.name: _joint_from_yaml(joint)
        for joint in data.get("joints", [])
    }
    groups = data.get("groups", {})
    arm_names = list(groups.get("arm", {}).get("joints", []))
    gripper_names = list(groups.get("gripper", {}).get("joints", []))
    gripper_mapping = data.get("gripper_mapping", {}) or {}
    force_control = data.get("gripper_force_control", {}) or {}
    safety = data.get("safety", {}) or {}

    return ArxDCanConfig(
        name=str(data.get("name", "ARX-D-CAN")),
        model=str(data.get("model", "custom")),
        hardware_config_path=(
            None if data.get("hardware_path") is None else str(data["hardware_path"])
        ),
        urdf_path=None if data.get("urdf_path") is None else str(data["urdf_path"]),
        joint_transform_path=(
            None
            if data.get("joint_transform_path") is None
            else str(data["joint_transform_path"])
        ),
        end_effector_frame=str(data.get("end_effector_frame", "gripper_end")),
        port=str(data.get("channel", "/dev/ttyACM0") if port is None else port),
        transport=str(
            data.get("transport", "auto") if transport is None else transport
        ),
        baud=int(data.get("baud", 1_000_000) if baud is None else baud),
        control_hz=float(
            data.get("rate", 100.0) if control_hz is None else control_hz
        ),
        arm_control_mode=arm_control_mode,
        arm_joints=tuple(joints_by_name[name] for name in arm_names),
        gripper=joints_by_name.get(gripper_names[0]) if gripper_names else None,
        gripper_closed_value=float(gripper_mapping.get("closed_value", 0.0)),
        gripper_open_value=float(gripper_mapping.get("open_value", 2.64)),
        gripper_force_control_enabled=_config_bool(
            force_control.get("enabled", False),
            name="gripper_force_control.enabled",
        ),
        gripper_force_control=GripperForceControlConfig(
            close_speed=float(force_control.get("close_speed", 1.0)),
            contact_torque=float(force_control.get("contact_torque", 0.8)),
            overload_torque=float(force_control.get("overload_torque", 1.5)),
            motion_window_s=float(force_control.get("motion_window_s", 0.2)),
            stall_movement=float(force_control.get("stall_movement", 0.01)),
            min_position_error=float(force_control.get("min_position_error", 0.05)),
            contact_hold_s=float(force_control.get("contact_hold_s", 0.2)),
            overload_hold_s=float(force_control.get("overload_hold_s", 0.05)),
            hold_offset=float(force_control.get("hold_offset", 0.08)),
            retreat_distance=float(force_control.get("retreat_distance", 0.15)),
            max_step_interval_s=float(force_control.get("max_step_interval_s", 0.05)),
            overload_retreat_interval_s=float(
                force_control.get("overload_retreat_interval_s", 0.1)
            ),
            hold_kp=float(force_control.get("hold_kp", 2.0)),
            hold_kd=float(force_control.get("hold_kd", 0.5)),
        ),
        watchdog_enabled=_config_bool(
            safety.get("watchdog_enabled", True),
            name="safety.watchdog_enabled",
        ),
        command_timeout_s=float(safety.get("command_timeout_s", 0.25)),
        enable_grace_s=float(safety.get("enable_grace_s", 2.0)),
        watchdog_poll_s=float(safety.get("watchdog_poll_s", 0.02)),
        watchdog_action=str(safety.get("watchdog_action", "safe_hold")),
        safe_hold_hz=float(safety.get("safe_hold_hz", 100.0)),
        feedback_fault_threshold=int(safety.get("feedback_fault_threshold", 3)),
        max_cached_feedback_age_s=float(
            safety.get("max_cached_feedback_age_s", 0.02)
        ),
    )


def default_config(
    *,
    model: str | None = None,
    config_path: str | Path | None = None,
    port: str | None = None,
    channel: str | None = None,
    baud: int | None = None,
    transport: str | None = None,
    control_hz: float | None = None,
    arm_control_mode: str = "posvel",
) -> ArxDCanConfig:
    """Build the public SDK config from one built-in or custom model profile."""
    data = load_cfg(config_path, model=model)
    return _config_from_loaded(
        data,
        port=_connection_channel(port, channel),
        baud=baud,
        transport=transport,
        control_hz=control_hz,
        arm_control_mode=arm_control_mode,
    )


def _actuator_config_from_sdk(config: ArxDCanConfig) -> dict:
    """Adapt an explicit SDK config without reading another YAML file."""
    joints = [
        JointCfg(
            name=joint.name,
            motor_id=joint.motor_id,
            feedback_id=joint.feedback_id,
            model=joint.model,
            kp=joint.mit_kp,
            kd=joint.mit_kd,
            vel_kp=joint.pv_vel_kp,
            vel_ki=joint.pv_vel_ki,
            pos_kp=joint.pv_pos_kp,
            pos_ki=joint.pv_pos_ki,
            vlim=joint.pv_vlim,
            direction=joint.direction,
            torque_range=joint.torque_range,
            effort_limit=joint.effort_limit,
            coupled_effort_limit=joint.coupled_effort_limit,
            coupled_motor_kd=joint.coupled_motor_kd,
            coupled_velocity_filter_s=joint.coupled_velocity_filter_s,
            coupled_torque_rise_rate=joint.coupled_torque_rise_rate,
            coupled_hold_torque_rise_rate=joint.coupled_hold_torque_rise_rate,
            coupled_torque_brake_rate=joint.coupled_torque_brake_rate,
            velocity_range=joint.velocity_range,
            lower_limit=joint.lower_limit,
            upper_limit=joint.upper_limit,
        )
        for joint in config.arm_joints
    ]
    groups: dict[str, dict[str, list[str]]] = {
        "arm": {"joints": [joint.name for joint in config.arm_joints]}
    }
    if config.gripper is not None:
        joint = config.gripper
        joints.append(
            JointCfg(
                name=joint.name,
                motor_id=joint.motor_id,
                feedback_id=joint.feedback_id,
                model=joint.model,
                kp=joint.mit_kp,
                kd=joint.mit_kd,
                vel_kp=joint.pv_vel_kp,
                vel_ki=joint.pv_vel_ki,
                pos_kp=joint.pv_pos_kp,
                pos_ki=joint.pv_pos_ki,
                vlim=joint.pv_vlim,
                direction=joint.direction,
                torque_range=joint.torque_range,
                effort_limit=joint.effort_limit,
                coupled_effort_limit=joint.coupled_effort_limit,
                coupled_motor_kd=joint.coupled_motor_kd,
                coupled_velocity_filter_s=joint.coupled_velocity_filter_s,
                coupled_torque_rise_rate=joint.coupled_torque_rise_rate,
                coupled_hold_torque_rise_rate=joint.coupled_hold_torque_rise_rate,
                coupled_torque_brake_rate=joint.coupled_torque_brake_rate,
                velocity_range=joint.velocity_range,
                lower_limit=joint.lower_limit,
                upper_limit=joint.upper_limit,
            )
        )
        groups["gripper"] = {"joints": [joint.name]}
    force = config.gripper_force_control
    return {
        "name": config.name,
        "model": config.model,
        "hardware_path": config.hardware_config_path,
        "urdf_path": config.urdf_path,
        "joint_transform_path": config.joint_transform_path,
        "end_effector_frame": config.end_effector_frame,
        "channel": config.port,
        "transport": config.transport,
        "baud": config.baud,
        "rate": config.control_hz,
        "groups": groups,
        "joints": joints,
        "gripper_mapping": {
            "closed_value": config.gripper_closed_value,
            "open_value": config.gripper_open_value,
        },
        "gripper_force_control": {
            "enabled": config.gripper_force_control_enabled,
            "close_speed": force.close_speed,
            "contact_torque": force.contact_torque,
            "overload_torque": force.overload_torque,
        },
        "safety": {
            "watchdog_enabled": config.watchdog_enabled,
            "command_timeout_s": config.command_timeout_s,
            "enable_grace_s": config.enable_grace_s,
            "watchdog_poll_s": config.watchdog_poll_s,
            "watchdog_action": config.watchdog_action,
            "safe_hold_hz": config.safe_hold_hz,
            "feedback_fault_threshold": config.feedback_fault_threshold,
            "max_cached_feedback_age_s": config.max_cached_feedback_age_s,
        },
    }


class ArxDCanArm:
    """High-level SDK for an ARX arm over dm-serial or Linux SocketCAN."""

    def __init__(
        self,
        *,
        port: str | None = None,
        channel: str | None = None,
        baud: int | None = None,
        transport: str | None = None,
        model: str | None = None,
        config_path: str | Path | None = None,
        config: ArxDCanConfig | None = None,
        control_mode: str = "posvel",
        enable_gripper: bool = False,
        gripper_gain_scale: float = 1.0,
    ) -> None:
        if config is not None and (model is not None or config_path is not None):
            raise ValueError("config cannot be combined with model or config_path")
        connection_channel = _connection_channel(port, channel)
        if config is None:
            loaded_config = load_cfg(config_path, model=model)
            self.config = _config_from_loaded(
                loaded_config,
                port=connection_channel,
                baud=baud,
                transport=transport,
                arm_control_mode=control_mode,
            )
            loaded_config = dict(loaded_config)
            loaded_config["channel"] = self.config.port
            loaded_config["transport"] = self.config.transport
            loaded_config["baud"] = self.config.baud
        else:
            self.config = replace(
                config,
                port=(
                    config.port
                    if connection_channel is None
                    else connection_channel
                ),
                baud=config.baud if baud is None else baud,
                transport=config.transport if transport is None else transport,
            )
            loaded_config = _actuator_config_from_sdk(self.config)
        self._joint_transform = (
            None
            if self.config.joint_transform_path is None
            else CoupledJointTransform.load(
                self.config.joint_transform_path,
                joint_names=self.config.joint_names,
            )
        )
        if self._joint_transform is not None:
            transformed_indices = self._joint_transform.transformed_indices
            loaded_config = dict(loaded_config)
            loaded_config["joints"] = [
                replace(
                    joint,
                    kp=0.0,
                    kd=0.0,
                    lower_limit=None,
                    upper_limit=None,
                )
                if index in transformed_indices
                else joint
                for index, joint in enumerate(loaded_config["joints"])
            ]
        gain_scale = float(gripper_gain_scale)
        if not math.isfinite(gain_scale) or gain_scale <= 0.0:
            raise ValueError("gripper_gain_scale must be finite and positive")
        self.gripper_gain_scale = gain_scale
        if self.config.gripper is not None and not math.isclose(gain_scale, 1.0):
            gripper = self.config.gripper
            force_control = self.config.gripper_force_control
            self.config = replace(
                self.config,
                gripper=replace(
                    gripper,
                    mit_kp=gripper.mit_kp * gain_scale,
                    mit_kd=gripper.mit_kd * gain_scale,
                ),
                gripper_force_control=replace(
                    force_control,
                    hold_kp=force_control.hold_kp * gain_scale,
                    hold_kd=force_control.hold_kd * gain_scale,
                ),
            )
        self._validate_safety_config()
        self.enable_gripper = enable_gripper
        active_joint_names = list(self.config.joint_names)
        if self.enable_gripper and self.config.gripper is not None:
            active_joint_names.append(self.config.gripper.name)
        self.robot = ArxDCan(
            config_data=loaded_config,
            joint_names=active_joint_names,
        )
        self._connected = False
        self._enabled = False
        self._configured = False
        self._faulted = False
        self._fault_reason: str | None = None
        self._safe_holding = False
        self._state_lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._watchdog_deadline: float | None = None
        self._feedback_error_count = 0
        self._last_joint_command: tuple[float, ...] | None = None
        self._last_mit_command: MitCommand | None = None
        self._last_gripper_command: float | None = None
        self._last_state: ArxDCanState | None = None
        self._coupled_control_stop = threading.Event()
        self._coupled_control_wakeup = threading.Event()
        self._coupled_control_thread: threading.Thread | None = None
        self._coupled_torque_saturation = CoupledTorqueSaturation(
            active=False,
            motor_names=(),
            requested_torques=(),
            limited_torques=(),
            applied_torques=(),
            saturation_scale=1.0,
            timestamp=time.monotonic(),
        )
        self._coupled_torque_telemetry = CoupledTorqueTelemetry(
            motor_names=(),
            motor_positions=(),
            motor_velocities=(),
            transformed_torques=(),
            motor_kd_gains=(),
            damping_torques=(),
            requested_torques=(),
            limited_torques=(),
            applied_torques=(),
            estimated_total_torques=(),
            saturation_scale=1.0,
            timestamp=time.monotonic(),
        )
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
        self._coupled_feedback_update_counts: dict[str, int] = {}
        self._coupled_previous_motor_tau = np.zeros(
            len(self.config.arm_joints),
            dtype=np.float64,
        )
        self._coupled_filtered_virtual_velocity = np.zeros(
            len(self.config.arm_joints),
            dtype=np.float64,
        )
        self._coupled_velocity_filter_initialized = np.zeros(
            len(self.config.arm_joints),
            dtype=bool,
        )
        self._coupled_hold_candidate_since: dict[tuple[int, int], float] = {}
        self._mode = self.config.arm_control_mode.strip().lower().replace("_", "")
        self._gripper_command_lock = threading.RLock()
        self._gripper_force_controller: GripperForceController | None = None
        if (
            self.enable_gripper
            and self.config.gripper is not None
            and self.config.gripper_force_control_enabled
        ):
            self._gripper_force_controller = GripperForceController(
                self.config.gripper_force_control,
                open_value=self.config.gripper_open_value,
                closed_value=self.config.gripper_closed_value,
                normal_kp=self.config.gripper.mit_kp,
                normal_kd=self.config.gripper.mit_kd,
            )

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.config.joint_names

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def faulted(self) -> bool:
        return self._faulted

    @property
    def fault_reason(self) -> str | None:
        return self._fault_reason

    @property
    def safe_holding(self) -> bool:
        return self._safe_holding

    @property
    def coupled_torque_saturation(self) -> CoupledTorqueSaturation:
        """Return the most recent coupled-motor torque saturation status."""
        with self._state_lock:
            return self._coupled_torque_saturation

    @property
    def coupled_torque_telemetry(self) -> CoupledTorqueTelemetry:
        """Return the latest coupled-motor feedback and torque command stages."""
        with self._state_lock:
            return self._coupled_torque_telemetry

    @property
    def coupled_control_stats(self) -> CoupledControlStats:
        """Return measured coupled-loop timing and cached-feedback health."""
        with self._state_lock:
            return self._coupled_control_stats

    @property
    def gripper_control_state(self) -> GripperControlState:
        if self._gripper_force_controller is None:
            return GripperControlState.IDLE
        return self._gripper_force_controller.state

    def connect(self) -> None:
        if self._connected:
            return
        self.robot.connect()
        with self._state_lock:
            self._connected = True
            self._configured = False
            self._enabled = False
            self._faulted = False
            self._fault_reason = None
            self._safe_holding = False
            self._feedback_error_count = 0
            self._last_joint_command = None
            self._last_mit_command = None
            self._last_gripper_command = None
            self._last_state = None

    def configure(self, mode: str | None = None) -> None:
        self._require_operational()
        try:
            self.configure_mode(mode or self._mode)
            if self.enable_gripper and self.config.gripper is not None:
                if math.isclose(self.gripper_gain_scale, 1.0):
                    gripper_mode_ok = self.robot.gripper.mode_mit()
                else:
                    gripper = self.config.gripper
                    gripper_mode_ok = self.robot.gripper.mode_mit(
                        kp=np.array([gripper.mit_kp]),
                        kd=np.array([gripper.mit_kd]),
                    )
                if not gripper_mode_ok:
                    raise RuntimeError("ARX-D-CAN gripper did not enter MIT mode")
        except Exception as exc:
            self._trip_fault(f"configuration failed: {exc}")
            raise
        self._configured = True

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
        """Resolve one logical MIT command into physical motor coordinates."""
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

        # The physical motor-side gain is deliberately passive: its velocity
        # target is zero, so it can only oppose measured motion.  Adapt the
        # gain when necessary so feed-forward plus estimated damping remains
        # inside the motor's hard effort limit.
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
                # A direction reversal must release the stored force first.
                # Braking along the old vector avoids mixing opposite A/B
                # ratios in one cycle; the new vector starts from zero.
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
            # One scale factor for the pair keeps the requested A/B torque
            # direction intact throughout ordinary rise and decay.
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
            raise _StaleCoupledFeedbackError(
                "coupled MIT feedback is stale: "
                f"{details}; limit={self.config.max_cached_feedback_age_s * 1000.0:.1f}ms"
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
            except _StaleCoupledFeedbackError as exc:
                self._trip_fault(str(exc))
            except Exception as exc:
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

    def close(self, *, disable: bool = True) -> None:
        """Stop command production and close the bus.

        The default disables every motor first.  ``disable=False`` is reserved
        for read-only clients that never configured, enabled, or commanded the
        robot and therefore must not transmit motor-control frames on exit.
        """
        self._stop_coupled_control()
        self._stop_watchdog()
        error: Exception | None = None
        if self._connected:
            try:
                if disable:
                    self.robot.disconnect()
                else:
                    self.robot.disconnect(disable=False)
            except Exception as exc:
                error = exc
        with self._state_lock:
            self._connected = False
            self._configured = False
            self._safe_holding = False
            self._watchdog_deadline = None
            if error is None:
                self._enabled = False
            else:
                # The bus is closed, but physical disable was not confirmed.
                self._enabled = True
                self._faulted = True
                self._fault_reason = f"close failed: {error}"
        if error is not None:
            raise RuntimeError(f"ARX-D-CAN close failed: {error}") from error

    def enable(
        self,
        *,
        initial_positions: Sequence[float] | None = None,
        initial_velocities: Sequence[float] | None = None,
        initial_torques: Sequence[float] | None = None,
        mit_kp: float | Sequence[float] | None = None,
        mit_kd: float | Sequence[float] | None = None,
    ) -> None:
        self._require_operational()
        if not self._configured:
            raise RuntimeError("ARX-D-CAN arm must be configured before enable")
        if initial_positions is not None and self._mode != "mit":
            raise ValueError("initial position seeding is only supported in MIT mode")
        self._reset_coupled_motor_torque_state()
        joint_count = len(self.config.arm_joints)
        initial_position_vector = None
        initial_velocity_vector = None
        initial_torque_vector = None
        kp_vector = None
        kd_vector = None
        if initial_positions is not None:
            vectors = {
                "initial_positions": initial_positions,
                "initial_velocities": (
                    [0.0] * joint_count
                    if initial_velocities is None
                    else initial_velocities
                ),
                "initial_torques": (
                    [0.0] * joint_count
                    if initial_torques is None
                    else initial_torques
                ),
            }
            parsed = {}
            for name, values in vectors.items():
                if len(values) != joint_count:
                    raise ValueError(f"{name} must contain {joint_count} values")
                array = np.asarray(values, dtype=np.float64)
                if not np.all(np.isfinite(array)):
                    raise ValueError(f"{name} must be finite")
                parsed[name] = array
            kp_vector = _mit_gain_vector(mit_kp, joint_count=joint_count, name="Kp")
            kd_vector = _mit_gain_vector(mit_kd, joint_count=joint_count, name="Kd")
            kp_vector = self._resolved_mit_gains(kp_vector, gain="kp")
            kd_vector = self._resolved_mit_gains(kd_vector, gain="kd")
            initial_command = self._make_mit_command(
                parsed["initial_positions"],
                parsed["initial_velocities"],
                kp_vector,
                kd_vector,
                parsed["initial_torques"],
            )
            if self._joint_transform is None:
                initial_position_vector = np.asarray(initial_command.positions)
                initial_velocity_vector = np.asarray(initial_command.velocities)
                initial_torque_vector = np.asarray(initial_command.feedforward_torques)
            else:
                # Enabling must never seed coupled motors with virtual gains.
                # Before the inner loop has fresh enabled feedback, apply only
                # the transformed feed-forward torque; do not infer a PD error
                # from the fitted forward/inverse model's residual.
                initial_position_vector = (
                    self._joint_transform.virtual_positions_to_motor(
                        initial_command.positions
                    )
                )
                initial_velocity_vector = (
                    self._joint_transform.virtual_velocities_to_motor(
                        initial_command.positions,
                        initial_command.velocities,
                    )
                )
                initial_torque_vector = (
                    self._joint_transform.virtual_torques_to_motor(
                        initial_command.positions,
                        initial_command.feedforward_torques,
                    )
                )
                transformed_indices = sorted(
                    self._joint_transform.transformed_indices
                )
                initial_velocity_vector[transformed_indices] = 0.0
                kp_vector[transformed_indices] = 0.0
                kd_vector[transformed_indices] = np.asarray(
                    [
                        self.config.arm_joints[index].coupled_motor_kd
                        for index in transformed_indices
                    ],
                    dtype=np.float64,
                )
                initial_torque_vector = self._limit_coupled_motor_torques(
                    initial_torque_vector
                )
        try:
            if initial_position_vector is None:
                self.robot.arm.enable()
            else:
                self.robot.arm.enable(
                    mit_position=initial_position_vector,
                    mit_velocity=initial_velocity_vector,
                    mit_kp=kp_vector,
                    mit_kd=kd_vector,
                    mit_tau=initial_torque_vector,
                )
            with self._gripper_command_lock:
                if self.enable_gripper and self.config.gripper is not None:
                    self.robot.gripper.enable()
                    if self._gripper_force_controller is not None:
                        self._gripper_force_controller.reset()
        except Exception as exc:
            self._trip_fault(f"enable failed: {exc}")
            raise
        with self._state_lock:
            self._enabled = True
            if initial_positions is not None:
                self._last_joint_command = tuple(float(value) for value in initial_positions)
                self._last_mit_command = initial_command
            self._watchdog_deadline = time.monotonic() + max(
                self.config.enable_grace_s,
                self.config.command_timeout_s,
            )
        self._start_watchdog()
        self._start_coupled_control()

    def disable(self) -> None:
        self._require_connected()
        self._stop_coupled_control()
        self._stop_watchdog()
        try:
            self.robot.estop()
        except Exception as exc:
            with self._gripper_command_lock:
                if self._gripper_force_controller is not None:
                    self._gripper_force_controller.reset()
            with self._state_lock:
                # Physical disable is unconfirmed. Keep a conservative
                # software state instead of pretending the motors are safe.
                self._enabled = True
                self._faulted = True
                self._fault_reason = f"disable failed: {exc}"
                self._safe_holding = False
                self._watchdog_deadline = None
            raise
        with self._gripper_command_lock:
            if self._gripper_force_controller is not None:
                self._gripper_force_controller.reset()
        with self._state_lock:
            self._enabled = False
            self._safe_holding = False
            self._watchdog_deadline = None
            self._last_joint_command = None
            self._last_mit_command = None
        self._reset_coupled_motor_torque_state()

    def clear_fault(self) -> None:
        """Clear the SDK fault latch after healthy feedback is available."""
        self._require_connected()
        was_safe_holding = self._safe_holding
        self._stop_watchdog()
        if not was_safe_holding:
            try:
                self.robot.estop()
            except Exception:
                pass
        self.robot.get_state(
            request_feedback=True,
            require_complete=True,
            joint_names=self._active_joint_names(),
        )
        with self._state_lock:
            self._faulted = False
            self._fault_reason = None
            self._safe_holding = False
            self._enabled = was_safe_holding
            self._configured = False
            self._feedback_error_count = 0
            self._watchdog_deadline = None

    def clear_motor_faults(self) -> tuple[str, ...]:
        """Clear all active motor faults and leave every motor disabled."""
        self._require_connected()
        self._stop_watchdog()
        try:
            completed = self.robot.clear_errors(
                joint_names=self._active_joint_names(),
            )
        except Exception as exc:
            with self._state_lock:
                self._enabled = False
                self._configured = False
                self._faulted = True
                self._fault_reason = f"motor fault clear failed: {exc}"
                self._safe_holding = False
                self._feedback_error_count = 0
                self._watchdog_deadline = None
            raise

        with self._state_lock:
            self._enabled = False
            self._configured = False
            self._faulted = False
            self._fault_reason = None
            self._safe_holding = False
            self._feedback_error_count = 0
            self._watchdog_deadline = None
        return completed

    def recover(self) -> None:
        """Recover atomically from a latched fault and resume command handling."""
        self.clear_fault()
        try:
            self.configure()
            self.enable()
        except Exception:
            if not self._faulted:
                self._trip_fault("fault recovery failed")
            raise

    def configure_mode(self, mode: str = "posvel") -> None:
        self._require_operational()
        normalized = mode.strip().lower().replace("_", "")
        if normalized in ("posvel", "pv"):
            self._stop_coupled_control()
            with self._io_lock:
                if not self.robot.arm.mode_pos_vel():
                    raise RuntimeError("ARX-D-CAN arm did not enter POS_VEL mode")
            self._mode = "posvel"
            return
        if normalized == "mit":
            with self._io_lock:
                if not self.robot.arm.mode_mit():
                    raise RuntimeError("ARX-D-CAN arm did not enter MIT mode")
            self._mode = "mit"
            return
        raise ValueError("mode must be 'posvel' or 'mit'")

    def read_state(self, *, request_feedback: bool = True) -> ArxDCanState:
        """Read state while serializing access with normal command traffic."""
        return self._read_state(
            request_feedback=request_feedback,
            serialize_io=True,
        )

    def refresh_feedback_background(self) -> ArxDCanState:
        """Request complete feedback without holding the SDK command lock.

        This method is intended for a dedicated low-rate monitor thread.  The
        motor-drive-layer serial transport and paced transmit bus serialize
        their own I/O, so a feedback timeout here does not hold up the primary
        command loop through ``_io_lock``.
        """
        return self._read_state(
            request_feedback=True,
            serialize_io=False,
        )

    def _read_state(
        self,
        *,
        request_feedback: bool,
        serialize_io: bool,
    ) -> ArxDCanState:
        self._require_connected()
        active_joint_names = self._active_joint_names()

        def read_raw_state():
            pos, vel, tau = self.robot.get_state(
                request_feedback=request_feedback,
                require_complete=request_feedback,
                joint_names=active_joint_names,
            )
            status_codes = self.robot.get_status_codes(
                joint_names=active_joint_names,
            )
            return pos, vel, tau, status_codes

        try:
            if serialize_io:
                with self._io_lock:
                    pos, vel, tau, status_codes = read_raw_state()
            else:
                pos, vel, tau, status_codes = read_raw_state()
        except Exception as exc:
            if request_feedback:
                with self._state_lock:
                    self._feedback_error_count += 1
                    feedback_error_count = self._feedback_error_count
                if self._enabled and feedback_error_count >= max(
                    1, self.config.feedback_fault_threshold
                ):
                    self._begin_safe_hold(
                        f"feedback failed {feedback_error_count} consecutive times: "
                        f"{exc}"
                    )
            elif self._enabled:
                self._begin_safe_hold(
                    f"cached feedback became invalid: {exc}"
                )
            with self._state_lock:
                last_state = self._last_state
            if self._enabled and last_state is not None:
                return last_state
            raise
        if request_feedback:
            with self._state_lock:
                self._feedback_error_count = 0
        disabled_motors = [
            name for name, status in status_codes.items() if status == 0
        ]
        if self._enabled and disabled_motors:
            reason = (
                "motors unexpectedly disabled after feedback recovery: "
                + ", ".join(disabled_motors)
            )
            if not self._safe_holding:
                self._begin_safe_hold(reason)
            else:
                with self._state_lock:
                    self._fault_reason = (
                        f"{reason}; holding last successful command"
                    )
        elif self._safe_holding and request_feedback:
            self._resume_from_safe_hold()
        arm_count = len(self.config.arm_joints)
        arm_pos = pos[:arm_count]
        arm_vel = vel[:arm_count]
        arm_tau = tau[:arm_count]
        arm_pos, arm_vel, arm_tau = self._transform_feedback_vectors(
            arm_pos,
            arm_vel,
            arm_tau,
        )
        gripper_state = None
        if self.config.gripper is not None and len(pos) > arm_count:
            gripper_state = MotorState(
                name=self.config.gripper.name,
                motor_id=self.config.gripper.motor_id,
                feedback_id=self.config.gripper.feedback_id,
                position=float(pos[arm_count]),
                velocity=float(vel[arm_count]),
                torque=float(tau[arm_count]),
            )
        state = ArxDCanState(
            arm=JointState(
                names=self.config.joint_names,
                positions=tuple(float(value) for value in arm_pos),
                velocities=tuple(float(value) for value in arm_vel),
                torques=tuple(float(value) for value in arm_tau),
            ),
            gripper=gripper_state,
        )
        with self._state_lock:
            self._last_state = state
        return state

    def scan_ids(
        self,
        *,
        start_id: int = 1,
        end_id: int = 16,
        model: str = "4340P",
        timeout_ms: int = 30,
        feedback_base: str = "0x10",
    ) -> list[int]:
        command = build_scan_command(
            python_executable=sys.executable,
            port=self.config.port,
            baud=self.config.baud,
            transport=self.config.transport,
            model=model,
            start_id=start_id,
            end_id=end_id,
            feedback_base=feedback_base,
            timeout_ms=timeout_ms,
        )
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        if result.returncode != 0:
            raise RuntimeError(
                result.stderr.strip()
                or result.stdout.strip()
                or "motor-drive-layer scan failed"
            )
        return parse_scan_ids(result.stdout, model=model)

    def send_joint_positions(
        self,
        positions: Sequence[float],
        *,
        velocities: Sequence[float] | None = None,
        velocity_limits: Sequence[float] | None = None,
        torques: Sequence[float] | None = None,
        mit_kp: float | Sequence[float] | None = None,
        mit_kd: float | Sequence[float] | None = None,
        mode: str | None = None,
        require_enabled: bool = True,
    ) -> None:
        """Send one arm command.

        In MIT mode, ``mit_kp`` and ``mit_kd`` override the YAML gains for
        this packet only. Omitting either argument keeps that gain at its
        configured YAML value.
        """
        self._require_connected()
        if self._safe_holding:
            return
        self._require_operational()
        if require_enabled and not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        joint_count = len(self.config.arm_joints)
        if len(positions) != joint_count:
            raise ValueError(
                f"expected {joint_count} joint positions, got {len(positions)}"
            )
        if any(not math.isfinite(float(value)) for value in positions):
            raise ValueError("joint positions must be finite")
        target = {
            joint.name: float(value)
            for joint, value in zip(self.config.arm_joints, positions)
        }
        velocity_target: np.ndarray | None = None
        if velocities is not None:
            if len(velocities) != joint_count:
                raise ValueError(
                    f"expected {joint_count} joint velocities, "
                    f"got {len(velocities)}"
                )
            if any(not math.isfinite(float(value)) for value in velocities):
                raise ValueError("joint velocities must be finite")
            velocity_target = np.asarray(velocities, dtype=np.float64)
        velocity_limit_target: np.ndarray | None = None
        if velocity_limits is not None:
            if len(velocity_limits) != joint_count:
                raise ValueError(
                    f"expected {joint_count} joint velocity limits, "
                    f"got {len(velocity_limits)}"
                )
            if any(
                not math.isfinite(float(value)) or float(value) <= 0.0
                for value in velocity_limits
            ):
                raise ValueError("joint velocity limits must be finite and positive")
            velocity_limit_target = np.asarray(velocity_limits, dtype=np.float64)
        torque_target: np.ndarray | None = None
        if torques is not None:
            if len(torques) != joint_count:
                raise ValueError(
                    f"expected {joint_count} joint torques, "
                    f"got {len(torques)}"
                )
            if any(not math.isfinite(float(value)) for value in torques):
                raise ValueError("joint torques must be finite")
            torque_target = np.asarray(torques, dtype=np.float64)
        mit_kp_target = _mit_gain_vector(
            mit_kp,
            joint_count=joint_count,
            name="Kp",
        )
        mit_kd_target = _mit_gain_vector(
            mit_kd,
            joint_count=joint_count,
            name="Kd",
        )
        active_mode = (mode or self._mode).strip().lower().replace("_", "")
        if active_mode in ("posvel", "pv"):
            if velocity_target is not None:
                raise ValueError("target velocities are only supported in MIT mode")
            if torque_target is not None:
                raise ValueError("torques are only supported in MIT mode")
            if mit_kp_target is not None or mit_kd_target is not None:
                raise ValueError("MIT Kp/Kd are only supported in MIT mode")
        if active_mode == "mit" and velocity_limit_target is not None:
            raise ValueError("velocity limits are only supported in PV mode")
        logical_position_target = np.array(
            [target[joint.name] for joint in self.config.arm_joints],
            dtype=np.float64,
        )
        try:
            if active_mode in ("posvel", "pv"):
                if self._mode != "posvel":
                    self.configure_mode("posvel")
                motor_position_target, _, _, velocity_limit_target = (
                    self._transform_command_vectors(
                        logical_position_target,
                        velocity_limits=velocity_limit_target,
                    )
                )
                with self._io_lock:
                    self.robot.arm.send_pos_vel(
                        motor_position_target,
                        vlim=velocity_limit_target,
                        strict=True,
                    )
                self._record_successful_command(
                    joint_positions=tuple(
                        target[joint.name] for joint in self.config.arm_joints
                    )
                )
                return
            if active_mode == "mit":
                if self._mode != "mit":
                    self.configure_mode("mit")
                command = self._make_mit_command(
                    logical_position_target,
                    (
                        np.zeros(joint_count)
                        if velocity_target is None
                        else velocity_target
                    ),
                    self._resolved_mit_gains(mit_kp_target, gain="kp"),
                    self._resolved_mit_gains(mit_kd_target, gain="kd"),
                    np.zeros(joint_count) if torque_target is None else torque_target,
                )
                thread = self._coupled_control_thread
                background_running = (
                    self._joint_transform is not None
                    and thread is not None
                    and thread.is_alive()
                )
                if self._enabled and not background_running:
                    self._send_mit_command(command, strict=True)
                self._record_successful_command(
                    joint_positions=command.positions,
                    mit_command=command,
                )
                self._start_coupled_control()
                self._coupled_control_wakeup.set()
                return
        except Exception as exc:
            if isinstance(exc, _StaleCoupledFeedbackError):
                self._trip_fault(str(exc))
                raise
            holding = self._begin_safe_hold(f"joint command failed: {exc}")
            with self._state_lock:
                has_hold_target = self._last_joint_command is not None
            if holding and has_hold_target:
                return
            raise
        raise ValueError("mode must be 'posvel' or 'mit'")

    def hold_current_position(self) -> ArxDCanState:
        state = self.read_state(request_feedback=True)
        self.send_joint_positions(state.arm.positions)
        return state

    def set_zero(
        self,
        *,
        joint_names: Sequence[str] | None = None,
        verify_tolerance: float = 0.02,
        verify_velocity: float = 0.05,
        verify_samples: int = 3,
    ) -> tuple[str, ...]:
        """Write current positions as zero and verify consecutive fresh states."""
        self._require_operational()
        if self._enabled:
            raise RuntimeError("disable the arm before writing motor zero positions")
        if self._joint_transform is not None:
            coupled_names = {
                self.config.arm_joints[index].name
                for index in self._joint_transform.transformed_indices
            }
            requested_names = (
                set(self.config.joint_names)
                if joint_names is None
                else {str(name) for name in joint_names}
            )
            protected = sorted(coupled_names.intersection(requested_names))
            if protected:
                raise RuntimeError(
                    "model-defined coupled joints cannot be motor-zeroed: "
                    + ", ".join(protected)
                )
        return self.robot.set_zero(
            joint_names=list(joint_names) if joint_names is not None else None,
            verify_tolerance=verify_tolerance,
            verify_velocity=verify_velocity,
            verify_samples=verify_samples,
        )

    def set_gripper(
        self,
        value: float,
        *,
        input_min: float = 0.0,
        input_max: float = 1000.0,
        require_enabled: bool = True,
    ) -> None:
        self._require_connected()
        if self._safe_holding:
            return
        self._require_operational()
        if self.config.gripper is None:
            return
        if require_enabled and not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        if input_max == input_min:
            raise ValueError("input_max and input_min must differ")
        ratio = (float(value) - input_min) / (input_max - input_min)
        ratio = max(0.0, min(1.0, ratio))
        motor_value = (
            self.config.gripper_closed_value
            + (self.config.gripper_open_value - self.config.gripper_closed_value) * ratio
        )
        self.set_gripper_motor_value(motor_value, require_enabled=require_enabled)

    def set_gripper_motor_value(
        self,
        value: float,
        *,
        require_enabled: bool = True,
    ) -> None:
        self._require_connected()
        if self._safe_holding:
            return
        self._require_operational()
        if self.config.gripper is None:
            return
        if not self.enable_gripper:
            raise RuntimeError("ARX-D-CAN gripper is disabled; create ArxDCanArm(enable_gripper=True)")
        target = float(value)
        if not math.isfinite(target):
            raise ValueError("gripper motor value must be finite")
        lower = min(
            self.config.gripper_closed_value,
            self.config.gripper_open_value,
        )
        upper = max(
            self.config.gripper_closed_value,
            self.config.gripper_open_value,
        )
        target = min(max(target, lower), upper)
        with self._gripper_command_lock:
            if require_enabled and not self._enabled:
                raise RuntimeError("ARX-D-CAN arm is not enabled")
            if self._gripper_force_controller is not None:
                with self._io_lock:
                    position, _, torque = self.robot.gripper.read_state(
                        request_feedback=True
                    )
                if len(position) != 1 or len(torque) != 1:
                    raise RuntimeError("gripper feedback must contain exactly one motor")
                command = self._gripper_force_controller.update(
                    requested_position=target,
                    actual_position=float(position[0]),
                    actual_torque=float(torque[0]),
                    now=time.monotonic(),
                )
                try:
                    with self._io_lock:
                        self.robot.gripper.send_mit(
                            np.array([command.position]),
                            kp=np.array([command.kp]),
                            kd=np.array([command.kd]),
                            strict=True,
                        )
                except Exception:
                    self._gripper_force_controller.reset()
                    holding = self._begin_safe_hold("gripper command failed")
                    with self._state_lock:
                        has_hold_target = self._last_gripper_command is not None
                    if holding and has_hold_target:
                        return
                    raise
                self._record_successful_command(
                    gripper_position=float(command.position)
                )
                return
            try:
                with self._io_lock:
                    self.robot.gripper.send_mit(
                        np.array([target]),
                        strict=True,
                    )
            except Exception as exc:
                holding = self._begin_safe_hold(f"gripper command failed: {exc}")
                with self._state_lock:
                    has_hold_target = self._last_gripper_command is not None
                if holding and has_hold_target:
                    return
                raise
            self._record_successful_command(gripper_position=target)

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("ARX-D-CAN arm is not connected")

    def _active_joint_names(self) -> list[str]:
        names = list(self.config.joint_names)
        if self.enable_gripper and self.config.gripper is not None:
            names.append(self.config.gripper.name)
        return names

    def _validate_safety_config(self) -> None:
        if (
            not math.isfinite(self.config.control_hz)
            or self.config.control_hz <= 0.0
        ):
            raise ValueError("control_hz must be finite and positive")
        if self.config.watchdog_enabled and self.config.command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be positive")
        if self.config.enable_grace_s < 0.0:
            raise ValueError("enable_grace_s must not be negative")
        if self.config.watchdog_poll_s <= 0.0:
            raise ValueError("watchdog_poll_s must be positive")
        action = self.config.watchdog_action.strip().lower()
        if action not in {"safe_hold", "disable"}:
            raise ValueError("watchdog_action must be 'safe_hold' or 'disable'")
        if self.config.safe_hold_hz <= 0.0:
            raise ValueError("safe_hold_hz must be positive")
        if self.config.feedback_fault_threshold < 1:
            raise ValueError("feedback_fault_threshold must be at least 1")
        if (
            not math.isfinite(self.config.max_cached_feedback_age_s)
            or self.config.max_cached_feedback_age_s <= 0.0
        ):
            raise ValueError("max_cached_feedback_age_s must be finite and positive")
        for joint in self.config.arm_joints:
            if (
                not math.isfinite(joint.coupled_motor_kd)
                or joint.coupled_motor_kd < 0.0
            ):
                raise ValueError(
                    f"{joint.name}.coupled_motor_kd must be finite and non-negative"
                )
            if (
                not math.isfinite(joint.coupled_velocity_filter_s)
                or joint.coupled_velocity_filter_s < 0.0
            ):
                raise ValueError(
                    f"{joint.name}.coupled_velocity_filter_s must be finite "
                    "and non-negative"
                )
            for name, value in (
                ("coupled_effort_limit", joint.coupled_effort_limit),
                ("coupled_torque_rise_rate", joint.coupled_torque_rise_rate),
                (
                    "coupled_hold_torque_rise_rate",
                    joint.coupled_hold_torque_rise_rate,
                ),
                ("coupled_torque_brake_rate", joint.coupled_torque_brake_rate),
            ):
                if value is not None and (
                    not math.isfinite(value) or value <= 0.0
                ):
                    raise ValueError(
                        f"{joint.name}.{name} must be finite and positive"
                    )

    def _require_operational(self) -> None:
        self._require_connected()
        if self._faulted:
            raise RuntimeError(
                f"ARX-D-CAN arm is faulted: {self._fault_reason}; "
                "call clear_fault(), configure(), and enable() to recover"
            )

    def _record_successful_command(
        self,
        *,
        joint_positions: tuple[float, ...] | None = None,
        mit_command: MitCommand | None = None,
        gripper_position: float | None = None,
    ) -> None:
        with self._state_lock:
            if joint_positions is not None:
                self._last_joint_command = joint_positions
            if mit_command is not None:
                self._last_mit_command = mit_command
            if gripper_position is not None:
                self._last_gripper_command = gripper_position
            if self._enabled:
                self._watchdog_deadline = (
                    time.monotonic() + self.config.command_timeout_s
                )

    def _start_watchdog(self) -> None:
        with self._state_lock:
            safe_holding = self._safe_holding
        if not self.config.watchdog_enabled and not safe_holding:
            return
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            return
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="arx-d-can-command-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()

    def _stop_watchdog(self) -> None:
        self._watchdog_stop.set()
        thread = self._watchdog_thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=1.0)
        if thread is not threading.current_thread():
            self._watchdog_thread = None

    def _watchdog_loop(self) -> None:
        with self._state_lock:
            safe_holding = self._safe_holding
        if safe_holding:
            self._safe_hold_loop()
            return

        poll_s = max(0.005, self.config.watchdog_poll_s)
        while not self._watchdog_stop.wait(poll_s):
            with self._state_lock:
                deadline = self._watchdog_deadline
                enabled = self._enabled
            if enabled and deadline is not None and time.monotonic() > deadline:
                reason = (
                    f"command watchdog timed out after "
                    f"{self.config.command_timeout_s:.3f}s"
                )
                if self.config.watchdog_action.strip().lower() == "safe_hold":
                    if self._begin_safe_hold(
                        reason,
                        expected_deadline=deadline,
                    ):
                        self._safe_hold_loop()
                else:
                    self._trip_fault(reason)
                return

    def _begin_safe_hold(
        self,
        reason: str,
        *,
        expected_deadline: float | None = None,
    ) -> bool:
        called_from_watchdog = threading.current_thread() is self._watchdog_thread
        if not called_from_watchdog:
            self._stop_watchdog()
        with self._state_lock:
            if (
                not self._enabled
                or (
                    expected_deadline is not None
                    and (
                        self._watchdog_deadline != expected_deadline
                        or time.monotonic() <= expected_deadline
                    )
                )
            ):
                return False
            self._faulted = True
            self._safe_holding = True
            self._fault_reason = f"{reason}; holding last successful command"
            self._watchdog_deadline = None
        if not called_from_watchdog:
            self._start_watchdog()
        return True

    def _safe_hold_loop(self) -> None:
        period = 1.0 / self.config.safe_hold_hz
        while not self._watchdog_stop.is_set():
            with self._state_lock:
                if not self._safe_holding or not self._enabled:
                    return
                joint_target = self._last_joint_command
                mit_command = self._last_mit_command
                gripper_target = self._last_gripper_command
            try:
                if joint_target is not None:
                    if self._mode == "mit" and mit_command is not None:
                        self._send_mit_command(mit_command, strict=False)
                    else:
                        target, _, _, _ = self._transform_command_vectors(
                            joint_target
                        )
                        with self._io_lock:
                            self.robot.arm.send_pos_vel(target, strict=False)
                with self._io_lock:
                    if gripper_target is not None:
                        kp = self.config.gripper_force_control.hold_kp
                        kd = self.config.gripper_force_control.hold_kd
                        self.robot.gripper.send_mit(
                            np.array([gripper_target]),
                            kp=np.array([kp]),
                            kd=np.array([kd]),
                            strict=False,
                        )
            except Exception as exc:
                if isinstance(exc, _StaleCoupledFeedbackError):
                    self._trip_fault(str(exc))
                    return
                with self._state_lock:
                    if self._safe_holding:
                        marker = "; hold retry failed:"
                        if marker not in (self._fault_reason or ""):
                            self._fault_reason = (
                                f"{self._fault_reason}{marker} {exc}"
                            )
            self._watchdog_stop.wait(period)

    def _resume_from_safe_hold(self) -> None:
        self._stop_watchdog()
        with self._state_lock:
            if not self._safe_holding:
                return
            self._faulted = False
            self._safe_holding = False
            self._fault_reason = None
            self._feedback_error_count = 0
            self._watchdog_deadline = (
                time.monotonic() + self.config.command_timeout_s
                if self._enabled
                else None
            )
        self._start_watchdog()

    def _trip_fault(self, reason: str) -> None:
        with self._state_lock:
            if self._faulted and not self._safe_holding:
                return
            self._faulted = True
            self._safe_holding = False
            self._fault_reason = reason
            self._enabled = False
            self._watchdog_deadline = None
            self._watchdog_stop.set()
        try:
            self.robot.estop()
        except Exception as exc:
            with self._state_lock:
                self._fault_reason = f"{reason}; emergency disable error: {exc}"
                self._enabled = True
