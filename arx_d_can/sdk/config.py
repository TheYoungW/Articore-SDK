"""高层 SDK 的配置模型与适配器。"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np

from ..actuator import JointCfg, load_cfg
from .gripper_force_control import GripperForceControlConfig


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
    gripper_control_hz: float = 100.0
    gripper_fault_action: str = "hold"
    watchdog_enabled: bool = True
    command_timeout_s: float = 0.25
    enable_grace_s: float = 2.0
    watchdog_poll_s: float = 0.02
    watchdog_action: str = "safe_hold"
    safe_hold_hz: float = 100.0
    safe_hold_pv_velocity_limit: float = 0.2
    safe_hold_mit_kp: float = 5.0
    safe_hold_mit_kd: float = 1.0
    safe_hold_failure_threshold: int = 1
    feedback_check_hz: float = 100.0
    feedback_fault_threshold: int = 3
    max_cached_feedback_age_s: float = 0.02
    motor_communication_timeout_ms: int = 500
    name: str = "ARX-D-CAN"
    model: str = "custom"
    hardware_config_path: str | None = None
    urdf_path: str | None = None
    joint_transform_path: str | None = None
    end_effector_frame: str = "gripper_end"

    @property
    def channel(self) -> str:
        """返回通信通道；为兼容现有 API，内部仍保留 ``port`` 字段。"""
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
        joint.name: _joint_from_yaml(joint) for joint in data.get("joints", [])
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
        gripper_control_hz=float(force_control.get("control_hz", 100.0)),
        gripper_fault_action=str(safety.get("gripper_fault_action", "hold")),
        watchdog_enabled=_config_bool(
            safety.get("watchdog_enabled", True),
            name="safety.watchdog_enabled",
        ),
        command_timeout_s=float(safety.get("command_timeout_s", 0.25)),
        enable_grace_s=float(safety.get("enable_grace_s", 2.0)),
        watchdog_poll_s=float(safety.get("watchdog_poll_s", 0.02)),
        watchdog_action=str(safety.get("watchdog_action", "safe_hold")),
        safe_hold_hz=float(safety.get("safe_hold_hz", 100.0)),
        safe_hold_pv_velocity_limit=float(
            safety.get("safe_hold_pv_velocity_limit", 0.2)
        ),
        safe_hold_mit_kp=float(safety.get("safe_hold_mit_kp", 5.0)),
        safe_hold_mit_kd=float(safety.get("safe_hold_mit_kd", 1.0)),
        safe_hold_failure_threshold=int(
            safety.get("safe_hold_failure_threshold", 1)
        ),
        feedback_check_hz=float(safety.get("feedback_check_hz", 100.0)),
        feedback_fault_threshold=int(safety.get("feedback_fault_threshold", 3)),
        max_cached_feedback_age_s=float(
            safety.get("max_cached_feedback_age_s", 0.02)
        ),
        motor_communication_timeout_ms=int(
            safety.get("motor_communication_timeout_ms", 500)
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
    """根据内置或自定义机型配置构建公开的 SDK 配置。"""
    data = load_cfg(config_path, model=model)
    return _config_from_loaded(
        data,
        port=_connection_channel(port, channel),
        baud=baud,
        transport=transport,
        control_hz=control_hz,
        arm_control_mode=arm_control_mode,
    )


def _actuator_joint_from_sdk(joint: JointMotorConfig) -> JointCfg:
    return JointCfg(
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


def _actuator_config_from_sdk(config: ArxDCanConfig) -> dict:
    """直接转换显式 SDK 配置，不再读取其他 YAML 文件。"""
    joints = [_actuator_joint_from_sdk(joint) for joint in config.arm_joints]
    groups: dict[str, dict[str, list[str]]] = {
        "arm": {"joints": [joint.name for joint in config.arm_joints]}
    }
    if config.gripper is not None:
        joints.append(_actuator_joint_from_sdk(config.gripper))
        groups["gripper"] = {"joints": [config.gripper.name]}
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
            "control_hz": config.gripper_control_hz,
            "close_speed": force.close_speed,
            "contact_torque": force.contact_torque,
            "overload_torque": force.overload_torque,
            "motion_window_s": force.motion_window_s,
            "stall_movement": force.stall_movement,
            "min_position_error": force.min_position_error,
            "contact_hold_s": force.contact_hold_s,
            "overload_hold_s": force.overload_hold_s,
            "hold_offset": force.hold_offset,
            "retreat_distance": force.retreat_distance,
            "max_step_interval_s": force.max_step_interval_s,
            "overload_retreat_interval_s": force.overload_retreat_interval_s,
            "hold_kp": force.hold_kp,
            "hold_kd": force.hold_kd,
        },
        "safety": {
            "watchdog_enabled": config.watchdog_enabled,
            "command_timeout_s": config.command_timeout_s,
            "enable_grace_s": config.enable_grace_s,
            "watchdog_poll_s": config.watchdog_poll_s,
            "watchdog_action": config.watchdog_action,
            "safe_hold_hz": config.safe_hold_hz,
            "safe_hold_pv_velocity_limit": config.safe_hold_pv_velocity_limit,
            "safe_hold_mit_kp": config.safe_hold_mit_kp,
            "safe_hold_mit_kd": config.safe_hold_mit_kd,
            "safe_hold_failure_threshold": config.safe_hold_failure_threshold,
            "feedback_check_hz": config.feedback_check_hz,
            "feedback_fault_threshold": config.feedback_fault_threshold,
            "max_cached_feedback_age_s": config.max_cached_feedback_age_s,
            "motor_communication_timeout_ms": config.motor_communication_timeout_ms,
            "gripper_fault_action": config.gripper_fault_action,
        },
    }


__all__ = ["ArxDCanConfig", "JointMotorConfig", "default_config"]
