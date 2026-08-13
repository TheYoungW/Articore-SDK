"""高层 SDK 的配置模型与适配器。"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Sequence

import numpy as np

from ..actuator import JointCfg, load_cfg
# 1～10 档，字段依次为接触/过载力矩、运动 Kp/Kd、保持 Kp/Kd。
_DEFAULT_GRIPPER_FORCE_PROFILES = (
    (0.5, 1.0, 3.0, 0.3, 1.0, 0.2),
    (0.575, 1.125, 3.25, 0.35, 1.25, 0.275),
    (0.65, 1.25, 3.5, 0.4, 1.5, 0.35),
    (0.725, 1.375, 3.75, 0.45, 1.75, 0.425),
    (0.8, 1.5, 4.0, 0.5, 2.0, 0.5),
    (0.88, 1.6, 4.4, 0.56, 2.2, 0.54),
    (0.96, 1.7, 4.8, 0.62, 2.4, 0.58),
    (1.04, 1.8, 5.2, 0.68, 2.6, 0.62),
    (1.12, 1.9, 5.6, 0.74, 2.8, 0.66),
    (1.2, 2.0, 6.0, 0.8, 3.0, 0.7),
)
_GRIPPER_PROFILE_FIELDS = (
    "contact_torque",
    "overload_torque",
    "moving_kp",
    "moving_kd",
    "hold_kp",
    "hold_kd",
)
_MIT_GAIN_MAX = {"Kp": 500.0, "Kd": 5.0}


@dataclass(slots=True, frozen=True)
class GripperProtectionConfig:
    """传递给 motor 原生运行时的夹爪保护参数。"""

    control_hz: float = 500.0
    close_speed: float = 1.0
    contact_torque: float = 0.8
    overload_torque: float = 1.5
    motion_window_s: float = 0.2
    stall_movement: float = 0.01
    min_position_error: float = 0.05
    contact_hold_s: float = 0.2
    overload_hold_s: float = 0.05
    hold_offset: float = 0.08
    retreat_distance: float = 0.15
    max_step_interval_s: float = 0.05
    overload_retreat_interval_s: float = 0.1
    hold_kp: float = 2.0
    hold_kd: float = 0.5
    force_profiles: tuple[tuple[float, float, float, float, float, float], ...] = (
        _DEFAULT_GRIPPER_FORCE_PROFILES
    )

    def __post_init__(self) -> None:
        values = tuple(
            float(getattr(self, name))
            for name in self.__dataclass_fields__
            if name != "force_profiles"
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("gripper protection values must be finite and non-negative")
        required_positive = {
            "control_hz": self.control_hz,
            "close_speed": self.close_speed,
            "contact_torque": self.contact_torque,
            "motion_window_s": self.motion_window_s,
            "hold_offset": self.hold_offset,
            "retreat_distance": self.retreat_distance,
            "max_step_interval_s": self.max_step_interval_s,
            "overload_retreat_interval_s": self.overload_retreat_interval_s,
            "hold_kp": self.hold_kp,
        }
        for name, value in required_positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.overload_torque <= self.contact_torque:
            raise ValueError("overload_torque must be greater than contact_torque")
        if len(self.force_profiles) != 10 or any(
            len(profile) != 6
            or not all(math.isfinite(value) and value >= 0 for value in profile)
            or profile[1] <= profile[0]
            or profile[2] <= 0
            or profile[4] <= 0
            for profile in self.force_profiles
        ):
            raise ValueError("gripper force_profiles must contain ten valid profiles")


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
    velocity_range: float | None = None
    lower_limit: float | None = None
    upper_limit: float | None = None


@dataclass(slots=True, frozen=True)
class ArxDCanConfig:
    port: str = "/dev/ttyACM0"
    transport: str = "auto"
    baud: int = 1_000_000
    control_hz: float = 500.0
    arm_control_mode: str = "posvel"
    arm_joints: tuple[JointMotorConfig, ...] = ()
    gripper: JointMotorConfig | None = None
    gripper_open_value: float = 2.64
    gripper_closed_value: float = 0.0
    gripper_protection: GripperProtectionConfig = field(
        default_factory=GripperProtectionConfig
    )
    gripper_fault_action: str = "hold"
    soft_limit_margin: float = 0.0
    soft_limit_braking_zone: float = math.radians(5.0)
    braking_acceleration: float = 2.0
    command_timeout_s: float = 0.25
    enable_grace_s: float = 2.0
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
        velocity_range=joint.velocity_range,
        lower_limit=joint.lower_limit,
        upper_limit=joint.upper_limit,
    )


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
    maximum = _MIT_GAIN_MAX[name]
    if (
        np.any(~np.isfinite(values))
        or np.any(values < 0.0)
        or np.any(values > maximum)
    ):
        raise ValueError(f"MIT {name} values must be finite and in [0, {maximum:g}]")
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
    protection = data.get("gripper_protection", {}) or {}
    joint_safety = data.get("joint_safety", {}) or {}
    safety = data.get("safety", {}) or {}
    gripper_control_hz = float(
        protection.get("control_hz", data.get("rate", 500.0))
    )
    if not np.isfinite(gripper_control_hz) or gripper_control_hz <= 0.0:
        raise ValueError("gripper_protection.control_hz must be finite and positive")
    gripper_fault_action = str(
        safety.get("gripper_fault_action", "hold")
    ).strip().lower()
    if gripper_fault_action not in {"hold", "disable"}:
        raise ValueError("safety.gripper_fault_action must be 'hold' or 'disable'")

    configured_force_profiles = protection.get("force_profiles", {}) or {}
    if not isinstance(configured_force_profiles, dict):
        raise ValueError("gripper_protection.force_profiles must be a mapping")
    force_profiles = []
    profile_names = tuple(f"level_{level}" for level in range(1, 11))
    unknown_profiles = set(configured_force_profiles) - set(profile_names)
    if unknown_profiles:
        raise ValueError(
            "unknown gripper force profiles: "
            + ", ".join(sorted(map(str, unknown_profiles)))
        )
    for name, fallback in zip(profile_names, _DEFAULT_GRIPPER_FORCE_PROFILES):
        raw = configured_force_profiles.get(name, {}) or {}
        if not isinstance(raw, dict):
            raise ValueError(
                f"gripper_protection.force_profiles.{name} must be a mapping"
            )
        force_profiles.append(
            tuple(
                float(raw.get(field_name, default))
                for field_name, default in zip(_GRIPPER_PROFILE_FIELDS, fallback)
            )
        )

    return ArxDCanConfig(
        name=str(data.get("name", "ARX-D-CAN")),
        model=str(data.get("model", "custom")),
        hardware_config_path=(
            None if data.get("hardware_path") is None else str(data["hardware_path"])
        ),
        urdf_path=None if data.get("urdf_path") is None else str(data["urdf_path"]),
        end_effector_frame=str(data.get("end_effector_frame", "gripper_end")),
        port=str(data.get("channel", "/dev/ttyACM0") if port is None else port),
        transport=str(
            data.get("transport", "auto") if transport is None else transport
        ),
        baud=int(data.get("baud", 1_000_000) if baud is None else baud),
        control_hz=float(
            data.get("rate", 500.0) if control_hz is None else control_hz
        ),
        arm_control_mode=arm_control_mode,
        arm_joints=tuple(joints_by_name[name] for name in arm_names),
        gripper=joints_by_name.get(gripper_names[0]) if gripper_names else None,
        gripper_closed_value=float(gripper_mapping.get("closed_value", 0.0)),
        gripper_open_value=float(gripper_mapping.get("open_value", 2.64)),
        gripper_protection=GripperProtectionConfig(
            control_hz=gripper_control_hz,
            close_speed=float(protection.get("close_speed", 1.0)),
            contact_torque=float(protection.get("contact_torque", 0.8)),
            overload_torque=float(protection.get("overload_torque", 1.5)),
            motion_window_s=float(protection.get("motion_window_s", 0.2)),
            stall_movement=float(protection.get("stall_movement", 0.01)),
            min_position_error=float(protection.get("min_position_error", 0.05)),
            contact_hold_s=float(protection.get("contact_hold_s", 0.2)),
            overload_hold_s=float(protection.get("overload_hold_s", 0.05)),
            hold_offset=float(protection.get("hold_offset", 0.08)),
            retreat_distance=float(protection.get("retreat_distance", 0.15)),
            max_step_interval_s=float(protection.get("max_step_interval_s", 0.05)),
            overload_retreat_interval_s=float(
                protection.get("overload_retreat_interval_s", 0.1)
            ),
            hold_kp=float(protection.get("hold_kp", 2.0)),
            hold_kd=float(protection.get("hold_kd", 0.5)),
            force_profiles=tuple(force_profiles),
        ),
        gripper_fault_action=gripper_fault_action,
        soft_limit_margin=float(joint_safety.get("soft_limit_margin", 0.0)),
        soft_limit_braking_zone=float(
            joint_safety.get("soft_limit_braking_zone", math.radians(5.0))
        ),
        braking_acceleration=float(joint_safety.get("braking_acceleration", 2.0)),
        command_timeout_s=float(safety.get("command_timeout_s", 0.25)),
        enable_grace_s=float(safety.get("enable_grace_s", 2.0)),
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
    protection = config.gripper_protection
    result = {
        "name": config.name,
        "model": config.model,
        "hardware_path": config.hardware_config_path,
        "urdf_path": config.urdf_path,
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
        "gripper_protection": {
            "control_hz": protection.control_hz,
            "close_speed": protection.close_speed,
            "contact_torque": protection.contact_torque,
            "overload_torque": protection.overload_torque,
            "motion_window_s": protection.motion_window_s,
            "stall_movement": protection.stall_movement,
            "min_position_error": protection.min_position_error,
            "contact_hold_s": protection.contact_hold_s,
            "overload_hold_s": protection.overload_hold_s,
            "hold_offset": protection.hold_offset,
            "retreat_distance": protection.retreat_distance,
            "max_step_interval_s": protection.max_step_interval_s,
            "overload_retreat_interval_s": protection.overload_retreat_interval_s,
            "hold_kp": protection.hold_kp,
            "hold_kd": protection.hold_kd,
            "force_profiles": {
                name: dict(zip(_GRIPPER_PROFILE_FIELDS, profile))
                for name, profile in zip(
                    (f"level_{level}" for level in range(1, 11)),
                    protection.force_profiles,
                )
            },
        },
        "joint_safety": {
            "soft_limit_margin": config.soft_limit_margin,
            "soft_limit_braking_zone": config.soft_limit_braking_zone,
            "braking_acceleration": config.braking_acceleration,
        },
        "safety": {
            "command_timeout_s": config.command_timeout_s,
            "enable_grace_s": config.enable_grace_s,
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
    return result


__all__ = [
    "ArxDCanConfig",
    "JointMotorConfig",
    "default_config",
]
