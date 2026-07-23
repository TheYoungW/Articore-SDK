"""Standalone ARX-D-CAN arm SDK."""
from __future__ import annotations

import math
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np

from ..actuator import ArxDCan, JointCfg, load_cfg
from ..driver import build_scan_command, parse_scan_ids
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


@dataclass(slots=True, frozen=True)
class ArxDCanConfig:
    port: str = "/dev/ttyACM0"
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
    name: str = "ARX-D-CAN"
    model: str = "custom"
    hardware_config_path: str | None = None
    urdf_path: str | None = None
    end_effector_frame: str = "gripper_end"

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


def _config_from_loaded(
    data: dict,
    *,
    port: str | None = None,
    baud: int | None = None,
    control_hz: float = 100.0,
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
        end_effector_frame=str(data.get("end_effector_frame", "gripper_end")),
        port=str(port or data.get("channel", "/dev/ttyACM0")),
        baud=int(baud or data.get("baud", 1_000_000)),
        control_hz=control_hz,
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
    )


def default_config(
    *,
    model: str | None = None,
    config_path: str | Path | None = None,
    port: str | None = None,
    baud: int | None = None,
    control_hz: float = 100.0,
    arm_control_mode: str = "posvel",
) -> ArxDCanConfig:
    """Build the public SDK config from one built-in or custom model profile."""
    data = load_cfg(config_path, model=model)
    return _config_from_loaded(
        data,
        port=port,
        baud=baud,
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
            )
        )
        groups["gripper"] = {"joints": [joint.name]}
    force = config.gripper_force_control
    return {
        "name": config.name,
        "model": config.model,
        "hardware_path": config.hardware_config_path,
        "urdf_path": config.urdf_path,
        "end_effector_frame": config.end_effector_frame,
        "channel": config.port,
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
        },
    }


class ArxDCanArm:
    """High-level Python SDK for an ARX arm using Damiao motors over USB2CAN."""

    def __init__(
        self,
        *,
        port: str | None = None,
        baud: int | None = None,
        model: str | None = None,
        config_path: str | Path | None = None,
        config: ArxDCanConfig | None = None,
        control_mode: str = "posvel",
        enable_gripper: bool = False,
    ) -> None:
        if config is not None and (model is not None or config_path is not None):
            raise ValueError("config cannot be combined with model or config_path")
        if config is None:
            loaded_config = load_cfg(config_path, model=model)
            self.config = _config_from_loaded(
                loaded_config,
                port=port,
                baud=baud,
                arm_control_mode=control_mode,
            )
            loaded_config = dict(loaded_config)
            loaded_config["channel"] = self.config.port
            loaded_config["baud"] = self.config.baud
        else:
            self.config = config
            loaded_config = _actuator_config_from_sdk(config)
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
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._watchdog_deadline: float | None = None
        self._feedback_error_count = 0
        self._last_joint_command: tuple[float, ...] | None = None
        self._last_gripper_command: float | None = None
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

    def configure(self, mode: str | None = None) -> None:
        self._require_operational()
        try:
            self.configure_mode(mode or self._mode)
            if self.enable_gripper and self.config.gripper is not None:
                if not self.robot.gripper.mode_mit():
                    raise RuntimeError("ARX-D-CAN gripper did not enter MIT mode")
        except Exception as exc:
            self._trip_fault(f"configuration failed: {exc}")
            raise
        self._configured = True

    def close(self) -> None:
        """Stop command production, disable every motor, and close the bus."""
        self._stop_watchdog()
        error: Exception | None = None
        if self._connected:
            try:
                self.robot.disconnect()
            except Exception as exc:
                error = exc
        with self._state_lock:
            self._connected = False
            self._configured = False
            self._enabled = False
            self._safe_holding = False
            self._watchdog_deadline = None
        if error is not None:
            raise RuntimeError(f"ARX-D-CAN close failed: {error}") from error

    def enable(self) -> None:
        self._require_operational()
        if not self._configured:
            raise RuntimeError("ARX-D-CAN arm must be configured before enable")
        try:
            self.robot.arm.enable()
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
            self._watchdog_deadline = time.monotonic() + max(
                self.config.enable_grace_s,
                self.config.command_timeout_s,
            )
        self._start_watchdog()

    def disable(self) -> None:
        self._require_connected()
        self._stop_watchdog()
        try:
            self.robot.estop()
        finally:
            with self._gripper_command_lock:
                if self._gripper_force_controller is not None:
                    self._gripper_force_controller.reset()
            with self._state_lock:
                self._enabled = False
                self._safe_holding = False
                self._watchdog_deadline = None

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
            if not self.robot.arm.mode_pos_vel():
                raise RuntimeError("ARX-D-CAN arm did not enter POS_VEL mode")
            self._mode = "posvel"
            return
        if normalized == "mit":
            if not self.robot.arm.mode_mit():
                raise RuntimeError("ARX-D-CAN arm did not enter MIT mode")
            self._mode = "mit"
            return
        raise ValueError("mode must be 'posvel' or 'mit'")

    def read_state(self, *, request_feedback: bool = True) -> ArxDCanState:
        self._require_connected()
        try:
            pos, vel, tau = self.robot.get_state(
                request_feedback=request_feedback,
                require_complete=request_feedback,
                joint_names=self._active_joint_names(),
            )
        except Exception as exc:
            self._feedback_error_count += 1
            if self._enabled and self._feedback_error_count >= max(
                1, self.config.feedback_fault_threshold
            ):
                self._trip_fault(
                    f"feedback failed {self._feedback_error_count} consecutive times: {exc}"
                )
            raise
        self._feedback_error_count = 0
        arm_count = len(self.config.arm_joints)
        arm_pos = pos[:arm_count]
        arm_vel = vel[:arm_count]
        arm_tau = tau[:arm_count]
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
        return ArxDCanState(
            arm=JointState(
                names=self.config.joint_names,
                positions=tuple(float(value) for value in arm_pos),
                velocities=tuple(float(value) for value in arm_vel),
                torques=tuple(float(value) for value in arm_tau),
            ),
            gripper=gripper_state,
        )

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
        return parse_scan_ids(result.stdout)

    def send_joint_positions(
        self,
        positions: Sequence[float],
        *,
        velocities: Sequence[float] | None = None,
        velocity_limits: Sequence[float] | None = None,
        torques: Sequence[float] | None = None,
        mit_kp: Sequence[float] | None = None,
        mit_kd: Sequence[float] | None = None,
        mode: str | None = None,
        require_enabled: bool = True,
    ) -> None:
        """Send one arm command.

        In MIT mode, ``mit_kp`` and ``mit_kd`` override the YAML gains for
        this packet only. Omitting either argument keeps that gain at its
        configured YAML value.
        """
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
        mit_kp_target: np.ndarray | None = None
        if mit_kp is not None:
            if len(mit_kp) != joint_count:
                raise ValueError(
                    f"expected {joint_count} MIT Kp values, got {len(mit_kp)}"
                )
            if any(
                not math.isfinite(float(value)) or float(value) < 0.0
                for value in mit_kp
            ):
                raise ValueError("MIT Kp values must be finite and non-negative")
            mit_kp_target = np.asarray(mit_kp, dtype=np.float64)
        mit_kd_target: np.ndarray | None = None
        if mit_kd is not None:
            if len(mit_kd) != joint_count:
                raise ValueError(
                    f"expected {joint_count} MIT Kd values, got {len(mit_kd)}"
                )
            if any(
                not math.isfinite(float(value)) or float(value) < 0.0
                for value in mit_kd
            ):
                raise ValueError("MIT Kd values must be finite and non-negative")
            mit_kd_target = np.asarray(mit_kd, dtype=np.float64)
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
        try:
            if active_mode in ("posvel", "pv"):
                if self._mode != "posvel":
                    self.configure_mode("posvel")
                self.robot.arm.send_pos_vel(
                    np.array([target[joint.name] for joint in self.config.arm_joints]),
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
                self.robot.arm.send_mit(
                    np.array([target[joint.name] for joint in self.config.arm_joints]),
                    vel=velocity_target,
                    kp=mit_kp_target,
                    kd=mit_kd_target,
                    tau=torque_target,
                    strict=True,
                )
                self._record_successful_command(
                    joint_positions=tuple(
                        target[joint.name] for joint in self.config.arm_joints
                    )
                )
                return
        except Exception as exc:
            self._trip_fault(f"joint command failed: {exc}")
            raise
        raise ValueError("mode must be 'posvel' or 'mit'")

    def send_joint_torques(
        self,
        torques: Sequence[float],
        *,
        require_enabled: bool = True,
    ) -> None:
        """Send a pure MIT torque command with Kp=Kd=0 for every joint."""
        self._require_operational()
        if require_enabled and not self._enabled:
            raise RuntimeError("ARX-D-CAN arm is not enabled")
        joint_count = len(self.config.arm_joints)
        if len(torques) != joint_count:
            raise ValueError(
                f"expected {joint_count} joint torques, got {len(torques)}"
            )
        if any(not math.isfinite(float(value)) for value in torques):
            raise ValueError("joint torques must be finite")

        zeros = np.zeros(joint_count, dtype=np.float64)
        torque_target = np.asarray(torques, dtype=np.float64)
        try:
            if self._mode != "mit":
                self.configure_mode("mit")
            self.robot.arm.send_mit(
                zeros,
                vel=zeros,
                kp=zeros,
                kd=zeros,
                tau=torque_target,
                strict=True,
            )
            # Refresh the watchdog without recording the packet's irrelevant
            # zero position as a future safe-hold target.
            self._record_successful_command()
        except Exception as exc:
            self._trip_fault(f"joint torque command failed: {exc}")
            raise

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
                    self.robot.gripper.send_mit(
                        np.array([command.position]),
                        kp=np.array([command.kp]),
                        kd=np.array([command.kd]),
                        strict=True,
                    )
                except Exception:
                    self._gripper_force_controller.reset()
                    self._trip_fault("gripper command failed")
                    raise
                self._record_successful_command(
                    gripper_position=float(command.position)
                )
                return
            try:
                self.robot.gripper.send_mit(
                    np.array([target]),
                    strict=True,
                )
            except Exception as exc:
                self._trip_fault(f"gripper command failed: {exc}")
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
        gripper_position: float | None = None,
    ) -> None:
        with self._state_lock:
            if joint_positions is not None:
                self._last_joint_command = joint_positions
            if gripper_position is not None:
                self._last_gripper_command = gripper_position
            if self._enabled:
                self._watchdog_deadline = (
                    time.monotonic() + self.config.command_timeout_s
                )

    def _start_watchdog(self) -> None:
        if not self.config.watchdog_enabled:
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
                    self._enter_safe_hold(reason, expected_deadline=deadline)
                else:
                    self._trip_fault(reason)
                return

    def _enter_safe_hold(self, reason: str, *, expected_deadline: float) -> None:
        arm_count = len(self.config.arm_joints)
        joint_target: tuple[float, ...] | None = None
        gripper_target: float | None = None
        attempts = max(1, self.config.feedback_fault_threshold)
        for attempt in range(attempts):
            try:
                positions, _, _ = self.robot.get_state(
                    request_feedback=True,
                    require_complete=True,
                    joint_names=self._active_joint_names(),
                )
                joint_target = tuple(float(value) for value in positions[:arm_count])
                gripper_target = (
                    float(positions[arm_count])
                    if self.enable_gripper
                    and self.config.gripper is not None
                    and len(positions) > arm_count
                    else None
                )
                break
            except Exception:
                if attempt + 1 < attempts:
                    time.sleep(min(0.02, self.config.watchdog_poll_s))

        if joint_target is None:
            with self._state_lock:
                joint_target = self._last_joint_command
                gripper_target = self._last_gripper_command
            if joint_target is None:
                self._trip_fault(f"{reason}; current position unavailable")
                return

        with self._state_lock:
            if (
                not self._enabled
                or self._watchdog_deadline != expected_deadline
                or time.monotonic() <= expected_deadline
            ):
                return
            self._faulted = True
            self._safe_holding = True
            self._fault_reason = f"{reason}; safe hold active"
            self._watchdog_deadline = None

        period = 1.0 / self.config.safe_hold_hz
        while not self._watchdog_stop.is_set():
            try:
                target = np.asarray(joint_target, dtype=np.float64)
                if self._mode == "mit":
                    self.robot.arm.send_mit(target, strict=True)
                else:
                    self.robot.arm.send_pos_vel(target, strict=True)
                if gripper_target is not None:
                    kp = self.config.gripper_force_control.hold_kp
                    kd = self.config.gripper_force_control.hold_kd
                    self.robot.gripper.send_mit(
                        np.array([gripper_target]),
                        kp=np.array([kp]),
                        kd=np.array([kd]),
                        strict=True,
                    )
            except Exception as exc:
                self._trip_fault(f"safe hold command failed: {exc}")
                return
            self._watchdog_stop.wait(period)

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
