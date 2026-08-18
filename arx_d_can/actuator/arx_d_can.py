"""高层 SDK 内部使用的配置驱动执行器后端。"""
from __future__ import annotations

import time
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict, List, Optional
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from ..driver import (
    CallError,
    Controller,
    MotorMitCommand,
    Mode,
    NativeFeedbackMotorFaultError,
    NativeFeedbackTimeoutError,
    NativeFeedbackTransportError,
    NativeIncompleteFeedbackError,
    PosVelCommand,
    create_controller,
    resolve_transport,
)
from ..errors import (
    FeedbackTimeoutError,
    IncompleteFeedbackError,
    MotorFaultError,
    TransportError,
)

_CFG_DIR = Path(__file__).resolve().parents[1] / "config"
_MODEL_REGISTRY = _CFG_DIR / "models.yaml"
_HEALTHY_DAMIAO_STATUS_CODES = frozenset((0x0, 0x1))  # disabled, enabled
_COMPLETE_FEEDBACK_ATTEMPTS = 2
_NATIVE_TORQUE_RANGES = {
    "4310": 10.0,
    "4340P": 28.0,
    "8009": 54.0,
}
_NATIVE_VELOCITY_RANGES = {
    "4310": 30.0,
    "4340P": 10.0,
    "8009": 45.0,
}
_MIT_GAIN_MAX = {"Kp": 500.0, "Kd": 5.0}
_NATIVE_ROBOT_MODELS = frozenset(
    {"yunyi_v1_0_left", "yunyi_v1_0_right"}
)


def _validate_mit_gains(kp: np.ndarray, kd: np.ndarray) -> None:
    for name, values in (("Kp", kp), ("Kd", kd)):
        maximum = _MIT_GAIN_MAX[name]
        if (
            not np.all(np.isfinite(values))
            or np.any(values < 0.0)
            or np.any(values > maximum)
        ):
            raise ValueError(f"MIT {name} values must be in [0, {maximum:g}]")


def _transport_error(
    exc: CallError,
    *,
    operation: str,
    motor_names: tuple[str, ...] = (),
) -> TransportError:
    return TransportError(
        f"{operation} failed: {exc}",
        operation=operation,
        motor_names=motor_names,
        retryable=True,
    )


def _complete_feedback_error(
    errors: list[Exception],
    *,
    attempts: int,
    motor_names: tuple[str, ...],
    motor_names_by_id: dict[int, str],
    transport: str,
    channel: str,
) -> Exception:
    detail = "; ".join(str(error) for error in errors)
    attempt_label = "attempt" if attempts == 1 else "attempts"
    missing_ids = tuple(
        sorted(
            {
                int(motor_id)
                for error in errors
                for motor_id in getattr(error, "missing_motor_ids", ())
            }
        )
    )
    missing_detail = ""
    if missing_ids:
        labels = ", ".join(
            f"ID {motor_id} ({motor_names_by_id.get(motor_id, 'unknown')})"
            for motor_id in missing_ids
        )
        missing_detail = f"; channel {channel} missing {labels}"
    if any(isinstance(error, NativeFeedbackTransportError) for error in errors):
        return TransportError(
            f"fresh feedback failed after {attempts} {attempt_label}: "
            f"{detail}{missing_detail}",
            operation="request_feedback",
            transport=transport,
            channel=channel,
            motor_names=motor_names,
            retryable=True,
        )
    if any(isinstance(error, NativeFeedbackMotorFaultError) for error in errors):
        return MotorFaultError(
            f"fresh feedback failed after {attempts} {attempt_label}: {detail}"
        )
    # motor 的结构化异常可以明确区分超时与反馈不完整。自定义后端仍可能只抛出
    # 基础 CallError；这种情况保守地按“不完整”处理，而不是猜测成超时。
    error_type = (
        FeedbackTimeoutError
        if errors
        and all(isinstance(error, NativeFeedbackTimeoutError) for error in errors)
        else IncompleteFeedbackError
    )
    return error_type(
        f"fresh feedback failed after {attempts} {attempt_label}: "
        f"{detail}{missing_detail}",
        operation="request_feedback",
        transport=transport,
        channel=channel,
        motor_names=motor_names,
        retryable=True,
    )


def _feedback_request_error(
    exc: CallError,
    *,
    motor_names: tuple[str, ...],
) -> Exception:
    """把 motor 的稳定反馈分类映射为 SDK 公共异常。"""
    context = {
        "operation": "request_feedback",
        "motor_names": motor_names,
        "retryable": True,
    }
    if isinstance(exc, NativeFeedbackTimeoutError):
        return FeedbackTimeoutError(f"fresh feedback timed out: {exc}", **context)
    if isinstance(exc, NativeIncompleteFeedbackError):
        return IncompleteFeedbackError(
            f"fresh feedback is incomplete: {exc}",
            **context,
        )
    if isinstance(exc, NativeFeedbackTransportError):
        return TransportError(f"fresh feedback transport failed: {exc}", **context)
    if isinstance(exc, NativeFeedbackMotorFaultError):
        return MotorFaultError(f"motor fault reported during feedback: {exc}")
    return TransportError(f"fresh feedback request failed: {exc}", **context)


def _read_yaml_mapping(path: Path, *, description: str) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"{description} not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{description} must contain a YAML mapping: {path}")
    return data


def _load_model_registry() -> dict[str, Any]:
    data = _read_yaml_mapping(_MODEL_REGISTRY, description="model registry")
    models = data.get("models")
    if not isinstance(models, dict) or not models:
        raise ValueError("models.yaml must define a non-empty 'models' mapping")
    default_model = data.get("default_model")
    if not isinstance(default_model, str) or default_model not in models:
        raise ValueError("models.yaml default_model must reference a registered model")
    return data


def available_models() -> tuple[str, ...]:
    """按固定顺序返回内置机型配置名称。"""
    return tuple(sorted(str(name) for name in _load_model_registry()["models"]))


def _resolve_hw_cfg_path(
    hw_yaml: str | Path | None = None,
    *,
    model: str | None = None,
) -> tuple[Path, str | None]:
    if hw_yaml is not None and model is not None:
        raise ValueError("model and config_path are mutually exclusive")

    selected_model = model
    if hw_yaml is None:
        registry = _load_model_registry()
        selected_model = selected_model or str(registry["default_model"])
        models = registry["models"]
        if selected_model not in models:
            choices = ", ".join(sorted(str(name) for name in models))
            raise ValueError(
                f"unknown arm model {selected_model!r}; available models: {choices}"
            )
        entry = models[selected_model]
        if isinstance(entry, str):
            hw_yaml = entry
        elif isinstance(entry, dict) and isinstance(entry.get("config"), str):
            hw_yaml = entry["config"]
        else:
            raise ValueError(
                f"model registry entry {selected_model!r} must be a path or "
                "a mapping containing 'config'"
            )

    path = Path(hw_yaml).expanduser()
    if path.is_absolute():
        resolved = path
    elif path.is_file():
        resolved = path.resolve()
    else:
        resolved = (_CFG_DIR / path).resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"hardware config not found: {resolved}")
    return resolved, selected_model


def _select_product_arm(
    data: dict[str, Any],
    *,
    selected_model: str | None,
    path: Path,
) -> dict[str, Any]:
    """从产品级配置中选出一条独立 CAN 通道对应的机械臂。"""
    arms = data.get("arms")
    if arms is None:
        return data
    if not isinstance(arms, dict) or not arms:
        raise ValueError(f"product config must define a non-empty arms mapping: {path}")
    if selected_model is None:
        raise ValueError(
            f"product config contains multiple arms; select a registered model: {path}"
        )

    entry = _load_model_registry()["models"].get(selected_model)
    side = entry.get("arm") if isinstance(entry, dict) else None
    if not isinstance(side, str) or side not in arms:
        raise ValueError(
            f"model {selected_model!r} must select one arm from product config {path}"
        )
    arm = arms[side]
    if not isinstance(arm, dict):
        raise ValueError(f"arms.{side} must be a mapping: {path}")

    merged = {key: value for key, value in data.items() if key != "arms"}
    merged.update(arm)
    merged["product_name"] = str(data.get("name", path.stem))
    merged["arm_side"] = side
    return merged


# --------------------------------------------------------------------------
# 配置加载
# --------------------------------------------------------------------------

@dataclass
class JointCfg:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    direction: float = 1.0
    torque_range: float | None = None
    effort_limit: float | None = None
    velocity_range: float | None = None
    kp: float = 0.0
    kd: float = 0.0
    vel_kp: float = 0.0
    vel_ki: float = 0.0
    pos_kp: float = 0.0
    pos_ki: float = 0.0
    vlim: float = 0.0
    lower_limit: float | None = None
    upper_limit: float | None = None


def _finite(value: Any, *, field: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{field} must be finite")
    return parsed


def _torque_range_scales(joint: JointCfg) -> tuple[float, float]:
    """返回自定义 MIT 力矩范围对应的命令与反馈缩放系数。"""
    if joint.torque_range is None:
        return 1.0, 1.0
    native_range = _NATIVE_TORQUE_RANGES.get(joint.model)
    if native_range is None:
        raise ValueError(
            f"{joint.name}.torque_range requires a known Damiao model, "
            f"got {joint.model!r}"
        )
    return native_range / joint.torque_range, joint.torque_range / native_range


def _velocity_range_scales(joint: JointCfg) -> tuple[float, float]:
    """返回自定义 MIT 速度映射对应的命令与反馈缩放系数。"""
    if joint.velocity_range is None:
        return 1.0, 1.0
    native_range = _NATIVE_VELOCITY_RANGES.get(joint.model)
    if native_range is None:
        raise ValueError(
            f"{joint.name}.velocity_range requires a known Damiao model, "
            f"got {joint.model!r}"
        )
    return native_range / joint.velocity_range, joint.velocity_range / native_range


def _resolve_resource_path(
    hw_path: Path,
    value: Any,
    *,
    description: str,
) -> Path | None:
    if value in (None, ""):
        return None
    path = Path(str(value)).expanduser()
    candidates = [path] if path.is_absolute() else [hw_path.parent / path, _CFG_DIR.parent / path]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(f"{description} not found for {hw_path}: {value}")


def _resolve_urdf_path(hw_path: Path, value: Any) -> Path | None:
    return _resolve_resource_path(hw_path, value, description="URDF")


def _apply_urdf_joint_limits(
    urdf_path: Path | None,
    joints: list[JointCfg],
) -> None:
    """将 URDF 中的位置和力矩限制关联到逻辑关节坐标。"""
    if urdf_path is None:
        return
    root = ET.parse(urdf_path).getroot()
    urdf_joints = {
        element.attrib.get("name"): element
        for element in root.findall("joint")
    }
    for joint in joints:
        element = urdf_joints.get(joint.name)
        if element is None:
            continue
        limit = element.find("limit")
        if limit is None:
            continue
        lower_text = limit.attrib.get("lower")
        upper_text = limit.attrib.get("upper")
        if lower_text is not None and upper_text is not None:
            lower = _finite(lower_text, field=f"{joint.name}.limit.lower")
            upper = _finite(upper_text, field=f"{joint.name}.limit.upper")
            if lower > upper:
                raise ValueError(
                    f"{joint.name} URDF lower limit must not exceed upper limit"
                )
            joint.lower_limit = lower
            joint.upper_limit = upper
        effort_text = limit.attrib.get("effort")
        if effort_text is not None:
            urdf_effort = _finite(
                effort_text,
                field=f"{joint.name}.limit.effort",
            )
            if urdf_effort <= 0.0:
                raise ValueError(f"{joint.name} URDF effort limit must be positive")
            if joint.effort_limit is None:
                joint.effort_limit = urdf_effort
            elif joint.effort_limit > urdf_effort:
                raise ValueError(
                    f"{joint.name}.effort_limit cannot exceed URDF effort "
                    f"{urdf_effort}"
                )


def _apply_native_joint_limits(
    model: str,
    joints: list[JointCfg],
) -> None:
    """从 Runtime 私有产品模型读取内置机械臂的位置边界。"""
    from ..native_robotics import load_native_robot_model

    by_name = {joint.name: joint for joint in joints}
    with load_native_robot_model(model) as native:
        missing = set(native.joint_names).difference(by_name)
        if missing:
            raise ValueError(
                f"{model} native model references unknown joints: "
                + ", ".join(sorted(missing))
            )
        for name, lower, upper in zip(
            native.joint_names,
            native.lower_position_limits,
            native.upper_position_limits,
        ):
            joint = by_name[name]
            joint.lower_limit = float(lower)
            joint.upper_limit = float(upper)


def _optional_mapping(data: dict[str, Any], field: str) -> dict[str, Any]:
    value = data.get(field, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be a mapping")
    return value


def _validate_groups(groups: Any, joints: list[JointCfg], *, path: Path) -> dict[str, Any]:
    if not isinstance(groups, dict) or not groups:
        raise ValueError(f"hardware config must define non-empty groups: {path}")
    known = {joint.name for joint in joints}
    normalized: dict[str, Any] = {}
    referenced: set[str] = set()
    for group_name, group_data in groups.items():
        if not isinstance(group_data, dict):
            raise ValueError(f"group {group_name!r} must be a mapping")
        names = group_data.get("joints")
        if not isinstance(names, list):
            raise ValueError(f"group {group_name!r} must define a joints list")
        names = [str(name) for name in names]
        if len(names) != len(set(names)):
            raise ValueError(f"group {group_name!r} contains duplicate joints")
        unknown = set(names).difference(known)
        if unknown:
            raise ValueError(
                f"group {group_name!r} references unknown joints: "
                + ", ".join(sorted(unknown))
            )
        duplicates = referenced.intersection(names)
        if duplicates:
            raise ValueError(
                "joints may belong to only one configured group: "
                + ", ".join(sorted(duplicates))
            )
        referenced.update(names)
        normalized[str(group_name)] = {**group_data, "joints": names}
    arm_names = normalized.get("arm", {}).get("joints", [])
    if not arm_names:
        raise ValueError("hardware config must define at least one joint in groups.arm")
    gripper_names = normalized.get("gripper", {}).get("joints", [])
    if len(gripper_names) > 1:
        raise ValueError("groups.gripper supports at most one joint")
    return normalized


def load_cfg(
    hw_yaml: str | Path | None = None,
    *,
    model: str | None = None,
) -> dict[str, Any]:
    """加载并校验一个机型配置。

    ``hw_yaml`` 继续作为兼容旧版的位置参数，用于传入自定义配置；新代码可改用
    ``model`` 选择内置机型。
    """
    hw_path, selected_model = _resolve_hw_cfg_path(hw_yaml, model=model)
    data = _select_product_arm(
        _read_yaml_mapping(hw_path, description="hardware config"),
        selected_model=selected_model,
        path=hw_path,
    )

    raw_joints = data.get("joints")
    if not isinstance(raw_joints, list) or not raw_joints:
        raise ValueError(f"hardware config must define a non-empty joints list: {hw_path}")
    joints: list[JointCfg] = []
    names: set[str] = set()
    motor_ids: set[int] = set()
    feedback_ids: set[int] = set()
    for index, j in enumerate(raw_joints):
        if not isinstance(j, dict):
            raise ValueError(f"joints[{index}] must be a mapping")
        mc = j.get("MIT", {})
        pc = j.get("POS_VEL", {})
        if not isinstance(mc, dict) or not isinstance(pc, dict):
            raise ValueError(
                f"joints[{index}] MIT and POS_VEL must be mappings"
            )
        name = str(j.get("name", "")).strip()
        if not name:
            raise ValueError(f"joints[{index}].name is required")
        motor_id = int(j["motor_id"])
        feedback_id = int(j["feedback_id"])
        if name in names:
            raise ValueError(f"duplicate joint name: {name}")
        if motor_id in motor_ids:
            raise ValueError(f"duplicate motor_id: 0x{motor_id:X}")
        if feedback_id in feedback_ids:
            raise ValueError(f"duplicate feedback_id: 0x{feedback_id:X}")
        if not 0 <= motor_id <= 0x7FF or not 0 <= feedback_id <= 0x7FF:
            raise ValueError(f"joint {name} CAN IDs must be in 0..0x7FF")
        names.add(name)
        motor_ids.add(motor_id)
        feedback_ids.add(feedback_id)
        model = str(j.get("model", "4340P"))
        raw_torque_range = j.get("torque_range")
        torque_range = (
            None
            if raw_torque_range is None
            else _finite(raw_torque_range, field=f"{name}.torque_range")
        )
        raw_effort_limit = j.get("effort_limit")
        effort_limit = (
            None
            if raw_effort_limit is None
            else _finite(raw_effort_limit, field=f"{name}.effort_limit")
        )
        raw_velocity_range = j.get("velocity_range")
        velocity_range = (
            None
            if raw_velocity_range is None
            else _finite(raw_velocity_range, field=f"{name}.velocity_range")
        )
        joint = JointCfg(
            name=name,
            motor_id=motor_id,
            feedback_id=feedback_id,
            model=model,
            direction=_finite(
                j.get("direction", 1.0),
                field=f"{name}.direction",
            ),
            torque_range=torque_range,
            effort_limit=effort_limit,
            velocity_range=velocity_range,
            kp=_finite(mc.get("kp", 0.0), field=f"{name}.MIT.kp"),
            kd=_finite(mc.get("kd", 0.0), field=f"{name}.MIT.kd"),
            vel_kp=_finite(pc.get("vel_kp", 0.0), field=f"{name}.POS_VEL.vel_kp"),
            vel_ki=_finite(pc.get("vel_ki", 0.0), field=f"{name}.POS_VEL.vel_ki"),
            pos_kp=_finite(pc.get("pos_kp", 0.0), field=f"{name}.POS_VEL.pos_kp"),
            pos_ki=_finite(pc.get("pos_ki", 0.0), field=f"{name}.POS_VEL.pos_ki"),
            vlim=_finite(pc.get("vlim", 2.0), field=f"{name}.POS_VEL.vlim"),
        )
        if joint.direction not in (-1.0, 1.0):
            raise ValueError(f"{name}.direction must be 1 or -1")
        if joint.torque_range is not None:
            if joint.torque_range <= 0.0:
                raise ValueError(f"{name}.torque_range must be positive")
            _torque_range_scales(joint)
        if joint.effort_limit is not None and joint.effort_limit <= 0.0:
            raise ValueError(f"{name}.effort_limit must be positive")
        if joint.velocity_range is not None:
            if joint.velocity_range <= 0.0:
                raise ValueError(f"{name}.velocity_range must be positive")
            _velocity_range_scales(joint)
        if joint.vlim <= 0.0:
            raise ValueError(f"{name}.POS_VEL.vlim must be positive")
        joints.append(joint)

    groups = _validate_groups(data.get("groups"), joints, path=hw_path)
    if selected_model in _NATIVE_ROBOT_MODELS:
        # URDF remains a public robot-description resource for visualization
        # and external tools. Runtime model info, rather than XML parsing, is
        # authoritative for built-in product control limits.
        urdf_path = _resolve_urdf_path(hw_path, data.get("urdf_path"))
        _apply_native_joint_limits(selected_model, joints)
    else:
        urdf_path = _resolve_urdf_path(hw_path, data.get("urdf_path"))
        _apply_urdf_joint_limits(urdf_path, joints)
    end_effector_frame = str(
        data.get("end_effector_frame", "gripper_end")
    ).strip()
    channel = str(data.get("channel", "/dev/ttyACM0")).strip()
    transport = str(data.get("transport", "auto")).strip().lower()
    baud = int(data.get("baud", 1_000_000))
    rate = _finite(data.get("rate", 500.0), field="rate")
    if not channel:
        raise ValueError("channel must not be empty")
    resolve_transport(transport, channel)
    if baud <= 0:
        raise ValueError("baud must be positive")
    if rate <= 0.0:
        raise ValueError("rate must be positive")
    if not end_effector_frame:
        raise ValueError("end_effector_frame must not be empty")

    return {
        "name": data.get("name", "ARX-D-CAN"),
        "product_name": data.get("product_name"),
        "arm_side": data.get("arm_side"),
        "model": selected_model or str(data.get("name", hw_path.stem)),
        "hardware_path": str(hw_path),
        "urdf_path": None if urdf_path is None else str(urdf_path),
        "end_effector_frame": end_effector_frame,
        "channel": channel,
        "transport": transport,
        "baud": baud,
        "rate": rate,
        "groups": groups,
        "joints": joints,
        "motion": _optional_mapping(data, "motion"),
        "gripper_profile": (
            None
            if data.get("gripper_profile") is None
            else str(data["gripper_profile"]).strip()
        ),
        "safety": _optional_mapping(data, "safety"),
    }


# --------------------------------------------------------------------------
# NoOpGroup — 无执行器时的空操作桩
# --------------------------------------------------------------------------

class NoOpGroup:
    """当配置中不存在 gripper 组时的空实现。

    所有属性和方法与 JointGroup 接口兼容，但不对电机发送任何指令，
    方便用户代码在有/无夹爪时共用同一套逻辑，无需条件判断。
    """

    name: str = "gripper"
    _mode: str = "mit"

    @property
    def num_joints(self) -> int:
        return 0

    @property
    def joint_names(self) -> List[str]:
        return []

    @property
    def mode(self) -> str:
        return "mit"

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def clear_errors(self) -> tuple[str, ...]:
        return ()

    def mode_mit(self, kp=None, kd=None) -> bool:
        self._mode = "mit"
        return True

    def mode_pos_vel(self, vlim=None) -> bool:
        self._mode = "pos_vel"
        return True

    def send_mit(
        self,
        pos,
        vel=None,
        kp=None,
        kd=None,
        tau=None,
        *,
        strict: bool = False,
    ) -> None:
        pass

    def send_pos_vel(self, pos, vlim=None) -> None:
        pass

    def read_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        empty = np.array([], dtype=np.float64)
        return empty, empty.copy(), empty.copy()

    def __repr__(self) -> str:
        return "NoOpGroup(gripper, no actuator)"


# --------------------------------------------------------------------------
# JointGroup — 单组关节控制
# --------------------------------------------------------------------------

class JointGroup:
    """一组关节的独立控制器。

    每组拥有独立的控制模式（MIT / POS_VEL）、PID 参数和电机列表，
    可单独使能、切换模式、发送命令。

    由 ArxDCan 通过 __getattr__ 代理访问，例如 arm.arm / arm.gripper。
    组内关节数量、顺序由配置决定。
    """

    def __init__(
        self,
        name: str,
        joint_names: List[str],
        all_joints: List[JointCfg],
        motor_map: Dict[str, any],
        ctrl_map: Dict[str, Controller],
    ) -> None:
        self.name = name
        self._jn: List[str] = joint_names
        self._jcfgs: List[JointCfg] = [
            next(j for j in all_joints if j.name == n) for n in joint_names
        ]
        self._mm: Dict[str, any] = motor_map
        self._cm: Dict[str, Controller] = ctrl_map
        self._mode: str = "mit"
        self._mit_kp: np.ndarray = np.array([j.kp for j in self._jcfgs], dtype=np.float64)
        self._mit_kd: np.ndarray = np.array([j.kd for j in self._jcfgs], dtype=np.float64)
        _validate_mit_gains(self._mit_kp, self._mit_kd)
        self._pv_vlim: np.ndarray = np.array([j.vlim for j in self._jcfgs], dtype=np.float64)

    # ── 属性 ────────────────────────────────────────────────────────────

    @property
    def num_joints(self) -> int:
        return len(self._jn)

    @property
    def joint_names(self) -> List[str]:
        return list(self._jn)

    @property
    def mode(self) -> str:
        return self._mode

    def _controller_for_batch(self) -> Controller:
        """返回本组所在的底层 Controller，仅供 SDK 组合批量发送。"""
        try:
            return self._cm["main"]
        except KeyError as exc:
            raise RuntimeError("joint group is not connected") from exc

    def _make_mit_batch_commands(
        self,
        pos: np.ndarray,
        vel: Optional[np.ndarray] = None,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
        tau: Optional[np.ndarray] = None,
    ) -> tuple[MotorMitCommand, ...]:
        """生成已完成限位和方向换算的底层 MIT 批量命令。"""
        n = self.num_joints
        position = np.asarray(pos, dtype=np.float64).reshape(-1)
        velocity = (
            np.zeros(n)
            if vel is None
            else np.asarray(vel, dtype=np.float64).reshape(-1)
        )
        stiffness = (
            self._mit_kp
            if kp is None
            else np.asarray(kp, dtype=np.float64).reshape(-1)
        )
        damping = (
            self._mit_kd
            if kd is None
            else np.asarray(kd, dtype=np.float64).reshape(-1)
        )
        torque = (
            np.zeros(n)
            if tau is None
            else np.asarray(tau, dtype=np.float64).reshape(-1)
        )
        vectors = (position, velocity, stiffness, damping, torque)
        if any(vector.size != n for vector in vectors):
            raise ValueError(f"MIT batch expects {n} values per vector")
        if any(not np.all(np.isfinite(vector)) for vector in vectors):
            raise ValueError("MIT batch values must be finite")
        _validate_mit_gains(stiffness, damping)

        commands = []
        for index, joint in enumerate(self._jcfgs):
            velocity_scale, _ = _velocity_range_scales(joint)
            torque_scale, _ = _torque_range_scales(joint)
            limited_position = self._clamp_position(joint, float(position[index]))
            limited_torque = self._clamp_effort(joint, float(torque[index]))
            commands.append(
                MotorMitCommand(
                    self._mm[joint.name],
                    joint.direction * limited_position,
                    joint.direction * float(velocity[index]) * velocity_scale,
                    float(stiffness[index]),
                    float(damping[index]),
                    joint.direction * limited_torque * torque_scale,
                )
            )
        return tuple(commands)

    def _make_pos_vel_batch_commands(
        self,
        pos: np.ndarray,
        vlim: Optional[np.ndarray] = None,
    ) -> tuple[PosVelCommand, ...]:
        """生成已完成限位和方向换算的底层 PV 批量命令。"""
        n = self.num_joints
        position = np.asarray(pos, dtype=np.float64).reshape(-1)
        velocity_limit = (
            self._pv_vlim
            if vlim is None
            else np.asarray(vlim, dtype=np.float64).reshape(-1)
        )
        if position.size != n or velocity_limit.size != n:
            raise ValueError(
                f"POS_VEL batch expects {n} positions and velocity limits"
            )
        if not np.all(np.isfinite(position)) or not np.all(
            np.isfinite(velocity_limit)
        ):
            raise ValueError("POS_VEL batch values must be finite")
        if np.any(velocity_limit <= 0.0):
            raise ValueError("POS_VEL velocity limits must be positive")

        return tuple(
            PosVelCommand(
                self._mm[joint.name],
                joint.direction
                * self._clamp_position(joint, float(position[index])),
                float(velocity_limit[index]),
            )
            for index, joint in enumerate(self._jcfgs)
        )

    # ── 使能 / 失能 ────────────────────────────────────────────────────

    def enable(
        self,
        poll_max: int = 20,
        poll_interval: float = 0.05,
        *,
        mit_position: Optional[np.ndarray] = None,
        mit_velocity: Optional[np.ndarray] = None,
        mit_kp: Optional[np.ndarray] = None,
        mit_kd: Optional[np.ndarray] = None,
        mit_tau: Optional[np.ndarray] = None,
    ) -> None:
        """使能组内所有电机，并可选择立即写入一条 MIT 保持命令。"""
        hold_commands = None
        if mit_position is not None:
            hold_commands = self._make_mit_batch_commands(
                mit_position,
                vel=mit_velocity,
                kp=mit_kp,
                kd=mit_kd,
                tau=mit_tau,
            )

        errors = []
        for index, jc in enumerate(self._jcfgs):
            try:
                motor = self._mm[jc.name]
                motor.enable()
                if hold_commands is not None:
                    command = hold_commands[index]
                    motor.send_mit(
                        command.pos,
                        command.vel,
                        command.kp,
                        command.kd,
                        command.tau,
                    )
            except Exception as exc:
                errors.append(f"{jc.name}: {exc}")
                break
        time.sleep(0.05)

        if not errors:
            for jc in self._jcfgs:
                try:
                    self._wait_for_enabled_state(jc, poll_max, poll_interval)
                except Exception as exc:
                    errors.append(f"{jc.name}: {exc}")
        if not errors:
            return

        try:
            self.disable()
        except Exception as exc:
            errors.append(f"rollback disable: {exc}")
        raise RuntimeError(
            "not all motors entered ENABLED state: " + "; ".join(errors)
        )

    def disable(
        self,
        poll_max: int = 20,
        poll_interval: float = 0.05,
    ) -> None:
        """失能组内所有电机，并通过新鲜反馈确认其处于失能状态。"""
        errors = []
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].disable()
            except Exception as exc:
                errors.append(f"{jc.name}: {exc}")
        time.sleep(0.05)
        for jc in self._jcfgs:
            try:
                self._wait_for_disabled_state(jc, poll_max, poll_interval)
            except Exception as exc:
                errors.append(f"{jc.name}: {exc}")
        if errors:
            raise RuntimeError("failed to disable motors: " + "; ".join(errors))

    def clear_errors(
        self,
        poll_max: int = 20,
        poll_interval: float = 0.05,
    ) -> tuple[str, ...]:
        """清除本组所有电机故障，并确保所有电机保持失能。"""
        completed: list[str] = []
        errors: list[str] = []
        for jc in self._jcfgs:
            motor = self._mm[jc.name]
            try:
                try:
                    motor.disable()
                except Exception:
                    # 故障电机在 clear_error 成功前可能拒绝失能命令。
                    pass
                motor.clear_error()
                time.sleep(0.05)
                motor.disable()
                self._wait_for_disabled_state(jc, poll_max, poll_interval)
                completed.append(jc.name)
            except Exception as exc:
                errors.append(f"{jc.name}: {exc}")
        if errors:
            raise RuntimeError(
                "failed to clear motor faults; "
                f"cleared={completed}: {'; '.join(errors)}"
            )
        return tuple(completed)

    def _wait_for_disabled_state(
        self,
        jc: JointCfg,
        poll_max: int,
        poll_interval: float,
    ):
        motor = self._mm[jc.name]
        last_state = None
        last_error = None
        for _ in range(max(1, poll_max)):
            try:
                last_state = motor.request_fresh_state(timeout_ms=50)
                if last_state is not None and last_state.status_code == 0:
                    return last_state
            except Exception as exc:
                last_error = exc
            time.sleep(max(0.0, poll_interval))
        status = None if last_state is None else last_state.status_code
        detail = f", last_error={last_error}" if last_error is not None else ""
        raise RuntimeError(
            f"fresh DISABLED feedback unavailable, status={status}{detail}"
        )

    def _wait_for_enabled_state(
        self,
        jc: JointCfg,
        poll_max: int,
        poll_interval: float,
    ):
        motor = self._mm[jc.name]
        last_state = None
        last_error = None
        for _ in range(max(1, poll_max)):
            try:
                last_state = motor.request_fresh_state(timeout_ms=50)
                if last_state is not None and last_state.status_code == 1:
                    return last_state
            except Exception as exc:
                last_error = exc
            time.sleep(max(0.0, poll_interval))
        status = None if last_state is None else last_state.status_code
        detail = f", last_error={last_error}" if last_error is not None else ""
        raise RuntimeError(
            f"enabled feedback unavailable, status={status}{detail}"
        )

    # ── 模式切换 ────────────────────────────────────────────────────────

    def _write_pv_params(self, jc: JointCfg) -> None:
        m = self._mm[jc.name]
        m.write_register_f32(25, jc.vel_kp)
        m.write_register_f32(26, jc.vel_ki)
        m.write_register_f32(27, jc.pos_kp)
        m.write_register_f32(28, jc.pos_ki)
        time.sleep(0.02)

    def mode_mit(
        self,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
    ) -> bool:
        stiffness = (
            self._mit_kp
            if kp is None
            else np.asarray(kp, dtype=np.float64).reshape(-1)
        )
        damping = (
            self._mit_kd
            if kd is None
            else np.asarray(kd, dtype=np.float64).reshape(-1)
        )
        if stiffness.size != self.num_joints or damping.size != self.num_joints:
            raise ValueError(f"MIT mode expects {self.num_joints} Kp/Kd values")
        _validate_mit_gains(stiffness, damping)
        self._mit_kp = stiffness
        self._mit_kd = damping
        self._mode = "mit"
        ok = True
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].ensure_mode(Mode.MIT, 1000)
            except CallError as e:
                print(f"[{self.name}/mode_mit/{jc.name}] {e}")
                ok = False
            time.sleep(0.05)
        time.sleep(0.2)
        return ok

    def mode_pos_vel(
        self,
        vlim: Optional[np.ndarray] = None,
    ) -> bool:
        if self.name == "gripper":
            raise ValueError("the gripper only supports MIT mode")
        self._mode = "pos_vel"
        if vlim is not None:
            self._pv_vlim = np.asarray(vlim, dtype=np.float64).reshape(-1)
        ok = True
        for jc in self._jcfgs:
            self._write_pv_params(jc)
            try:
                self._mm[jc.name].ensure_mode(Mode.POS_VEL, 1000)
            except CallError as e:
                print(f"[{self.name}/mode_pos_vel/{jc.name}] {e}")
                ok = False
            time.sleep(0.05)
        time.sleep(0.2)
        return ok

    @staticmethod
    def _clamp_position(joint: JointCfg, position: float) -> float:
        if joint.lower_limit is not None:
            position = max(position, joint.lower_limit)
        if joint.upper_limit is not None:
            position = min(position, joint.upper_limit)
        return position

    @staticmethod
    def _clamp_effort(joint: JointCfg, torque: float) -> float:
        if joint.effort_limit is None:
            return torque
        return float(np.clip(torque, -joint.effort_limit, joint.effort_limit))

    # ── MIT 发送 ────────────────────────────────────────────────────────

    def send_mit(
        self,
        pos: np.ndarray,
        vel: Optional[np.ndarray] = None,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
        tau: Optional[np.ndarray] = None,
        *,
        strict: bool = True,
    ) -> None:
        commands = self._make_mit_batch_commands(pos, vel, kp, kd, tau)
        for jc, command in zip(self._jcfgs, commands):
            try:
                self._mm[jc.name].send_mit(
                    command.pos,
                    command.vel,
                    command.kp,
                    command.kd,
                    command.tau,
                )
            except CallError as exc:
                if strict:
                    raise _transport_error(
                        exc,
                        operation="send_mit",
                        motor_names=(jc.name,),
                    ) from exc

    # ── POS_VEL 发送 ───────────────────────────────────────────────────

    def send_pos_vel(
        self,
        pos: np.ndarray,
        vlim: Optional[np.ndarray] = None,
        *,
        strict: bool = True,
    ) -> None:
        pos = np.asarray(pos, dtype=np.float64).reshape(-1)
        if vlim is None:
            vlim = self._pv_vlim
        vlim = np.asarray(vlim, dtype=np.float64).reshape(-1)
        for i in range(min(len(pos), len(vlim))):
            try:
                jc = self._jcfgs[i]
                limited_position = self._clamp_position(jc, float(pos[i]))
                self._mm[jc.name].send_pos_vel(
                    jc.direction * limited_position,
                    float(vlim[i]),
                )
            except CallError as exc:
                if strict:
                    raise _transport_error(
                        exc,
                        operation="send_pos_vel",
                        motor_names=(jc.name,),
                    ) from exc

    # ── 状态读取 ───────────────────────────────────────────────────────

    def _request_feedback(self) -> None:
        try:
            self._cm["main"].request_feedback_all(timeout_ms=50)
        except CallError as exc:
            raise _feedback_request_error(
                exc,
                motor_names=tuple(joint.name for joint in self._jcfgs),
            ) from exc

    def read_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """读取本组状态；无法取得新鲜且健康的反馈时抛出异常。"""
        if request_feedback:
            try:
                self._cm["main"].request_feedback_all(timeout_ms=50)
            except CallError as exc:
                raise _feedback_request_error(
                    exc,
                    motor_names=tuple(joint.name for joint in self._jcfgs),
                ) from exc

        positions, velocities, torques = [], [], []
        for jc in self._jcfgs:
            try:
                state = self._mm[jc.name].get_state()
            except CallError as exc:
                raise FeedbackTimeoutError(
                    f"{self.name}/{jc.name}: cached feedback read failed: {exc}",
                    operation="read_feedback",
                    motor_names=(jc.name,),
                    retryable=True,
                ) from exc
            if state is None:
                raise IncompleteFeedbackError(
                    f"{self.name}/{jc.name}: no fresh feedback",
                    operation="read_feedback",
                    motor_names=(jc.name,),
                    retryable=True,
                )
            if getattr(state, "status_code", 0) not in _HEALTHY_DAMIAO_STATUS_CODES:
                raise MotorFaultError(
                    f"{self.name}/{jc.name}: motor fault status={state.status_code}",
                    status_codes={jc.name: state.status_code},
                )
            _, velocity_feedback_scale = _velocity_range_scales(jc)
            _, torque_feedback_scale = _torque_range_scales(jc)
            positions.append(jc.direction * state.pos)
            velocities.append(
                jc.direction * state.vel * velocity_feedback_scale
            )
            torques.append(
                jc.direction * state.torq * torque_feedback_scale
            )
        return (
            np.asarray(positions, dtype=np.float64),
            np.asarray(velocities, dtype=np.float64),
            np.asarray(torques, dtype=np.float64),
        )

    def __repr__(self) -> str:
        return f"JointGroup({self.name!r}, joints={self.num_joints}, mode={self._mode})"


# --------------------------------------------------------------------------
# ArxDCan — 分组控制器容器
# --------------------------------------------------------------------------

class ArxDCan:
    """持有高层 SDK 所需电机句柄和内部关节分组。"""

    def __init__(
        self,
        hw_yaml: str | Path | None = None,
        channel: str | None = None,
        baud: int | None = None,
        transport: str | None = None,
        joint_names: Optional[List[str]] = None,
        *,
        model: str | None = None,
        config_data: dict[str, Any] | None = None,
    ) -> None:
        if config_data is not None and (hw_yaml is not None or model is not None):
            raise ValueError("config_data cannot be combined with model or config_path")
        cfg = dict(config_data) if config_data is not None else load_cfg(hw_yaml, model=model)
        self._model = str(cfg.get("model", "custom"))
        if channel:
            cfg["channel"] = channel
        if transport is not None:
            cfg["transport"] = transport

        self._name: str = cfg["name"]
        self._channel: str = cfg["channel"]
        self._transport: str = resolve_transport(
            str(cfg.get("transport", "auto")),
            self._channel,
        )
        self._baud: int = int(baud or cfg.get("baud", 1_000_000))
        self._rate: float = cfg["rate"]
        configured_joints: List[JointCfg] = cfg["joints"]
        if joint_names is None:
            self._all_joints = configured_joints
        else:
            requested_names = set(joint_names)
            configured_names = {joint.name for joint in configured_joints}
            unknown = requested_names.difference(configured_names)
            if unknown:
                raise ValueError(
                    f"unknown configured joints: {', '.join(sorted(unknown))}"
                )
            self._all_joints = [
                joint for joint in configured_joints if joint.name in requested_names
            ]
            if not self._all_joints:
                raise ValueError("at least one configured joint must be active")
        self._groups_def: dict = cfg["groups"]

        self._ctrl_map: Dict[str, Controller] = {}
        self._motor_map: Dict[str, any] = {}
        self._groups: Dict[str, JointGroup] = {}

        self._connected: bool = False

        self._build_groups()

    def connect(self) -> None:
        """连接总线、注册电机。模式切换需在 connect 后调用。"""
        if self._connected:
            return
        self._setup_motors()
        self._connected = True

    def _make_controller(self) -> Controller:
        return create_controller(
            transport=self._transport,
            channel=self._channel,
            baud=self._baud,
        )

    def _setup_motors(self) -> None:
        if "main" not in self._ctrl_map:
            self._ctrl_map["main"] = self._make_controller()
        ctrl = self._ctrl_map["main"]
        for jc in self._all_joints:
            mot = ctrl.add_damiao_motor(jc.motor_id, jc.feedback_id, jc.model)
            self._motor_map[jc.name] = mot

    def _build_groups(self) -> None:
        active_names = {joint.name for joint in self._all_joints}
        for gname, gdef in self._groups_def.items():
            joints_def = [
                name for name in gdef.get("joints", []) if name in active_names
            ]
            if gname == "gripper" and not joints_def:
                self._groups[gname] = NoOpGroup()
                continue
            g = JointGroup(
                name=gname,
                joint_names=joints_def,
                all_joints=self._all_joints,
                motor_map=self._motor_map,
                ctrl_map=self._ctrl_map,
            )
            self._groups[gname] = g
        if "gripper" not in self._groups:
            self._groups["gripper"] = NoOpGroup()

    # ── 属性 ────────────────────────────────────────────────────────────

    @property
    def num_joints(self) -> int:
        return len(self._all_joints)

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self._all_joints]

    @property
    def has_gripper(self) -> bool:
        return not isinstance(self._groups.get("gripper", None), NoOpGroup)

    @property
    def model(self) -> str:
        return self._model

    def __getattr__(self, name: str) -> any:
        if name.startswith("_"):
            raise AttributeError(name)
        if name in self._groups:
            return self._groups[name]
        raise AttributeError(name)

    def disable_all(
        self,
        poll_max: int = 20,
        poll_interval: float = 0.05,
    ) -> None:
        """失能所有活动电机，并通过新鲜反馈确认其处于失能状态。"""
        errors = []
        for jc in self._all_joints:
            motor = self._motor_map.get(jc.name)
            if motor is None:
                errors.append(f"{jc.name}: motor handle unavailable")
                continue
            try:
                motor.disable()
            except Exception as exc:
                errors.append(f"{jc.name}: {exc}")
        time.sleep(0.05)
        for jc in self._all_joints:
            motor = self._motor_map.get(jc.name)
            if motor is None:
                continue
            last_state = None
            last_error = None
            for _ in range(max(1, poll_max)):
                try:
                    last_state = motor.request_fresh_state(timeout_ms=50)
                    if last_state is not None and last_state.status_code == 0:
                        break
                except Exception as exc:
                    last_error = exc
                time.sleep(max(0.0, poll_interval))
            else:
                status = None if last_state is None else last_state.status_code
                detail = (
                    f", last_error={last_error}"
                    if last_error is not None
                    else ""
                )
                errors.append(
                    f"{jc.name}: fresh DISABLED feedback unavailable, "
                    f"status={status}{detail}"
                )
        if errors:
            raise RuntimeError("failed to disable motors: " + "; ".join(errors))

    def clear_errors(
        self,
        joint_names: Optional[List[str]] = None,
        poll_max: int = 20,
        poll_interval: float = 0.05,
    ) -> tuple[str, ...]:
        """清除所选电机故障；尝试处理全部目标后再统一报告错误。"""
        configured_names = {joint.name for joint in self._all_joints}
        selected = set(joint_names or configured_names)
        unknown = selected.difference(configured_names)
        if unknown:
            raise ValueError(f"unknown joints: {', '.join(sorted(unknown))}")
        if not selected:
            raise ValueError("at least one joint must be selected")

        completed: list[str] = []
        errors: list[str] = []
        for joint in self._all_joints:
            if joint.name not in selected:
                continue
            motor = self._motor_map[joint.name]
            try:
                try:
                    motor.disable()
                except Exception:
                    # 故障电机在 clear_error 成功前可能拒绝失能命令。
                    pass
                motor.clear_error()
                time.sleep(0.05)
                motor.disable()
                self._wait_for_healthy_state(
                    joint,
                    poll_max=poll_max,
                    poll_interval=poll_interval,
                )
                completed.append(joint.name)
            except Exception as exc:
                errors.append(f"{joint.name}: {exc}")

        if errors:
            raise RuntimeError(
                "failed to clear motor faults; "
                f"cleared={completed}: {'; '.join(errors)}"
            )
        return tuple(completed)

    # ── 零点 ────────────────────────────────────────────────────────────

    def set_zero(
        self,
        joint_names: Optional[List[str]] = None,
        poll_max: int = 20,
        poll_interval: float = 0.05,
        verify_tolerance: float = 0.02,
        verify_velocity: float = 0.05,
        verify_samples: int = 3,
    ) -> tuple[str, ...]:
        """将所选电机的当前位置设为零点，并通过新鲜反馈验证。

        ``joint_names`` 为空时处理当前启用的全部电机，包括已启用的夹爪。
        """
        if verify_samples < 1:
            raise ValueError("verify_samples must be at least 1")
        if not np.isfinite(verify_tolerance) or verify_tolerance < 0.0:
            raise ValueError("verify_tolerance must be finite and non-negative")
        if not np.isfinite(verify_velocity) or verify_velocity < 0.0:
            raise ValueError("verify_velocity must be finite and non-negative")

        self.disable_all(
            poll_max=poll_max,
            poll_interval=poll_interval,
        )
        time.sleep(0.3)

        selected = set(joint_names or [joint.name for joint in self._all_joints])
        unknown = selected.difference(joint.name for joint in self._all_joints)
        if unknown:
            raise ValueError(f"unknown joints: {', '.join(sorted(unknown))}")
        targets = [joint for joint in self._all_joints if joint.name in selected]
        if not targets:
            raise ValueError("at least one joint must be selected for zeroing")

        # 此操作并非原子操作，因此开始前先校验全部目标，并保留写零前状态，
        # 以便给出有意义的验证错误。
        before_states = {}
        for jc in targets:
            before_states[jc.name] = self._wait_for_healthy_state(
                jc,
                poll_max,
                poll_interval,
            )

        completed: list[str] = []
        for jc in targets:
            motor = self._motor_map[jc.name]
            try:
                motor.set_zero_position()
            except CallError as exc:
                raise RuntimeError(
                    f"zeroing failed for {jc.name}; completed={completed}: {exc}"
                ) from exc
            time.sleep(0.1)
            before = before_states[jc.name]
            for sample_index in range(1, verify_samples + 1):
                try:
                    state = motor.request_fresh_state(timeout_ms=50)
                except Exception as exc:
                    raise RuntimeError(
                        f"zero verification failed for {jc.name} at fresh sample "
                        f"{sample_index}/{verify_samples}: feedback unavailable; "
                        f"completed={completed}: {exc}"
                    ) from exc
                if state is None:
                    raise RuntimeError(
                        f"zero verification failed for {jc.name} at fresh sample "
                        f"{sample_index}/{verify_samples}: feedback unavailable; "
                        f"completed={completed}"
                    )
                if state.status_code != 0:
                    raise RuntimeError(
                        f"zero verification failed for {jc.name} at fresh sample "
                        f"{sample_index}/{verify_samples}: "
                        f"motor status={state.status_code}; completed={completed}"
                    )
                position = float(state.pos)
                _, velocity_feedback_scale = _velocity_range_scales(jc)
                velocity = float(state.vel) * velocity_feedback_scale
                if (
                    abs(position) > verify_tolerance
                    or abs(velocity) > verify_velocity
                ):
                    raise RuntimeError(
                        f"zero verification failed for {jc.name} at fresh sample "
                        f"{sample_index}/{verify_samples}: "
                        f"before_position={float(before.pos):+.6f} rad, "
                        f"position={position:+.6f} rad "
                        f"(limit {verify_tolerance:.6f}), "
                        f"velocity={velocity:+.6f} rad/s "
                        f"(limit {verify_velocity:.6f}); completed={completed}"
                    )
            completed.append(jc.name)
        return tuple(completed)

    def _wait_for_healthy_state(
        self,
        jc: JointCfg,
        poll_max: int,
        poll_interval: float,
    ):
        motor = self._motor_map[jc.name]
        last_state = None
        last_error = None
        for _ in range(max(1, poll_max)):
            try:
                last_state = motor.request_fresh_state(timeout_ms=50)
                if last_state is not None and last_state.status_code == 0:
                    return last_state
            except Exception as exc:
                last_error = exc
            time.sleep(max(0.0, poll_interval))
        status = None if last_state is None else last_state.status_code
        detail = f", last_error={last_error}" if last_error is not None else ""
        raise RuntimeError(
            f"{jc.name}: healthy feedback unavailable, status={status}{detail}"
        )

    # ── 全局状态读取 ───────────────────────────────────────────────────

    def get_state(
        self,
        request_feedback: bool = True,
        require_complete: bool = False,
        joint_names: Optional[List[str]] = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        joints_by_name = {joint.name: joint for joint in self._all_joints}
        if joint_names is None:
            selected_joints = self._all_joints
        else:
            unknown = set(joint_names).difference(joints_by_name)
            if unknown:
                raise ValueError(f"unknown joints: {', '.join(sorted(unknown))}")
            selected_joints = [joints_by_name[name] for name in joint_names]

        feedback_errors = []
        if request_feedback:
            attempts = _COMPLETE_FEEDBACK_ATTEMPTS if require_complete else 1
            for _ in range(attempts):
                feedback_errors = []
                for ctrl in self._ctrl_map.values():
                    try:
                        ctrl.request_feedback_all(timeout_ms=50)
                    except CallError as exc:
                        feedback_errors.append(exc)
                if not feedback_errors:
                    break
            if require_complete and feedback_errors:
                error = _complete_feedback_error(
                    feedback_errors,
                    attempts=attempts,
                    motor_names=tuple(joint.name for joint in selected_joints),
                    motor_names_by_id={
                        joint.motor_id: joint.name for joint in selected_joints
                    },
                    transport=getattr(self, "_transport", "unknown"),
                    channel=getattr(self, "_channel", "unknown"),
                )
                raise error from feedback_errors[-1]
        pos, vel, torq = [], [], []
        for jc in selected_joints:
            try:
                st = self._motor_map[jc.name].get_state()
            except CallError as exc:
                raise FeedbackTimeoutError(
                    f"{jc.name}: cached feedback read failed: {exc}",
                    operation="read_feedback",
                    motor_names=(jc.name,),
                    retryable=True,
                ) from exc
            if st is not None:
                if st.status_code not in _HEALTHY_DAMIAO_STATUS_CODES:
                    raise MotorFaultError(
                        f"{jc.name}: motor fault status={st.status_code}",
                        status_codes={jc.name: st.status_code},
                    )
                _, velocity_feedback_scale = _velocity_range_scales(jc)
                _, torque_feedback_scale = _torque_range_scales(jc)
                pos.append(jc.direction * st.pos)
                vel.append(jc.direction * st.vel * velocity_feedback_scale)
                torq.append(jc.direction * st.torq * torque_feedback_scale)
            else:
                if require_complete:
                    raise IncompleteFeedbackError(
                        f"{jc.name}: no motor feedback",
                        operation="read_feedback",
                        motor_names=(jc.name,),
                        retryable=True,
                    )
                pos.append(0.0)
                vel.append(0.0)
                torq.append(0.0)
        return (
            np.array(pos, dtype=np.float64),
            np.array(vel, dtype=np.float64),
            np.array(torq, dtype=np.float64),
        )

    def get_status_codes(
        self,
        joint_names: Optional[List[str]] = None,
    ) -> dict[str, int]:
        """不请求新帧，直接返回所选关节的最新电机状态。"""
        joints_by_name = {joint.name: joint for joint in self._all_joints}
        if joint_names is None:
            selected_joints = self._all_joints
        else:
            unknown = set(joint_names).difference(joints_by_name)
            if unknown:
                raise ValueError(f"unknown joints: {', '.join(sorted(unknown))}")
            selected_joints = [joints_by_name[name] for name in joint_names]

        statuses: dict[str, int] = {}
        for joint in selected_joints:
            try:
                state = self._motor_map[joint.name].get_state()
            except CallError as exc:
                raise FeedbackTimeoutError(
                    f"{joint.name}: cached status read failed: {exc}",
                    operation="read_feedback_status",
                    motor_names=(joint.name,),
                    retryable=True,
                ) from exc
            if state is None:
                raise IncompleteFeedbackError(
                    f"{joint.name}: no motor feedback",
                    operation="read_feedback_status",
                    motor_names=(joint.name,),
                    retryable=True,
                )
            statuses[joint.name] = int(state.status_code)
        return statuses

    def get_feedback_stats(
        self,
        joint_names: Optional[List[str]] = None,
    ) -> dict[str, Any]:
        """不发送任何帧，直接返回缓存反馈的计数和数据年龄。"""
        joints_by_name = {joint.name: joint for joint in self._all_joints}
        if joint_names is None:
            selected_joints = self._all_joints
        else:
            unknown = set(joint_names).difference(joints_by_name)
            if unknown:
                raise ValueError(f"unknown joints: {', '.join(sorted(unknown))}")
            selected_joints = [joints_by_name[name] for name in joint_names]
        result: dict[str, Any] = {}
        for joint in selected_joints:
            try:
                result[joint.name] = self._motor_map[joint.name].get_feedback_stats()
            except CallError as exc:
                raise FeedbackTimeoutError(
                    f"{joint.name}: feedback statistics unavailable: {exc}",
                    operation="read_feedback_stats",
                    motor_names=(joint.name,),
                    retryable=True,
                ) from exc
        return result

    # ── 生命周期 ────────────────────────────────────────────────────────

    def disconnect(self, *, disable: bool = True) -> None:
        """关闭总线，并可选择先失能电机。

        ``disable=False`` 仅适用于从未使能或控制电机的客户端，例如只读诊断工具。
        运动控制代码应保留默认值，以便尝试并验证物理失能。
        """
        if not self._connected:
            return
        errors = []
        if disable:
            try:
                self.disable_all()
            except Exception as exc:
                errors.append(str(exc))
        time.sleep(0.1)
        for ctrl in self._ctrl_map.values():
            try:
                ctrl.close_bus()
            except Exception as exc:
                errors.append(f"controller close_bus: {exc}")
            try:
                ctrl.close()
            except Exception as exc:
                errors.append(f"controller close: {exc}")
        self._ctrl_map.clear()
        self._motor_map.clear()
        self._connected = False
        if errors:
            raise RuntimeError("disconnect completed with errors: " + "; ".join(errors))

    def estop(self) -> None:
        self.disable_all()

    # ── 上下文管理器 ───────────────────────────────────────────────────────

    def __enter__(self) -> "ArxDCan":
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        gs = ", ".join(f"{k}({g.num_joints}j)" for k, g in self._groups.items())
        return f"ArxDCan({self._name!r}, [{gs}], rate={self._rate}Hz)"
