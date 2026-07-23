"""ARX-D-CAN 分组控制系统 — JointGroup 架构。

配置驱动的硬件抽象层：
  - config/models.yaml 注册内置机型，每种机械臂使用独立硬件 YAML
  - 关节按 groups 分组，每组独立控制模式
  - 统一 loop 中按组顺序同步发送，防止总线争用

使用示例::

    # arm 组 POS_VEL，gripper 组 MIT（解耦混合控制）
    arm = ArxDCan()
    arm.connect()
    arm.arm.enable()
    arm.gripper.enable()
    arm.arm.mode_pos_vel()
    arm.gripper.mode_mit()

    def loop(ref, dt):
        ref.arm.send_pos_vel(joint_pos)
        ref.gripper.send_mit(gripper_pos)

    arm.start_control_loop(loop)

    # 全部组 MIT（纯测试）
    arm.arm.mode_mit()
    arm.gripper.mode_mit()

    arm.disconnect()
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np
import yaml

from ..driver import CallError, Controller, Mode

_CFG_DIR = Path(__file__).resolve().parents[1] / "config"
_MODEL_REGISTRY = _CFG_DIR / "models.yaml"
_HEALTHY_DAMIAO_STATUS_CODES = frozenset((0x0, 0x1))  # disabled, enabled
_COMPLETE_FEEDBACK_ATTEMPTS = 2


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
    """Return the built-in model profile names in deterministic order."""
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
        hw_yaml = str(models[selected_model])

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
    kp: float = 0.0
    kd: float = 0.0
    vel_kp: float = 0.0
    vel_ki: float = 0.0
    pos_kp: float = 0.0
    pos_ki: float = 0.0
    vlim: float = 0.0


def _finite(value: Any, *, field: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{field} must be finite")
    return parsed


def _resolve_urdf_path(hw_path: Path, value: Any) -> Path | None:
    if value in (None, ""):
        return None
    path = Path(str(value)).expanduser()
    candidates = [path] if path.is_absolute() else [hw_path.parent / path, _CFG_DIR.parent / path]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(f"URDF not found for {hw_path}: {value}")


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
    """Load and validate one model profile.

    ``hw_yaml`` remains the backward-compatible positional custom-config
    argument. New code may select a built-in ``model`` instead.
    """
    hw_path, selected_model = _resolve_hw_cfg_path(hw_yaml, model=model)
    data = _read_yaml_mapping(hw_path, description="hardware config")

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
            raise ValueError(f"joints[{index}] MIT and POS_VEL must be mappings")
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
        joint = JointCfg(
            name=name,
            motor_id=motor_id,
            feedback_id=feedback_id,
            model=str(j.get("model", "4340P")),
            direction=_finite(
                j.get("direction", 1.0),
                field=f"{name}.direction",
            ),
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
        if joint.vlim <= 0.0:
            raise ValueError(f"{name}.POS_VEL.vlim must be positive")
        joints.append(joint)

    groups = _validate_groups(data.get("groups"), joints, path=hw_path)
    urdf_path = _resolve_urdf_path(hw_path, data.get("urdf_path"))
    end_effector_frame = str(
        data.get("end_effector_frame", "gripper_end")
    ).strip()
    channel = str(data.get("channel", "/dev/ttyACM0")).strip()
    baud = int(data.get("baud", 1_000_000))
    rate = _finite(data.get("rate", 500.0), field="rate")
    if not channel:
        raise ValueError("channel must not be empty")
    if baud <= 0:
        raise ValueError("baud must be positive")
    if rate <= 0.0:
        raise ValueError("rate must be positive")
    if not end_effector_frame:
        raise ValueError("end_effector_frame must not be empty")

    return {
        "name": data.get("name", "ARX-D-CAN"),
        "model": selected_model or str(data.get("name", hw_path.stem)),
        "hardware_path": str(hw_path),
        "urdf_path": None if urdf_path is None else str(urdf_path),
        "end_effector_frame": end_effector_frame,
        "channel": channel,
        "baud": baud,
        "rate": rate,
        "groups": groups,
        "joints": joints,
        "gripper_mapping": _optional_mapping(data, "gripper_mapping"),
        "gripper_force_control": _optional_mapping(data, "gripper_force_control"),
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

    def mode_vel(self) -> bool:
        self._mode = "vel"
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

    def send_vel(self, vel) -> None:
        pass

    def get_positions(self) -> np.ndarray:
        return np.array([], dtype=np.float64)

    def get_velocities(self) -> np.ndarray:
        return np.array([], dtype=np.float64)

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

    # ── 使能 / 失能 ────────────────────────────────────────────────────

    def enable(
        self,
        poll_max: int = 20,
        poll_interval: float = 0.05,
    ) -> None:
        """Enable every motor and verify that each reports ENABLED."""
        for jc in self._jcfgs:
            self._mm[jc.name].enable()
        time.sleep(0.05)

        errors = []
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

    def disable(self) -> None:
        errors = []
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].disable()
            except Exception as exc:
                errors.append(f"{jc.name}: {exc}")
        time.sleep(0.05)
        if errors:
            raise RuntimeError("failed to disable motors: " + "; ".join(errors))

    def clear_errors(
        self,
        poll_max: int = 20,
        poll_interval: float = 0.05,
    ) -> tuple[str, ...]:
        """Clear every motor fault in this group and leave all motors disabled."""
        completed: list[str] = []
        errors: list[str] = []
        for jc in self._jcfgs:
            motor = self._mm[jc.name]
            try:
                try:
                    motor.disable()
                except Exception:
                    # A faulted motor may reject disable until clear_error succeeds.
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
            f"disabled feedback unavailable after clear_error, status={status}{detail}"
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
        self._mode = "mit"
        if kp is not None:
            self._mit_kp = np.asarray(kp, dtype=np.float64).reshape(-1)
        if kd is not None:
            self._mit_kd = np.asarray(kd, dtype=np.float64).reshape(-1)
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

    def mode_vel(self) -> bool:
        self._mode = "vel"
        ok = True
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].ensure_mode(Mode.VEL, 1000)
            except CallError as e:
                print(f"[{self.name}/mode_vel/{jc.name}] {e}")
                ok = False
            time.sleep(0.05)
        time.sleep(0.2)
        return ok

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
        n = self.num_joints
        pos = np.asarray(pos, dtype=np.float64).reshape(-1)
        if vel is None:
            vel = np.zeros(n)
        if tau is None:
            tau = np.zeros(n)
        if kp is None:
            kp = self._mit_kp
        if kd is None:
            kd = self._mit_kd

        for i, jc in enumerate(self._jcfgs):
            try:
                self._mm[jc.name].send_mit(
                    jc.direction * float(pos[i]),
                    jc.direction * float(vel[i]),
                    float(kp[i]),
                    float(kd[i]),
                    jc.direction * float(tau[i]),
                )
            except CallError:
                if strict:
                    raise

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
                self._mm[self._jcfgs[i].name].send_pos_vel(
                    self._jcfgs[i].direction * float(pos[i]),
                    float(vlim[i]),
                )
            except CallError:
                if strict:
                    raise

    # ── VEL 发送 ───────────────────────────────────────────────────────

    def send_vel(self, vel: np.ndarray, *, strict: bool = True) -> None:
        vel = np.asarray(vel, dtype=np.float64).reshape(-1)
        for i in range(min(len(vel), self.num_joints)):
            try:
                jc = self._jcfgs[i]
                self._mm[jc.name].send_vel(jc.direction * float(vel[i]))
            except CallError:
                if strict:
                    raise

    # ── 状态读取 ───────────────────────────────────────────────────────

    def _request_feedback(self) -> None:
        self._cm["main"].request_feedback_all(timeout_ms=50)

    def read_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Read this group's state and fail if fresh, healthy feedback is unavailable."""
        if request_feedback:
            self._cm["main"].request_feedback_all(timeout_ms=50)

        positions, velocities, torques = [], [], []
        for jc in self._jcfgs:
            state = self._mm[jc.name].get_state()
            if state is None:
                raise RuntimeError(f"{self.name}/{jc.name}: no fresh feedback")
            if getattr(state, "status_code", 0) not in _HEALTHY_DAMIAO_STATUS_CODES:
                raise RuntimeError(
                    f"{self.name}/{jc.name}: motor fault status={state.status_code}"
                )
            positions.append(jc.direction * state.pos)
            velocities.append(jc.direction * state.vel)
            torques.append(jc.direction * state.torq)
        return (
            np.asarray(positions, dtype=np.float64),
            np.asarray(velocities, dtype=np.float64),
            np.asarray(torques, dtype=np.float64),
        )

    def get_positions(self, request_feedback: bool = True) -> np.ndarray:
        if request_feedback:
            self._request_feedback()
        return np.array([
            jc.direction * self._mm[jc.name].get_state().pos
            if self._mm[jc.name].get_state() is not None else 0.0
            for jc in self._jcfgs
        ], dtype=np.float64)

    def get_velocities(self, request_feedback: bool = True) -> np.ndarray:
        if request_feedback:
            self._request_feedback()
        return np.array([
            jc.direction * self._mm[jc.name].get_state().vel
            if self._mm[jc.name].get_state() is not None else 0.0
            for jc in self._jcfgs
        ], dtype=np.float64)

    def __repr__(self) -> str:
        return f"JointGroup({self.name!r}, joints={self.num_joints}, mode={self._mode})"


# --------------------------------------------------------------------------
# ArxDCan — 分组控制器容器
# --------------------------------------------------------------------------

class ArxDCan:
    """ARX-D-CAN 分组控制系统。

    持有多个 JointGroup，每组独立控制模式，独立发送命令，
    在同一个控制循环中按组顺序同步发送，防止总线争用。

    按组访问（通过 __getattr__）::

        arm.arm       # 机械臂关节组
        arm.gripper   # 夹爪关节组（如果有）

    也可以通过 groups 字典::

        arm.groups["arm"]
        arm.groups["gripper"]

    手动添加组::

        arm.add_group("custom", ["joint1", "joint2"])
    """

    def __init__(
        self,
        hw_yaml: str | Path | None = None,
        channel: str | None = None,
        baud: int | None = None,
        joint_names: Optional[List[str]] = None,
        *,
        model: str | None = None,
        config_data: dict[str, Any] | None = None,
    ) -> None:
        if config_data is not None and (hw_yaml is not None or model is not None):
            raise ValueError("config_data cannot be combined with model or config_path")
        cfg = dict(config_data) if config_data is not None else load_cfg(hw_yaml, model=model)
        hardware_path = cfg.get("hardware_path")
        self._hw_yaml = (
            Path(str(hardware_path)).name
            if hardware_path
            else f"{cfg.get('model', 'custom')}.yaml"
        )
        self._model: str = str(cfg.get("model", "custom"))
        self._urdf_path: str | None = cfg.get("urdf_path")
        self._end_effector_frame: str = str(
            cfg.get("end_effector_frame", "gripper_end")
        )
        if channel:
            cfg["channel"] = channel

        self._name: str = cfg["name"]
        self._channel: str = cfg["channel"]
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

        self._running = False
        self._ctrl_thread: Optional[threading.Thread] = None
        self._ctrl_fn: Optional[Callable] = None
        self._ctrl_rate: float = self._rate
        self._connected: bool = False

        self._build_groups()

    def connect(self) -> None:
        """连接总线、注册电机。模式切换需在 connect 后调用。"""
        if self._connected:
            return
        self._setup_motors()
        self._connected = True

    def _make_controller(self) -> Controller:
        if self._channel.startswith("/dev/tty"):
            return Controller.from_dm_serial(self._channel, self._baud)
        return Controller(self._channel)

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
    def groups(self) -> Dict[str, JointGroup]:
        return self._groups

    @property
    def control_loop_active(self) -> bool:
        t = getattr(self, "_ctrl_thread", None)
        return t is not None and t.is_alive()

    @property
    def rate(self) -> float:
        return self._ctrl_rate

    @property
    def has_gripper(self) -> bool:
        return not isinstance(self._groups.get("gripper", None), NoOpGroup)

    @property
    def hardware_yaml(self) -> str:
        return self._hw_yaml

    @property
    def model(self) -> str:
        return self._model

    @property
    def urdf_path(self) -> str | None:
        return self._urdf_path

    @property
    def end_effector_frame(self) -> str:
        return self._end_effector_frame

    def __getattr__(self, name: str) -> any:
        if name.startswith("_"):
            raise AttributeError(name)
        if name in self._groups:
            return self._groups[name]
        raise AttributeError(name)

    # ── 手动添加组 ────────────────────────────────────────────────────

    def add_group(self, name: str, joint_names: List[str]) -> JointGroup:
        if name in self._groups:
            raise ValueError(f"组 {name!r} 已存在")
        g = JointGroup(
            name=name,
            joint_names=joint_names,
            all_joints=self._all_joints,
            motor_map=self._motor_map,
            ctrl_map=self._ctrl_map,
        )
        self._groups[name] = g
        return g

    # ── 全局使能 / 失能 ────────────────────────────────────────────────

    def enable_all(self) -> None:
        enabled = []
        try:
            for jc in self._all_joints:
                self._motor_map[jc.name].enable()
                enabled.append(jc.name)
        except Exception as exc:
            try:
                self.disable_all()
            except Exception:
                pass
            raise RuntimeError(
                f"failed to enable {jc.name}; enabled before failure={enabled}: {exc}"
            ) from exc
        time.sleep(0.05)

    def disable_all(self) -> None:
        errors = []
        for jc in self._all_joints:
            motor = self._motor_map.get(jc.name)
            if motor is None:
                continue
            try:
                motor.disable()
            except Exception as exc:
                errors.append(f"{jc.name}: {exc}")
        time.sleep(0.05)
        if errors:
            raise RuntimeError("failed to disable motors: " + "; ".join(errors))

    def clear_errors(
        self,
        joint_names: Optional[List[str]] = None,
        poll_max: int = 20,
        poll_interval: float = 0.05,
    ) -> tuple[str, ...]:
        """Clear selected motor faults, attempting every motor before reporting errors."""
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
                    # A faulted motor may reject disable until clear_error succeeds.
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
        """Set selected motor positions to zero and verify fresh feedback."""
        if verify_samples < 1:
            raise ValueError("verify_samples must be at least 1")
        if not np.isfinite(verify_tolerance) or verify_tolerance < 0.0:
            raise ValueError("verify_tolerance must be finite and non-negative")
        if not np.isfinite(verify_velocity) or verify_velocity < 0.0:
            raise ValueError("verify_velocity must be finite and non-negative")

        self.disable_all()
        time.sleep(0.3)

        selected = set(joint_names or [joint.name for joint in self._all_joints])
        unknown = selected.difference(joint.name for joint in self._all_joints)
        if unknown:
            raise ValueError(f"unknown joints: {', '.join(sorted(unknown))}")
        targets = [joint for joint in self._all_joints if joint.name in selected]
        if not targets:
            raise ValueError("at least one joint must be selected for zeroing")

        # Validate every target before starting this non-atomic operation and
        # retain the pre-zero state for meaningful verification errors.
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
                velocity = float(state.vel)
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
                    except Exception as exc:
                        feedback_errors.append(str(exc))
                if not feedback_errors:
                    break
            if require_complete and feedback_errors:
                raise RuntimeError(
                    f"fresh feedback failed after {attempts} attempts: "
                    + "; ".join(feedback_errors)
                )
        pos, vel, torq = [], [], []
        for jc in selected_joints:
            st = self._motor_map[jc.name].get_state()
            if st is not None:
                if st.status_code not in _HEALTHY_DAMIAO_STATUS_CODES:
                    raise RuntimeError(
                        f"{jc.name}: motor fault status={st.status_code}"
                    )
                pos.append(jc.direction * st.pos)
                vel.append(jc.direction * st.vel)
                torq.append(jc.direction * st.torq)
            else:
                if require_complete:
                    raise RuntimeError(f"{jc.name}: no motor feedback")
                pos.append(0.0)
                vel.append(0.0)
                torq.append(0.0)
        return (
            np.array(pos, dtype=np.float64),
            np.array(vel, dtype=np.float64),
            np.array(torq, dtype=np.float64),
        )

    def get_positions(self) -> np.ndarray:
        return self.get_state()[0]

    def get_velocities(self) -> np.ndarray:
        return self.get_state()[1]

    def get_torques(self) -> np.ndarray:
        return self.get_state()[2]

    # ── 生命周期 ────────────────────────────────────────────────────────

    def disconnect(self) -> None:
        if not self._connected:
            return
        self.stop_control_loop()
        errors = []
        try:
            self.disable_all()
        except Exception as exc:
            errors.append(str(exc))
        time.sleep(0.1)
        for ctrl in self._ctrl_map.values():
            try:
                ctrl.shutdown()
            except Exception as exc:
                errors.append(f"controller shutdown: {exc}")
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

    def reconnect(
        self,
        init_delay: float = 1.0,
        post_setup_delay: float = 0.5,
    ) -> None:
        self.disconnect()
        time.sleep(init_delay)
        self._ctrl_map["main"] = self._make_controller()
        self._motor_map.clear()
        ctrl = self._ctrl_map["main"]
        for jc in self._all_joints:
            mot = ctrl.add_damiao_motor(jc.motor_id, jc.feedback_id, jc.model)
            self._motor_map[jc.name] = mot
            time.sleep(0.05)
        self._build_groups()
        time.sleep(post_setup_delay)
        print("[reconnect] 控制器和电机已重新初始化")

    # ── 控制循环 ────────────────────────────────────────────────────────

    def start_control_loop(
        self,
        control_fn: Callable[["ArxDCan", float], None],
        rate: Optional[float] = None,
    ) -> None:
        if self.control_loop_active:
            raise RuntimeError("控制循环已在运行，请先调用 stop_control_loop()")
        self._running = True
        self._ctrl_rate = rate if rate is not None else self._rate
        self._ctrl_fn = control_fn
        self._ctrl_thread = threading.Thread(
            target=self._control_loop_impl,
            name="arx_d_can-control-loop",
            daemon=True,
        )
        self._ctrl_thread.start()

    def _control_loop_impl(self) -> None:
        dt = 1.0 / self._ctrl_rate
        while self._running:
            t0 = time.perf_counter()
            try:
                self._ctrl_fn(self, dt)
            except Exception:
                if self._running:
                    self._running = False
                    try:
                        self.estop()
                    finally:
                        raise
            elapsed = time.perf_counter() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop_control_loop(self) -> None:
        self._running = False
        t = getattr(self, "_ctrl_thread", None)
        if t is not None and t.is_alive():
            t.join(timeout=5.0)

    # ── 上下文管理器 ───────────────────────────────────────────────────────

    def __enter__(self) -> "ArxDCan":
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        gs = ", ".join(f"{k}({g.num_joints}j)" for k, g in self._groups.items())
        return f"ArxDCan({self._name!r}, [{gs}], rate={self._ctrl_rate}Hz)"
