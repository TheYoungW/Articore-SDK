"""ARX-D-CAN 电机诊断数据和只读查询。"""
from __future__ import annotations

from dataclasses import dataclass

from .actuator.arx_d_can import _torque_range_scales, _velocity_range_scales


CTRL_MODE_REGISTER = 10

STATUS_NAMES = {
    0x0: "DISABLED",
    0x1: "ENABLED",
    0x2: "MOTOR_ENCODER_NOT_RECOGNIZED",
    0x3: "OUTPUT_ENCODER_NOT_RECOGNIZED",
    0x5: "ENCODER_READ_ERROR",
    0x6: "MOTOR_PARAMETER_READ_ERROR",
    0x8: "OVER_VOLTAGE",
    0x9: "UNDER_VOLTAGE",
    0xA: "OVER_CURRENT",
    0xB: "MOS_OVER_TEMPERATURE",
    0xC: "COIL_OVER_TEMPERATURE",
    0xD: "COMMUNICATION_LOST",
    0xE: "OVERLOAD",
}

MODE_NAMES = {1: "MIT", 2: "POS_VEL"}


@dataclass(frozen=True, slots=True)
class MotorDiagnostic:
    name: str
    motor_id: int
    feedback_id: int
    status_code: int | None = None
    control_mode: int | None = None
    position: float | None = None
    velocity: float | None = None
    torque: float | None = None
    mos_temperature: float | None = None
    rotor_temperature: float | None = None
    error: str | None = None

    @property
    def status(self) -> str:
        return status_name(self.status_code)

    @property
    def mode(self) -> str:
        return mode_name(self.control_mode)


def status_name(code: int | None) -> str:
    if code is None:
        return "NO_FEEDBACK"
    return STATUS_NAMES.get(code, "UNKNOWN")


def mode_name(mode: int | None) -> str:
    if mode is None:
        return "UNAVAILABLE"
    return MODE_NAMES.get(mode, "UNSUPPORTED")


def read_diagnostics(controller, motors, *, timeout_ms: int) -> list[MotorDiagnostic]:
    """读取已经注册的电机，不执行使能、失能或模式切换。"""
    feedback_error = None
    try:
        controller.request_feedback_all(timeout_ms=timeout_ms)
    except Exception as exc:
        feedback_error = str(exc)

    results = []
    for joint, motor in motors:
        try:
            state = motor.get_state()
            if state is None:
                raise RuntimeError(feedback_error or "no motor feedback")
            control_mode = motor.get_register_u32(
                CTRL_MODE_REGISTER,
                timeout_ms=timeout_ms,
            )
            _, velocity_scale = _velocity_range_scales(joint)
            _, torque_scale = _torque_range_scales(joint)
            results.append(
                MotorDiagnostic(
                    name=joint.name,
                    motor_id=joint.motor_id,
                    feedback_id=joint.feedback_id,
                    status_code=int(state.status_code),
                    control_mode=int(control_mode),
                    position=joint.direction * float(state.pos),
                    velocity=joint.direction * float(state.vel) * velocity_scale,
                    torque=joint.direction * float(state.torq) * torque_scale,
                    mos_temperature=float(state.t_mos),
                    rotor_temperature=float(state.t_rotor),
                )
            )
        except Exception as exc:
            results.append(
                MotorDiagnostic(
                    name=joint.name,
                    motor_id=joint.motor_id,
                    feedback_id=joint.feedback_id,
                    error=str(exc),
                )
            )
    return results


def read_motor_diagnostics(arm, *, timeout_ms: int = 100) -> list[MotorDiagnostic]:
    """通过高层机械臂对象读取所有活动电机的结构化诊断信息。"""
    if not arm.connected:
        raise RuntimeError("ARX-D-CAN arm is not connected")
    active_names = set(arm._active_joint_names())
    joints = [
        joint for joint in arm.robot._all_joints if joint.name in active_names
    ]
    motors = [(joint, arm.robot._motor_map[joint.name]) for joint in joints]
    with arm._io_lock:
        return read_diagnostics(
            arm.robot._ctrl_map["main"],
            motors,
            timeout_ms=timeout_ms,
        )


def print_diagnostic_summary(
    diagnostics: list[MotorDiagnostic],
    *,
    temperature_warning: float,
) -> None:
    """打印故障、温度和反馈完整性的简短汇总。"""
    unreadable = [item.name for item in diagnostics if item.error is not None]
    faults = [
        item
        for item in diagnostics
        if item.error is None and item.status_code not in (0x0, 0x1)
    ]
    hot = [
        item
        for item in diagnostics
        if item.error is None
        and max(item.mos_temperature or 0.0, item.rotor_temperature or 0.0)
        >= temperature_warning
    ]

    if faults:
        print(
            "故障：",
            ", ".join(
                f"{item.name}=0x{item.status_code:X}({item.status})"
                for item in faults
            ),
        )
    if hot:
        print(
            "温度警告：",
            ", ".join(
                f"{item.name}(mos={item.mos_temperature:.0f}C,rotor={item.rotor_temperature:.0f}C)"
                for item in hot
            ),
        )
    if unreadable:
        print("反馈不完整：", ", ".join(unreadable))
    if faults or hot or unreadable:
        print("请先检查硬件，不要直接使能或清除故障")


__all__ = [
    "CTRL_MODE_REGISTER",
    "MotorDiagnostic",
    "mode_name",
    "print_diagnostic_summary",
    "read_diagnostics",
    "read_motor_diagnostics",
    "status_name",
]
