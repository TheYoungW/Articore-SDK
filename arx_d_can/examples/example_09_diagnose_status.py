#!/usr/bin/env python3
"""Example 09: read ARX-D-CAN motor status, temperatures, and control modes."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import time

from arx_d_can.actuator import JointCfg, load_cfg
from arx_d_can.driver import Controller


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

MODE_NAMES = {
    1: "MIT",
    2: "POS_VEL",
    3: "VEL",
    4: "FORCE_POS",
}


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


def status_name(code: int | None) -> str:
    if code is None:
        return "NO_FEEDBACK"
    return STATUS_NAMES.get(code, "UNKNOWN")


def mode_name(mode: int | None) -> str:
    if mode is None:
        return "UNAVAILABLE"
    return MODE_NAMES.get(mode, "UNKNOWN")


def read_diagnostics(
    controller: Controller,
    motors,
    *,
    timeout_ms: int,
) -> list[MotorDiagnostic]:
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
            results.append(
                MotorDiagnostic(
                    name=joint.name,
                    motor_id=joint.motor_id,
                    feedback_id=joint.feedback_id,
                    status_code=int(state.status_code),
                    control_mode=int(control_mode),
                    position=float(state.pos),
                    velocity=float(state.vel),
                    torque=float(state.torq),
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


def print_diagnostics(sample: int, diagnostics: list[MotorDiagnostic]) -> None:
    print(f"sample {sample}")
    for item in diagnostics:
        prefix = (
            f"  {item.name:<8} id=0x{item.motor_id:02X} "
            f"feedback=0x{item.feedback_id:02X}"
        )
        if item.error is not None:
            print(f"{prefix} ERROR={item.error}")
            continue
        assert item.status_code is not None
        assert item.control_mode is not None
        assert item.position is not None
        assert item.velocity is not None
        assert item.torque is not None
        assert item.mos_temperature is not None
        assert item.rotor_temperature is not None
        print(
            f"{prefix} status=0x{item.status_code:X}({status_name(item.status_code)}) "
            f"mode={item.control_mode}({mode_name(item.control_mode)}) "
            f"pos={math.degrees(item.position):+.2f}deg "
            f"vel={math.degrees(item.velocity):+.2f}deg/s "
            f"tau={item.torque:+.3f} "
            f"mos={item.mos_temperature:.0f}C rotor={item.rotor_temperature:.0f}C"
        )


def print_summary(diagnostics: list[MotorDiagnostic], *, temperature_warning: float) -> None:
    unreadable = [item.name for item in diagnostics if item.error is not None]
    faults = [
        item
        for item in diagnostics
        if item.error is None and item.status_code not in (0x0, 0x1)
    ]
    enabled = [item.name for item in diagnostics if item.status_code == 0x1]
    disabled = [item.name for item in diagnostics if item.status_code == 0x0]
    hot = [
        item
        for item in diagnostics
        if item.error is None
        and max(item.mos_temperature or 0.0, item.rotor_temperature or 0.0)
        >= temperature_warning
    ]

    print("summary")
    print("  enabled :", ", ".join(enabled) or "none")
    print("  disabled:", ", ".join(disabled) or "none")
    if faults:
        print(
            "  faults  :",
            ", ".join(
                f"{item.name}=0x{item.status_code:X}({status_name(item.status_code)})"
                for item in faults
            ),
        )
    else:
        print("  faults  : none")
    if hot:
        print(
            "  WARNING : abnormal temperature: "
            + ", ".join(
                f"{item.name}(mos={item.mos_temperature:.0f}C,rotor={item.rotor_temperature:.0f}C)"
                for item in hot
            )
        )
    if unreadable:
        print("  WARNING : no complete diagnostics:", ", ".join(unreadable))
    if faults or hot or unreadable:
        print("  ACTION  : do not enable; inspect hardware before clearing faults")


def main(args: argparse.Namespace) -> None:
    config = load_cfg(args.config_path, model=args.arm_model)
    joints: list[JointCfg] = list(config["joints"])
    if args.arm_only:
        joints = [joint for joint in joints if joint.name != "gripper"]

    controller = Controller.from_dm_serial(
        args.port or str(config["channel"]),
        args.baud or int(config["baud"]),
    )
    motors = []
    try:
        for joint in joints:
            motor = controller.add_damiao_motor(
                joint.motor_id,
                joint.feedback_id,
                joint.model,
            )
            motors.append((joint, motor))

        latest = []
        for sample in range(1, max(1, args.samples) + 1):
            latest = read_diagnostics(
                controller,
                motors,
                timeout_ms=args.timeout_ms,
            )
            print_diagnostics(sample, latest)
            if sample < max(1, args.samples):
                time.sleep(max(0.0, args.interval))
        print_summary(latest, temperature_warning=args.temperature_warning)
    finally:
        # Read-only diagnostic: do not send enable, disable, mode, fault-clear,
        # or motion frames while closing the local serial handle.
        controller.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Read ARX-D-CAN status, temperature, and CTRL_MODE without enabling "
            "or commanding any motor."
        )
    )
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument("--temperature-warning", type=float, default=80.0)
    parser.add_argument("--arm-only", action="store_true", help="Do not inspect the gripper")
    profile = parser.add_mutually_exclusive_group()
    profile.add_argument("--arm-model", default=None)
    profile.add_argument(
        "--config-path",
        "--hardware-config",
        dest="config_path",
        default=None,
        help="Custom arm hardware YAML",
    )
    parser.add_argument("--port", default=None, help="Override profile USB2CAN serial port")
    parser.add_argument("--baud", type=int, default=None, help="Override profile serial baudrate")
    main(parser.parse_args())
