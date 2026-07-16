#!/usr/bin/env python3
"""Example 07: change one Damiao motor ESC_ID/MST_ID pair."""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Callable

try:
    from arx_d_can.driver import CallError, Controller
except ModuleNotFoundError:
    from teleop.adapters.arm.arx_d_can.driver import CallError, Controller

try:
    from arx_d_can.examples.common import add_connection_arguments, make_arm
except ModuleNotFoundError:
    from teleop.adapters.arm.arx_d_can.examples.common import (
        add_connection_arguments,
        make_arm,
    )


@dataclass(frozen=True)
class IdChangePlan:
    current_id: int
    current_feedback_id: int
    new_id: int
    new_feedback_id: int


def parse_id(value: str | int) -> int:
    if isinstance(value, int):
        motor_id = value
    else:
        motor_id = int(value, 0)
    if not 0 <= motor_id <= 0x7FF:
        raise ValueError(f"CAN ID out of range: 0x{motor_id:X}")
    return motor_id


def build_id_change_plan(
    *,
    current_id: int,
    new_id: int,
    current_feedback_id: int | None,
    new_feedback_id: int | None,
    feedback_base: int,
) -> IdChangePlan:
    current_id = parse_id(current_id)
    new_id = parse_id(new_id)
    feedback_base = parse_id(feedback_base)
    return IdChangePlan(
        current_id=current_id,
        current_feedback_id=(
            parse_id(current_feedback_id)
            if current_feedback_id is not None
            else parse_id(feedback_base + current_id)
        ),
        new_id=new_id,
        new_feedback_id=(
            parse_id(new_feedback_id)
            if new_feedback_id is not None
            else parse_id(feedback_base + new_id)
        ),
    )


def scan_ids(args: argparse.Namespace) -> list[int]:
    arm = make_arm(args)
    return arm.scan_ids(
        start_id=args.scan_start_id,
        end_id=args.scan_end_id,
        model=args.model,
        timeout_ms=args.timeout_ms,
        feedback_base=f"0x{parse_id(args.feedback_base):X}",
    )


def print_scan(label: str, ids: list[int]) -> None:
    print(f"{label}:", " ".join(f"0x{motor_id:02X}" for motor_id in ids) or "none")


def make_controller(port: str, baud: int) -> Controller:
    if port.startswith("/dev/tty"):
        return Controller.from_dm_serial(port, baud)
    return Controller(port)


ControllerFactory = Callable[[str, int], Controller]


def close_controller(ctrl: Controller, motor) -> None:
    try:
        motor.close()
    finally:
        ctrl.close_bus()
        ctrl.close()


def write_register_u32_with_warning(motor, rid: int, value: int, label: str) -> None:
    try:
        motor.write_register_u32(rid, value)
    except CallError as exc:
        print(f"warning: {label} write ack timeout or failed after send: {exc}")
        print("warning: continuing; final register verification will decide whether it really succeeded")


def apply_id_change(
    *,
    port: str,
    baud: int,
    model: str,
    store: bool,
    plan: IdChangePlan,
    controller_factory: ControllerFactory = make_controller,
) -> None:
    active_id = plan.current_id
    active_feedback_id = plan.current_feedback_id

    if plan.current_id != plan.new_id:
        ctrl = controller_factory(port, baud)
        motor = ctrl.add_damiao_motor(active_id, active_feedback_id, model)
        try:
            write_register_u32_with_warning(motor, 8, plan.new_id, "ESC_ID")
            print(f"write ESC_ID receive register 8: 0x{plan.current_id:X} -> 0x{plan.new_id:X}")
        finally:
            close_controller(ctrl, motor)
        active_id = plan.new_id

    if plan.current_feedback_id != plan.new_feedback_id:
        ctrl = controller_factory(port, baud)
        motor = ctrl.add_damiao_motor(active_id, active_feedback_id, model)
        try:
            write_register_u32_with_warning(motor, 7, plan.new_feedback_id, "MST_ID")
            print(
                f"write MST_ID feedback register 7: "
                f"0x{plan.current_feedback_id:X} -> 0x{plan.new_feedback_id:X}"
            )
        finally:
            close_controller(ctrl, motor)
        active_feedback_id = plan.new_feedback_id

    if store:
        ctrl = controller_factory(port, baud)
        motor = ctrl.add_damiao_motor(active_id, active_feedback_id, model)
        try:
            motor.store_parameters()
            print("store_parameters sent")
        finally:
            close_controller(ctrl, motor)


def verify_id_change(args: argparse.Namespace, plan: IdChangePlan) -> None:
    time.sleep(args.verify_delay)
    ctrl = make_controller(args.port, args.baud)
    motor = ctrl.add_damiao_motor(plan.new_id, plan.new_feedback_id, args.model)
    try:
        esc_id = motor.get_register_u32(8, args.timeout_ms)
        mst_id = motor.get_register_u32(7, args.timeout_ms)
        print(f"verify ESC_ID register 8: 0x{esc_id:X}")
        print(f"verify MST_ID register 7: 0x{mst_id:X}")
        if esc_id != plan.new_id or mst_id != plan.new_feedback_id:
            raise RuntimeError(
                f"verify failed: expected ESC_ID=0x{plan.new_id:X}, "
                f"MST_ID=0x{plan.new_feedback_id:X}; got ESC_ID=0x{esc_id:X}, MST_ID=0x{mst_id:X}"
            )
    finally:
        motor.close()
        ctrl.close_bus()
        ctrl.close()


def main(args: argparse.Namespace) -> None:
    plan = build_id_change_plan(
        current_id=parse_id(args.current_id),
        new_id=parse_id(args.new_id),
        current_feedback_id=(
            parse_id(args.current_feedback_id)
            if args.current_feedback_id is not None
            else None
        ),
        new_feedback_id=(
            parse_id(args.new_feedback_id)
            if args.new_feedback_id is not None
            else None
        ),
        feedback_base=parse_id(args.feedback_base),
    )

    print(
        "plan: "
        f"ESC_ID 0x{plan.current_id:X} -> 0x{plan.new_id:X}, "
        f"MST_ID 0x{plan.current_feedback_id:X} -> 0x{plan.new_feedback_id:X}, "
        f"model={args.model}, port={args.port}, baud={args.baud}"
    )
    print("warning: make sure the target motor is the only motor using the current ESC_ID.")

    if not args.skip_scan:
        before = scan_ids(args)
        print_scan("scan before", before)
        if plan.current_id not in before:
            raise RuntimeError(f"current ESC_ID 0x{plan.current_id:X} was not found")
        if plan.new_id in before and plan.new_id != plan.current_id:
            raise RuntimeError(f"new ESC_ID 0x{plan.new_id:X} already exists on this bus")

    if not args.yes:
        print("dry run only; add --yes to write the motor ID")
        return

    apply_id_change(
        port=args.port,
        baud=args.baud,
        model=args.model,
        store=args.store,
        plan=plan,
    )

    if not args.no_verify:
        verify_id_change(args, plan)
        after = scan_ids(args)
        print_scan("scan after", after)
        if plan.new_id not in after:
            raise RuntimeError(f"new ESC_ID 0x{plan.new_id:X} was not found after write")
        print("id change verified")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Change one Damiao motor ID. Defaults are for gripper ESC_ID 1 -> 7."
    )
    parser.add_argument("--current-id", default="0x01", help="Current Damiao ESC_ID/receive ID")
    parser.add_argument("--new-id", default="0x07", help="New Damiao ESC_ID/receive ID")
    parser.add_argument("--current-feedback-id", default=None, help="Current MST_ID; default feedback_base + current_id")
    parser.add_argument("--new-feedback-id", default=None, help="New MST_ID; default feedback_base + new_id")
    parser.add_argument("--feedback-base", default="0x10")
    parser.add_argument("--model", default="4310", help="Damiao motor model for the target motor")
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument("--scan-start-id", type=int, default=1)
    parser.add_argument("--scan-end-id", type=int, default=30)
    parser.add_argument("--skip-scan", action="store_true", help="Skip pre-write scan safety checks")
    parser.add_argument("--no-verify", action="store_true", help="Skip post-write register verification")
    parser.add_argument("--verify-delay", type=float, default=0.2)
    parser.add_argument("--store", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--yes", action="store_true", help="Actually write the new ID")
    add_connection_arguments(parser)
    main(parser.parse_args())
