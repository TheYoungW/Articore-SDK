from __future__ import annotations

import argparse
import time

from arx_d_can import ArxDCanDualArm


ROLES = ("l-joint4", "r-joint4")


def selected(state) -> tuple[bool | None, bool | None]:
    return state.left.arm.enabled[3], state.right.arm.enabled[3]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify cached per-motor power feedback without motion commands."
    )
    parser.add_argument(
        "--i-understand-motors-will-enable",
        action="store_true",
        help="required acknowledgement for enabling both joint4 motors",
    )
    args = parser.parse_args()
    if not args.i_understand_motors_will_enable:
        parser.error("--i-understand-motors-will-enable is required")

    robot = ArxDCanDualArm(control_mode="mit", with_grippers=True)
    connected = False
    try:
        robot.connect()
        connected = True
        before = robot.read_state()
        print("before", selected(before), before.left.arm.enabled, before.right.arm.enabled)
        if selected(before) != (False, False):
            raise RuntimeError("joint4 motors must start feedback-confirmed disabled")

        if not robot.enable(motors=ROLES):
            raise RuntimeError("subset enable returned false")
        time.sleep(0.1)
        enabled = robot.read_state()
        print("enabled", selected(enabled), enabled.timestamp_ns, enabled.sequence)
        if selected(enabled) != (True, True):
            raise RuntimeError("joint4 enable feedback was not confirmed in state v2")

        if not robot.disable(motors=ROLES):
            raise RuntimeError("subset disable returned false")
        time.sleep(0.1)
        disabled = robot.read_state()
        print("disabled", selected(disabled), disabled.timestamp_ns, disabled.sequence)
        if selected(disabled) != (False, False):
            raise RuntimeError("joint4 disable feedback was not confirmed in state v2")
        health = robot.get_health()
        print("health", health.state, health.fault_reason)
        return 0
    finally:
        if connected:
            try:
                robot.disable(motors=ROLES)
            except Exception as error:
                print("cleanup subset disable failed:", error)
            robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
