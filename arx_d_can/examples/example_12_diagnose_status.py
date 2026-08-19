#!/usr/bin/env python3
"""示例 12：通过 Runtime health 查看双臂通信和具体故障。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def _transport(label: str, value) -> None:
    print(
        f"{label}: connected={value.connected}, healthy={value.healthy}, "
        f"tx={value.tx_frames}, rx={value.rx_frames}, "
        f"send_errors={value.send_errors}, receive_errors={value.receive_errors}"
    )
    if value.last_error:
        print(f"  最近错误：{value.last_error}")


def main() -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    try:
        health = robot.safety_health
        print(f"Runtime 状态：{health.state.name}")
        _transport("左通道", health.left_transport)
        _transport("右通道", health.right_transport)
        if health.motor_faults:
            print("电机故障：", ", ".join(health.motor_faults))
        if health.unconfirmed_disable:
            print("未确认失能：", ", ".join(health.unconfirmed_disable))
        if health.operation_failed_motors:
            print("最近操作失败电机：", ", ".join(health.operation_failed_motors))
        error = health.last_operation_error or health.fault_reason or health.safety_reason
        print("具体错误：", error or "无")
        print(f"最近 CAN 帧率：{robot.get_fps():.0f} Hz")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
