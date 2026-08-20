#!/usr/bin/env python3
"""控制示例 08：开启 Yunyi 双臂重力补偿。"""
from __future__ import annotations

import argparse
import time

from arx_d_can import (
    ArxDCanDualArm,
    GravityCompensationPhase,
)


def _stop_and_close(robot: ArxDCanDualArm) -> None:
    """平滑退出重力补偿，并确保双臂失能和连接释放。"""
    errors: list[Exception] = []
    if robot.connected and robot.enabled:
        try:
            if (
                robot.gravity_compensation_status.phase
                is not GravityCompensationPhase.INACTIVE
            ):
                robot.stop_gravity_compensation()
                deadline = time.monotonic() + 2.0
                while (
                    robot.gravity_compensation_status.phase
                    is not GravityCompensationPhase.INACTIVE
                ):
                    if time.monotonic() >= deadline:
                        raise RuntimeError("等待 Runtime 退出重力补偿超时")
                    time.sleep(0.02)
        except Exception as exc:
            errors.append(exc)
        try:
            robot.disable()
        except Exception as exc:
            errors.append(exc)
    if robot.connected:
        try:
            robot.disconnect()
        except Exception as exc:
            errors.append(exc)
    if errors:
        raise RuntimeError(f"双臂重力补偿清理失败：{errors[0]}") from errors[0]


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    print("请托稳双臂，重力补偿启动后机械臂可以被手动拖动")
    for remaining in range(3, 0, -1):
        print(f"{remaining} 秒后启动……")
        time.sleep(1.0)
    try:
        robot.connect()
        robot.enable()
        robot.start_gravity_compensation(transition_ms=500)
        print("双臂重力补偿已启动，按 Ctrl+C 停止")
        while True:
            health = robot.get_health()
            if health.safe_holding or health.fault_reason:
                raise RuntimeError(health.fault_reason or "双臂已进入安全保持")
            robot.read_cached_state()
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        _stop_and_close(robot)
    print("双臂已失能")


def build_parser() -> argparse.ArgumentParser:
    return argparse.ArgumentParser(description=__doc__)


if __name__ == "__main__":
    main(build_parser().parse_args())
