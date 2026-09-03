#!/usr/bin/env python3
"""维护示例 01：清除左右臂和夹爪电机故障。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def main() -> None:
    robot = ArxDCanDualArm()
    robot.connect(maintenance=True)
    print("机器人已建立维护连接，电机保持失能且不配置控制模式")
    try:
        robot.clear_motor_faults()
        print("双臂和已安装夹爪的电机故障已清除")
    finally:
        robot.disconnect()
        print("双臂已失能并断开连接")


if __name__ == "__main__":
    main()
