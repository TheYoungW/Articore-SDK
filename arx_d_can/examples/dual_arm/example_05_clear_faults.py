#!/usr/bin/env python3
"""示例 05：清除左右臂和夹爪电机故障。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def main() -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人已连接，电机保持失能")
    try:
        robot.clear_motor_faults()
        print("双臂和已安装夹爪的电机故障已清除")
    finally:
        robot.close()
        print("双臂已失能并断开连接")


if __name__ == "__main__":
    main()
