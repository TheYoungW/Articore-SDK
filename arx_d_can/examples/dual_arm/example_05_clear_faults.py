#!/usr/bin/env python3
"""示例 05：清除左右臂和夹爪电机故障。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def main() -> None:
    robot = ArxDCanDualArm()
    print("正在以维护模式连接；不会切换 MIT/PV，也不会使能电机")
    try:
        left, right = robot.clear_motor_faults()
        print("左臂故障已清除：", ", ".join(left))
        print("右臂故障已清除：", ", ".join(right))
        print("所有电机保持失能状态")
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    main()
