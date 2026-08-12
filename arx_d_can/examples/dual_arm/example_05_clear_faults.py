#!/usr/bin/env python3
"""示例 05：清除左右臂和夹爪电机故障。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def main() -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功")
    try:
        left = robot.left.clear_motor_faults()
        right = robot.right.clear_motor_faults()
        print("左臂故障已清除：", ", ".join(left))
        print("右臂故障已清除：", ", ".join(right))
        print("所有电机保持失能状态")
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    main()
