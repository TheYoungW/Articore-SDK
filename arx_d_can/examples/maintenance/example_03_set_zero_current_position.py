#!/usr/bin/env python3
"""维护示例 03：将双臂所有已安装电机的当前位置设置为零点。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def main() -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功，电机保持失能状态")
    try:
        input("确认将双臂和已安装夹爪的当前位置标定为零点，按回车继续...")
        if not robot.set_zero():
            raise RuntimeError("Runtime 未能确认所有电机调零成功")
        print("双臂和已安装夹爪已完成调零")
    finally:
        robot.disconnect()
        print("已断开连接")


if __name__ == "__main__":
    main()
