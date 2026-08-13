#!/usr/bin/env python3
"""示例 13：将双臂所有已启用电机的当前位置设置为零点。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def main() -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功，电机保持失能状态")
    try:
        left = robot.left.set_zero()
        right = robot.right.set_zero()
        print("左臂零点设置完成：", ", ".join(left))
        print("右臂零点设置完成：", ", ".join(right))
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    main()
