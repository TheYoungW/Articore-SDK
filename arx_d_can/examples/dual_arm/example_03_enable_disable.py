#!/usr/bin/env python3
"""示例 03：交互式使能和失能双臂。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def main() -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    robot.connect()
    print("机器人连接成功，当前保持失能")
    try:
        input("确认环境安全后，按回车使能左右臂...")
        robot.enable()
        state = robot.read_cached_state()
        robot.set_joint_pv(
            left=state.left.positions,
            right=state.right.positions,
        )
        print("左右臂已使能，正在保持当前位置")

        input("按回车失能左右臂...")
        robot.disable()
        print("左右臂已失能")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        robot.close()
        print("已断开连接")


if __name__ == "__main__":
    main()
