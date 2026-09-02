#!/usr/bin/env python3
"""控制示例 06：控制双臂和已安装夹爪返回零位。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def main() -> None:
    robot = ArxDCanDualArm()
    robot.connect()
    print("机器人连接成功")
    try:
        robot.enable()
        robot.set_joint_mit_fast(
            left=[0.0] * len(robot.joint_names),
            right=[0.0] * len(robot.joint_names),
        )
        if robot.has_grippers:
            robot.set_grippers(left=0, right=0, gripper_level=5)
            target = "双臂和夹爪"
        else:
            target = "双臂"
        input(f"目标已提交，确认{target}回到零位后按回车...")
        print(f"{target}已返回零位")
    finally:
        robot.disconnect()
        print("已断开连接")


if __name__ == "__main__":
    main()
