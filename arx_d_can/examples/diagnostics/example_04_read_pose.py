#!/usr/bin/env python3
"""诊断示例 04：只读获取左右臂当前法兰位姿。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm


def _format_pose(pose: list[float]) -> str:
    return "[" + ", ".join(f"{value:.6f}" for value in pose) + "]"


def main() -> None:
    robot = ArxDCanDualArm()
    try:
        robot.connect()
        left = robot.get_pose("left")
        right = robot.get_pose("right")
        print("位姿格式：[x, y, z, roll, pitch, yaw]")
        print("位置单位：米；姿态单位：弧度")
        print(f"left:  {_format_pose(left)}")
        print(f"right: {_format_pose(right)}")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
