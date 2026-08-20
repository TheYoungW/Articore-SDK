#!/usr/bin/env python3
"""示例 10：整机清错、低速回到已标定零点，最后保持失能。"""
from __future__ import annotations

from arx_d_can import ArxDCanDualArm, RuntimeTransactionError


def main() -> None:
    robot = ArxDCanDualArm()
    try:
        try:
            robot.connect()
            print("机器人连接成功")
        except RuntimeTransactionError as error:
            if not robot.connected:
                raise
            print(f"连接配置报告异常，将交给恢复事务处理：{error}")

        input(
            "恢复将清除错误、低速回到已标定零点，"
            "最后保持失能；确认周围安全后按回车继续..."
        )
        robot.recover()
        print("双臂已清除可恢复错误、回到已标定零点并完成失能")
    finally:
        robot.disconnect()
        print("已断开连接")


if __name__ == "__main__":
    main()
