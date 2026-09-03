#!/usr/bin/env python3
"""控制示例 16：设置一侧法兰到活动 TCP 的偏移并验证恢复默认值。"""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanDualArm


def _six_values(text: str) -> tuple[float, ...]:
    values = tuple(float(value.strip()) for value in text.split(","))
    if len(values) != 6:
        raise argparse.ArgumentTypeError(
            "offset 必须是 x,y,z,roll,pitch,yaw 六个数值"
        )
    return values


def main(args: argparse.Namespace) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    try:
        robot.connect()
        print("default offset:", robot.get_tcp_offset(side=args.side))
        print("default pose:  ", robot.get_pose(args.side))

        robot.set_tcp_offset(side=args.side, offset=args.offset)
        print("custom offset: ", robot.get_tcp_offset(side=args.side))
        print("custom pose:   ", robot.get_pose(args.side))

        robot.reset_tcp_offset(side=args.side)
        print("reset offset:  ", robot.get_tcp_offset(side=args.side))
        print("reset pose:    ", robot.get_pose(args.side))
    finally:
        if robot.connected:
            robot.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--offset",
        type=_six_values,
        default=_six_values("-0.004,0,-0.128,0,0,0"),
        help="link7 到 TCP 的 x,y,z,roll,pitch,yaw；默认将夹爪 TCP 缩短 5 cm",
    )
    return parser


if __name__ == "__main__":
    main(build_parser().parse_args())
