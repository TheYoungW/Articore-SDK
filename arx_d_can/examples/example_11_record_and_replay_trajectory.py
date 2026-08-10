#!/usr/bin/env python3
"""示例 11：录制并回放机械臂与夹爪轨迹。"""
from __future__ import annotations

from arx_d_can.service_tools.trajectory_recording import (
    build_parser,
    run,
)


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
