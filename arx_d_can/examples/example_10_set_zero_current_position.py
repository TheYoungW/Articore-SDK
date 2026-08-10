#!/usr/bin/env python3
"""示例 10：安全地将当前静止位置设置为电机零点。"""
from __future__ import annotations

from collections.abc import Sequence

from arx_d_can.service_tools.zero_current_position import (
    build_parser,
    main as zero_current_position,
)


def main(argv: Sequence[str] | None = None) -> None:
    parser = build_parser(
        description="示例 10：确认电机静止后，将当前位置写为零点并验证。"
    )
    zero_current_position(parser.parse_args(argv))


if __name__ == "__main__":
    main()
