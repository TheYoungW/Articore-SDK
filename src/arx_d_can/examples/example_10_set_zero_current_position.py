#!/usr/bin/env python3
"""Example 10: safely set the current stationary motor positions as zero."""
from __future__ import annotations

from collections.abc import Sequence

from arx_d_can.service_tools.zero_current_position import (
    build_parser,
    main as zero_current_position,
)


def main(argv: Sequence[str] | None = None) -> None:
    parser = build_parser(
        description=(
            "Example 10: verify that ARX-D-CAN motors are stationary, then write "
            "their current positions as zero and verify every write."
        )
    )
    zero_current_position(parser.parse_args(argv))


if __name__ == "__main__":
    main()
