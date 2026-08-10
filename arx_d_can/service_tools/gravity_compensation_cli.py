"""重力补偿示例的命令行参数。"""
from __future__ import annotations

import argparse
import math

from arx_d_can.examples.common import add_connection_arguments


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return parsed


def non_negative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return parsed


def non_negative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return parsed


def parse_joint_values(
    text: str,
    *,
    expected_count: int,
    name: str,
    allow_negative: bool = False,
) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != expected_count:
        raise ValueError(
            f"expected {expected_count} comma-separated {name} values, got {len(values)}"
        )
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{name} values must be finite")
    if not allow_negative and any(value < 0.0 for value in values):
        raise ValueError(f"{name} values must be finite and non-negative")
    return values


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="运行 MIT 重力补偿")
    parser.add_argument("--seconds", type=non_negative_float, default=0.0)
    parser.add_argument("--hz", type=positive_float, default=100.0)
    parser.add_argument("--report-hz", type=positive_float, default=1.0)
    parser.add_argument("--transition-seconds", type=non_negative_float, default=0.0)
    parser.add_argument("--settle-seconds", type=non_negative_float, default=0.0)
    parser.add_argument("--gravity-scale", type=non_negative_float, default=1.0)
    parser.add_argument("--joint-scales")
    parser.add_argument("--damping", type=non_negative_float, default=0.0)
    parser.add_argument("--countdown", type=non_negative_int, default=3)
    add_connection_arguments(parser)
    return parser


__all__ = [
    "build_parser",
    "non_negative_float",
    "non_negative_int",
    "parse_joint_values",
    "positive_float",
]
