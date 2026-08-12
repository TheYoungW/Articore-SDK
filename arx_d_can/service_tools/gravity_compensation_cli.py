"""单臂重力补偿示例的连接参数。"""
from __future__ import annotations

import argparse

from .common import add_connection_arguments


def build_parser(*, allow_custom_config: bool = False) -> argparse.ArgumentParser:
    """创建只包含机型和通信选项的简洁参数解析器。"""
    parser = argparse.ArgumentParser(description="开启单臂 MIT 重力补偿")
    add_connection_arguments(
        parser,
        allow_custom_config=allow_custom_config,
        default_arm_model=None if allow_custom_config else "yunyi_v1_0_right",
    )
    return parser


__all__ = ["build_parser"]
