"""SDK 异常基类及共享通信上下文。"""
from __future__ import annotations

from collections.abc import Iterable


class ArxDCanError(RuntimeError):
    """SDK 报告的运行时故障基类。"""


class CommunicationError(ArxDCanError):
    """通信通道及电机反馈通信故障的基类。"""

    def __init__(
        self,
        message: str,
        *,
        operation: str | None = None,
        transport: str | None = None,
        channel: str | None = None,
        motor_names: Iterable[str] = (),
        retryable: bool = False,
    ) -> None:
        super().__init__(message)
        self.operation = operation
        self.transport = transport
        self.channel = channel
        self.motor_names = tuple(str(name) for name in motor_names)
        self.retryable = bool(retryable)
