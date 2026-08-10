"""ARX-D-CAN SDK 的公开异常层级。"""
from __future__ import annotations

from collections.abc import Iterable, Mapping


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


class TransportError(CommunicationError):
    """串口或 CAN 通道无法打开、读取或写入。"""


class FeedbackError(CommunicationError):
    """电机反馈通信故障的基类。"""


class FeedbackTimeoutError(FeedbackError):
    """超时前未收到完整的新鲜反馈。"""


class IncompleteFeedbackError(FeedbackError):
    """一个或多个必需电机缺少反馈。"""


class StaleFeedbackError(FeedbackError):
    """缓存反馈的数据年龄超过配置的安全限制。"""

    def __init__(
        self,
        message: str,
        *,
        feedback_ages_s: Mapping[str, float] | None = None,
        age_limit_s: float | None = None,
        **kwargs,
    ) -> None:
        super().__init__(message, **kwargs)
        self.feedback_ages_s = dict(feedback_ages_s or {})
        self.age_limit_s = age_limit_s


class MotorFaultError(ArxDCanError):
    """一个或多个电机明确报告了故障状态码。"""

    def __init__(
        self,
        message: str,
        *,
        status_codes: Mapping[str, int] | None = None,
    ) -> None:
        super().__init__(message)
        self.status_codes = {
            str(name): int(status) for name, status in (status_codes or {}).items()
        }


class UnexpectedMotorStateError(MotorFaultError):
    """电机状态有效，但与 SDK 生命周期状态不一致。"""


class CommandTimeoutError(ArxDCanError):
    """上游命令生产者未在截止时间前继续更新。"""


__all__ = [
    "ArxDCanError",
    "CommandTimeoutError",
    "CommunicationError",
    "FeedbackError",
    "FeedbackTimeoutError",
    "IncompleteFeedbackError",
    "MotorFaultError",
    "StaleFeedbackError",
    "TransportError",
    "UnexpectedMotorStateError",
]
