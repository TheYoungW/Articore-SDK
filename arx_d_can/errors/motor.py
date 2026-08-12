"""电机状态异常。"""
from __future__ import annotations

from collections.abc import Mapping

from .base import ArxDCanError


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
