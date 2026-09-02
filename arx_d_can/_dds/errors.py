from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .models import MotorPowerReport


class RuntimeErrorCode(str, Enum):
    INVALID_ARGUMENT = "INVALID_ARGUMENT"
    WRONG_STATE = "WRONG_STATE"
    BUSY = "BUSY"
    NO_LEASE = "NO_LEASE"
    STALE_SEQUENCE = "STALE_SEQUENCE"
    TIMEOUT = "TIMEOUT"
    TRANSPORT_ERROR = "TRANSPORT_ERROR"
    VERSION_MISMATCH = "VERSION_MISMATCH"
    INTERNAL_ERROR = "INTERNAL_ERROR"


class RuntimeCallError(RuntimeError):
    """RK3588 Runtime 拒绝请求或 DDS 请求失败。"""

    def __init__(
        self,
        message: str,
        *,
        code: RuntimeErrorCode | str | None = None,
    ) -> None:
        super().__init__(message)
        self.code = RuntimeErrorCode(code) if code is not None else None


class RuntimeTransactionError(RuntimeCallError):
    """整机事务失败，并携带 Runtime 的结构化报告。"""

    def __init__(
        self,
        message: str,
        report: MotorPowerReport,
    ) -> None:
        super().__init__(message)
        self.report = report
