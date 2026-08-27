from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .runtime_models import MotorPowerReport


class AbiLoadError(RuntimeError):
    """无法加载产品 Runtime 动态库。"""


class RuntimeCallError(RuntimeError):
    """C++ Runtime 拒绝了本次操作。"""


class RuntimeTransactionError(RuntimeCallError):
    """整机事务失败，并携带 Runtime 的结构化报告。"""

    def __init__(
        self,
        message: str,
        report: MotorPowerReport,
    ) -> None:
        super().__init__(message)
        self.report = report
