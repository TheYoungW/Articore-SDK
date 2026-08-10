"""Public exception hierarchy for the ARX-D-CAN SDK."""
from __future__ import annotations

from collections.abc import Iterable, Mapping


class ArxDCanError(RuntimeError):
    """Base class for runtime failures reported by the SDK."""


class CommunicationError(ArxDCanError):
    """Base class for transport and motor-feedback communication failures."""

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
    """The serial or CAN transport could not be opened, read, or written."""


class FeedbackError(CommunicationError):
    """Base class for motor-feedback communication failures."""


class FeedbackTimeoutError(FeedbackError):
    """Complete fresh feedback was not received before the timeout."""


class IncompleteFeedbackError(FeedbackError):
    """Feedback was missing for one or more required motors."""


class StaleFeedbackError(FeedbackError):
    """Cached feedback is older than the configured safety limit."""

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
    """One or more motors explicitly reported a fault status code."""

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
    """A motor state is valid but inconsistent with the SDK lifecycle state."""


class CommandTimeoutError(ArxDCanError):
    """The upstream command producer stopped updating before its deadline."""


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
