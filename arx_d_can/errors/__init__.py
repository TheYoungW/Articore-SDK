"""ARX-D-CAN SDK 的公开异常层级。"""

from .base import ArxDCanError, CommunicationError
from .communication import (
    FeedbackError,
    FeedbackTimeoutError,
    IncompleteFeedbackError,
    TransportError,
)
from .motor import MotorFaultError, UnexpectedMotorStateError

__all__ = [
    "ArxDCanError",
    "CommunicationError",
    "FeedbackError",
    "FeedbackTimeoutError",
    "IncompleteFeedbackError",
    "MotorFaultError",
    "TransportError",
    "UnexpectedMotorStateError",
]
