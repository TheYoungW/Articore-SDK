"""传输和电机反馈异常。"""

from .base import CommunicationError


class TransportError(CommunicationError):
    """串口或 CAN 通道无法打开、读取或写入。"""


class FeedbackError(CommunicationError):
    """电机反馈通信故障的基类。"""


class FeedbackTimeoutError(FeedbackError):
    """超时前未收到完整反馈。"""


class IncompleteFeedbackError(FeedbackError):
    """一个或多个必需电机缺少反馈。"""
