"""面向机器人用户的夹爪命令类型。"""
from __future__ import annotations

from enum import IntEnum


class GripperForceLevel(IntEnum):
    """十档夹持力；1 最轻，5 为默认值，10 最强。"""

    LEVEL_1 = 1
    LEVEL_2 = 2
    LEVEL_3 = 3
    LEVEL_4 = 4
    LEVEL_5 = 5
    LEVEL_6 = 6
    LEVEL_7 = 7
    LEVEL_8 = 8
    LEVEL_9 = 9
    LEVEL_10 = 10


__all__ = ["GripperForceLevel"]
