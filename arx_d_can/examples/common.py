"""双臂示例共用的参数解析和读取频率测试。"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable


def gripper_opening(text: str) -> float:
    """解析 0～1000 开合度；0 闭合，1000 打开。"""
    value = float(text)
    if not math.isfinite(value) or not 0.0 <= value <= 1000.0:
        raise ValueError("夹爪开合度必须是 0～1000 的有限数值")
    return value


def joint_values(text: str, *, name: str = "关节参数") -> tuple[float, ...]:
    """解析 Yunyi 单侧 7 个有限浮点参数。"""
    values = tuple(float(value) for value in text.split(",") if value.strip())
    if len(values) != 7:
        raise ValueError(f"{name}必须提供 7 个值，当前为 {len(values)} 个")
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{name}必须全部为有限值")
    return values


def joint_degrees(text: str) -> tuple[float, ...]:
    """解析 Yunyi 单侧 7 个关节角度并转换为弧度。"""
    return tuple(math.radians(value) for value in joint_values(text, name="关节角度"))


def joint_velocity_degrees(text: str) -> tuple[float, ...]:
    """解析 Yunyi 单侧 7 个目标速度并从度/秒转换为弧度/秒。"""
    return tuple(
        math.radians(value) for value in joint_values(text, name="关节目标速度")
    )


def speed_percent(text: str) -> float:
    """解析普通位置控制的 0～100 速度档位。"""
    value = float(text)
    if not math.isfinite(value) or not 0.0 <= value <= 100.0:
        raise ValueError("速度档位必须在 0～100 范围内")
    return value


def positive_velocity_degrees(text: str) -> float:
    """解析正的角速度（度/秒）并转换为弧度/秒。"""
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("速度必须是正的有限数值")
    return math.radians(value)


@dataclass(frozen=True, slots=True)
class BenchmarkResult:
    samples: int
    elapsed_s: float
    target_hz: float
    avg_read_s: float
    max_read_s: float
    missed_deadlines: int

    @property
    def achieved_hz(self) -> float:
        return self.samples / self.elapsed_s if self.elapsed_s > 0.0 else 0.0

    @property
    def miss_ratio(self) -> float:
        return self.missed_deadlines / self.samples if self.samples else 1.0

    @property
    def passed(self) -> bool:
        if self.target_hz <= 0.0:
            return self.samples > 0
        period_s = 1.0 / self.target_hz
        return self.achieved_hz >= self.target_hz * 0.95 and self.max_read_s <= period_s


def benchmark_state_reads(
    robot,
    *,
    seconds: float,
    target_hz: float,
    cached: bool = False,
    now: Callable[[], float] = time.perf_counter,
    sleep: Callable[[float], None] = time.sleep,
) -> BenchmarkResult:
    """按指定频率测试一帧完整双臂状态的读取性能。"""
    duration = float(seconds)
    frequency = float(target_hz)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("seconds must be finite and positive")
    if not math.isfinite(frequency) or frequency <= 0.0:
        raise ValueError("target_hz must be finite and positive")

    period = 1.0 / frequency
    started = now()
    deadline = started + duration
    next_tick = started
    samples = 0
    total_read_s = 0.0
    max_read_s = 0.0
    missed_deadlines = 0

    while now() < deadline:
        remaining = next_tick - now()
        if remaining > 0.0:
            sleep(remaining)
        read_started = now()
        (robot.read_cached_state if cached else robot.read_state)()
        read_s = now() - read_started
        total_read_s += read_s
        max_read_s = max(max_read_s, read_s)
        samples += 1
        next_tick += period
        if read_s > period or now() > next_tick:
            missed_deadlines += 1

    elapsed = max(0.0, now() - started)
    return BenchmarkResult(
        samples=samples,
        elapsed_s=elapsed,
        target_hz=frequency,
        avg_read_s=total_read_s / samples if samples else 0.0,
        max_read_s=max_read_s,
        missed_deadlines=missed_deadlines,
    )
