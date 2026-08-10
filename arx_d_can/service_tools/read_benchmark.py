"""状态读取频率测试的内部实现。"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable


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
        return self.missed_deadlines / self.samples if self.samples > 0 else 1.0

    @property
    def passed(self) -> bool:
        if self.target_hz <= 0.0:
            return self.samples > 0
        period_s = 1.0 / self.target_hz
        return self.achieved_hz >= self.target_hz * 0.95 and self.max_read_s <= period_s


def benchmark_state_reads(
    arm,
    *,
    seconds: float,
    target_hz: float,
    cached: bool = False,
    now: Callable[[], float] = time.perf_counter,
    sleep: Callable[[float], None] = time.sleep,
) -> BenchmarkResult:
    """按指定频率测试新鲜状态或缓存状态的读取性能。"""
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
        if cached:
            arm.read_cached_state()
        else:
            arm.read_state()
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


__all__ = ["BenchmarkResult", "benchmark_state_reads"]
