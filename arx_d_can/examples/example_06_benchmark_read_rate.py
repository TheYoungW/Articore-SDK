#!/usr/bin/env python3
"""Example 06: benchmark read_state loop rate."""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Callable

from arx_d_can import ArxDCanArm
from arx_d_can.examples.common import add_connection_arguments, arm_kwargs


@dataclass(frozen=True)
class BenchmarkResult:
    samples: int
    elapsed_s: float
    target_hz: float
    avg_read_s: float
    max_read_s: float
    missed_deadlines: int

    @property
    def achieved_hz(self) -> float:
        if self.elapsed_s <= 0.0:
            return 0.0
        return self.samples / self.elapsed_s

    @property
    def miss_ratio(self) -> float:
        if self.samples <= 0:
            return 1.0
        return self.missed_deadlines / self.samples

    @property
    def passed(self) -> bool:
        if self.target_hz <= 0.0:
            return self.samples > 0
        period_s = 1.0 / self.target_hz
        return self.achieved_hz >= self.target_hz * 0.95 and self.max_read_s <= period_s


def run_read_benchmark(
    arm,
    *,
    seconds: float,
    target_hz: float,
    request_feedback: bool = True,
    now: Callable[[], float] = time.perf_counter,
    sleep: Callable[[float], None] = time.sleep,
) -> BenchmarkResult:
    duration_s = max(0.0, float(seconds))
    target_hz = max(0.0, float(target_hz))
    period_s = 0.0 if target_hz <= 0.0 else 1.0 / target_hz
    deadline = now() + duration_s
    next_tick = now()
    samples = 0
    total_read_s = 0.0
    max_read_s = 0.0
    missed_deadlines = 0
    started = now()

    while now() < deadline:
        if period_s > 0.0:
            remaining = next_tick - now()
            if remaining > 0.0:
                sleep(remaining)

        read_started = now()
        arm.read_state(request_feedback=request_feedback)
        read_s = now() - read_started
        total_read_s += read_s
        max_read_s = max(max_read_s, read_s)
        samples += 1

        if period_s > 0.0:
            next_tick += period_s
            if read_s > period_s or now() > next_tick:
                missed_deadlines += 1

    elapsed_s = max(0.0, now() - started)
    avg_read_s = total_read_s / samples if samples else 0.0
    return BenchmarkResult(
        samples=samples,
        elapsed_s=elapsed_s,
        target_hz=target_hz,
        avg_read_s=avg_read_s,
        max_read_s=max_read_s,
        missed_deadlines=missed_deadlines,
    )


def print_result(result: BenchmarkResult) -> None:
    print(f"samples: {result.samples}")
    print(f"elapsed_s: {result.elapsed_s:.6f}")
    print(f"achieved_hz: {result.achieved_hz:.2f}")
    print(f"target_hz: {result.target_hz:.2f}")
    print(f"avg_read_ms: {result.avg_read_s * 1000.0:.3f}")
    print(f"max_read_ms: {result.max_read_s * 1000.0:.3f}")
    print(f"missed_deadlines: {result.missed_deadlines}")
    print(f"miss_ratio: {result.miss_ratio:.3%}")
    print(f"result: {'PASS' if result.passed else 'FAIL'}")


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(**arm_kwargs(args))
    try:
        arm.connect()
        if args.warmup_seconds > 0.0:
            run_read_benchmark(
                arm,
                seconds=args.warmup_seconds,
                target_hz=args.target_hz,
                request_feedback=not args.no_request_feedback,
            )
        result = run_read_benchmark(
            arm,
            seconds=args.seconds,
            target_hz=args.target_hz,
            request_feedback=not args.no_request_feedback,
        )
        print_result(result)
    finally:
        arm.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Benchmark ARX-D-CAN read_state loop rate.")
    parser.add_argument("--seconds", type=float, default=5.0, help="Benchmark duration")
    parser.add_argument("--target-hz", type=float, default=500.0, help="Target read loop frequency")
    parser.add_argument("--warmup-seconds", type=float, default=0.5, help="Warmup duration before measuring")
    parser.add_argument(
        "--no-request-feedback",
        action="store_true",
        help="Read cached state only; default requests fresh motor feedback each sample",
    )
    add_connection_arguments(parser)
    main(parser.parse_args())
