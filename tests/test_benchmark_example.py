from arx_d_can.examples.example_06_benchmark_read_rate import BenchmarkResult


def test_benchmark_result_reports_achieved_hz_and_pass_state():
    result = BenchmarkResult(
        samples=1000,
        elapsed_s=2.0,
        target_hz=500.0,
        avg_read_s=0.001,
        max_read_s=0.0015,
        missed_deadlines=0,
    )

    assert result.achieved_hz == 500.0
    assert result.miss_ratio == 0.0
    assert result.passed
