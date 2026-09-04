from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest


_SCRIPT = Path(__file__).parent / "hardware" / "measure_ordinary_pv_target.py"
_SPEC = importlib.util.spec_from_file_location("measure_ordinary_pv_target", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
measurement = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(measurement)


def test_metrics_separate_tracking_range_from_hold_jitter() -> None:
    times = [0.0, 1.0, 2.0, 3.0]
    positions = [
        (value,) + (0.0,) * 13
        for value in (0.0, 0.1, 0.11, 0.09)
    ]
    velocities = [
        (value,) + (0.0,) * 13
        for value in (0.0, 0.1, 0.01, -0.01)
    ]

    metrics = measurement._metrics(
        times,
        positions,
        velocities,
        (0.0,) * 14,
        (0.1,) + (0.0,) * 13,
        tolerance=0.02,
        settled_elapsed=2.0,
    )

    left_j1 = metrics[0]
    assert left_j1["position_peak_to_peak_deg"] == pytest.approx(
        math.degrees(0.11)
    )
    assert left_j1["hold_samples"] == 2
    assert left_j1["hold_position_peak_to_peak_deg"] == pytest.approx(
        math.degrees(0.02)
    )
    assert left_j1["hold_position_std_deg"] == pytest.approx(
        math.degrees(0.01)
    )
    assert left_j1["hold_max_inter_sample_change_deg"] == pytest.approx(
        math.degrees(0.02)
    )


def test_measurement_requires_explicit_targets_and_defaults_to_500_hz() -> None:
    parser = measurement.build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args([])

    args = parser.parse_args(
        [
            "--left", "0,0,0,0,0,0,0",
            "--right", "0,0,0,0,0,0,0",
        ]
    )
    assert args.sample_hz == pytest.approx(500.0)
    assert args.hold_seconds == pytest.approx(2.0)
    assert args.matlab is False


def test_matlab_batch_expression_uses_absolute_escaped_paths(tmp_path: Path) -> None:
    csv_path = tmp_path / "capture's data.csv"
    output_dir = tmp_path / "matlab's plots"

    expression = measurement._matlab_batch_expression(csv_path, output_dir)

    assert "plot_ordinary_pv_target_matlab(" in expression
    assert str(csv_path.resolve()).replace("'", "''") in expression
    assert str(output_dir.resolve()).replace("'", "''") in expression
