from __future__ import annotations

import importlib.util
from pathlib import Path

from arx_d_can._motor_abi import abi
from arx_d_can._motor_abi.native_payload import (
    distribution_payload_candidates,
    source_checkout_root,
)


def test_motor_distribution_contains_native_payload_without_python_module() -> None:
    assert importlib.util.find_spec("motor_drive_layer") is None

    motor = distribution_payload_candidates("lib", "libmotor_abi.so")
    runtime = distribution_payload_candidates("lib", "libarticore_runtime.so")

    assert len(motor) == 1 and motor[0].is_file()
    assert len(runtime) == 1 and runtime[0].is_file()
    assert "motor_drive_layer_native/lib" in runtime[0].as_posix()
    assert Path(abi.articore_runtime_library_path()) == runtime[0]


def test_source_checkout_override_is_explicit(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("MOTOR_DRIVE_LAYER_SOURCE", str(tmp_path))
    assert source_checkout_root() == tmp_path.resolve()
