from __future__ import annotations

import importlib.util
from pathlib import Path

from arx_d_can._motor_abi._runtime_abi import RuntimeAbi, runtime_library_path


def test_motor_distribution_contains_native_payload_without_python_module() -> None:
    assert importlib.util.find_spec("motor_drive_layer") is None
    runtime = Path(runtime_library_path())
    assert runtime.is_file()
    assert "motor_drive_layer_native/lib" in runtime.as_posix()
    assert RuntimeAbi().lib.articore_runtime_abi_version() >= 0x00020010


def test_runtime_library_override_is_explicit(monkeypatch, tmp_path: Path) -> None:
    library = tmp_path / "libarticore_runtime.so"
    library.touch()
    monkeypatch.setenv("ARTICORE_RUNTIME_LIB", str(library))
    assert Path(runtime_library_path()) == library
