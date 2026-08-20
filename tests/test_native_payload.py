from __future__ import annotations

import importlib.util
from pathlib import Path

from arx_d_can._motor_abi._runtime_abi import (
    ARTICORE_CAP_DIRECT_GRIPPER_GAIN_X10,
    ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE,
    ARTICORE_CAP_PRODUCT_GRIPPER_DIRECT_MODE,
    ARTICORE_CAP_PRODUCT_GRIPPER_FORCE_10_LEVELS,
    MIN_RUNTIME_ABI_VERSION,
    RuntimeAbi,
    runtime_library_path,
)


def test_motor_distribution_contains_native_payload_without_python_module() -> None:
    assert importlib.util.find_spec("motor_drive_layer") is None
    runtime = Path(runtime_library_path())
    assert runtime.is_file()
    assert "motor_drive_layer_native/lib" in runtime.as_posix()
    runtime_abi = RuntimeAbi()
    assert runtime_abi.abi_version >= MIN_RUNTIME_ABI_VERSION
    assert (
        runtime_abi.capabilities
        & ARTICORE_CAP_PRODUCT_GRIPPER_FORCE_10_LEVELS
    )
    assert runtime_abi.capabilities & ARTICORE_CAP_PRODUCT_GRIPPER_DIRECT_MODE
    assert runtime_abi.capabilities & ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE
    assert runtime_abi.capabilities & ARTICORE_CAP_DIRECT_GRIPPER_GAIN_X10


def test_runtime_library_override_is_explicit(monkeypatch, tmp_path: Path) -> None:
    library = tmp_path / "libarticore_runtime.so"
    library.touch()
    monkeypatch.setenv("ARTICORE_RUNTIME_LIB", str(library))
    assert Path(runtime_library_path()) == library
