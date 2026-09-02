from pathlib import Path

import arx_d_can


def test_sdk_contains_dds_transport_and_no_ctypes_abi_package() -> None:
    root = Path(arx_d_can.__file__).resolve().parent
    assert (root / "_dds" / "client.py").is_file()
    assert not (root / "_motor_abi" / "runtime.py").exists()
    assert not (root / "_motor_abi" / "_runtime_abi.py").exists()


def test_project_depends_on_cyclonedds_not_native_motor_wheel() -> None:
    pyproject = (Path(__file__).resolve().parents[1] / "pyproject.toml").read_text()
    assert '"cyclonedds==11.0.1"' in pyproject
    assert "motor-drive-layer" not in pyproject
