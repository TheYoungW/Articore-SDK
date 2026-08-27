from __future__ import annotations

import importlib.util
import ctypes
import os
from importlib.metadata import distribution
from pathlib import Path
from types import SimpleNamespace

import pytest

from arx_d_can._motor_abi import ArticoreRuntime, RuntimeControlMode
from arx_d_can._motor_abi._runtime_abi import (
    CGravityCompensationStatus,
    CMotionStatus,
    CProductJointAngleVelLimits,
    CProductState,
    CSafetyHealth,
    RUNTIME_ABI_VERSION,
    RuntimeAbi,
    runtime_library_path,
)
from arx_d_can._motor_abi.errors import RuntimeCallError


def test_motor_distribution_contains_native_payload_without_python_module() -> None:
    assert importlib.util.find_spec("motor_drive_layer") is None
    package = distribution("motor-drive-layer")
    assert package.version == "0.18.0"
    package_files = {entry.as_posix() for entry in package.files or ()}
    native_payload = {
        entry for entry in package_files
        if entry.startswith("motor_drive_layer_native/")
    }
    assert native_payload == {
        "motor_drive_layer_native/PAYLOAD",
        "motor_drive_layer_native/lib/libarticore_runtime.so",
    }
    assert not any("libmotor_abi" in entry for entry in package_files)
    assert not any("dm_device" in entry.lower() for entry in package_files)
    assert not any("libusb" in entry.lower() for entry in package_files)
    runtime = Path(runtime_library_path())
    assert runtime.is_file()
    if "ARTICORE_RUNTIME_LIB" not in os.environ:
        assert "motor_drive_layer_native/lib" in runtime.as_posix()
    runtime_abi = RuntimeAbi()
    assert runtime_abi.abi_version == RUNTIME_ABI_VERSION
    assert not hasattr(runtime_abi, "capabilities")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_joint_pv")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_joint_mit")
    assert not hasattr(runtime_abi.lib, "articore_runtime_set_max_speed")
    assert not hasattr(runtime_abi.lib, "articore_runtime_get_max_speed")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_max_acceleration")
    assert hasattr(runtime_abi.lib, "articore_runtime_get_max_acceleration")
    assert hasattr(runtime_abi.lib, "articore_runtime_move_pose")
    assert hasattr(runtime_abi.lib, "articore_runtime_start_trajectory")
    assert hasattr(runtime_abi.lib, "articore_runtime_move_linear")
    assert hasattr(runtime_abi.lib, "articore_runtime_move_circular")
    assert hasattr(runtime_abi.lib, "articore_runtime_get_motion_status")
    assert hasattr(runtime_abi.lib, "articore_runtime_cancel_motion")
    assert hasattr(runtime_abi.lib, "articore_runtime_cancel_all_motions")
    assert hasattr(runtime_abi.lib, "articore_runtime_get_state")
    assert hasattr(runtime_abi.lib, "articore_runtime_get_health")
    assert not hasattr(runtime_abi.lib, "articore_runtime_capabilities")
    assert not hasattr(runtime_abi.lib, "articore_runtime_move_poses")
    assert not hasattr(runtime_abi.lib, "articore_runtime_move_linear_v2")
    assert not hasattr(runtime_abi.lib, "articore_runtime_get_state_v2")
    assert not hasattr(runtime_abi.lib, "articore_runtime_get_health_v2")
    assert not hasattr(runtime_abi.lib, "articore_runtime_get_trajectory_status")
    assert not hasattr(runtime_abi.lib, "articore_runtime_cancel_trajectory")
    assert not hasattr(runtime_abi.lib, "articore_runtime_get_cartesian_motion_status")
    assert not hasattr(runtime_abi.lib, "articore_runtime_cancel_cartesian_motion")
    assert not hasattr(runtime_abi.lib, "articore_runtime_set_joint_positions_v2")
    assert runtime_abi.lib.articore_runtime_set_max_acceleration.argtypes == [
        ctypes.c_void_p, ctypes.c_float,
    ]
    assert runtime_abi.lib.articore_runtime_get_max_acceleration.argtypes == [
        ctypes.c_void_p, ctypes.POINTER(ctypes.c_float),
    ]
    assert runtime_abi.lib.articore_runtime_start_trajectory.argtypes[-1] is (
        ctypes.POINTER(ctypes.c_uint64)
    )
    assert runtime_abi.lib.articore_runtime_move_linear.argtypes[-2:] == [
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_uint64),
    ]
    assert runtime_abi.lib.articore_runtime_move_circular.argtypes[-2:] == [
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_uint64),
    ]
    assert runtime_abi.lib.articore_runtime_get_motion_status.argtypes[-1] is (
        ctypes.POINTER(CMotionStatus)
    )
    assert ctypes.sizeof(CMotionStatus) == 568
    assert {
        name: getattr(CMotionStatus, name).offset
        for name, _ctype in CMotionStatus._fields_
    } == {
        "struct_size": 0,
        "motion_id": 8,
        "motion_type": 16,
        "state": 20,
        "active_segment": 24,
        "waypoint_count": 28,
        "elapsed_s": 32,
        "duration_s": 40,
        "progress": 48,
        "error": 52,
    }
    assert ctypes.sizeof(CProductJointAngleVelLimits) == 176
    assert ctypes.sizeof(CGravityCompensationStatus) == 88
    assert ctypes.sizeof(CProductState) == 392
    assert ctypes.sizeof(CSafetyHealth) == 9176


def test_runtime_library_override_is_explicit(monkeypatch, tmp_path: Path) -> None:
    library = tmp_path / "libarticore_runtime.so"
    library.touch()
    monkeypatch.setenv("ARTICORE_RUNTIME_LIB", str(library))
    assert Path(runtime_library_path()) == library


@pytest.mark.parametrize("version", (0x00080000, 0x00090001, 0x000A0000))
def test_runtime_abi_requires_exact_nine_zero(monkeypatch, version: int) -> None:
    class VersionFunction:
        argtypes = None
        restype = None

        def __call__(self) -> int:
            return version

    fake_library = SimpleNamespace(
        articore_runtime_abi_version=VersionFunction(),
    )
    monkeypatch.setattr(ctypes, "CDLL", lambda _path: fake_library)
    monkeypatch.setattr(
        "arx_d_can._motor_abi._runtime_abi.runtime_library_path",
        lambda: "/tmp/fake-libarticore-runtime.so",
    )

    with pytest.raises(RuntimeError, match="exactly 9.0"):
        RuntimeAbi()


def test_native_ordinary_pv_acceleration_default_range_and_resolution() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=False
    )
    try:
        assert runtime.get_max_acceleration() == pytest.approx(4.0)
        runtime.set_max_acceleration(0.01)
        assert runtime.get_max_acceleration() == pytest.approx(0.01)
        runtime.set_max_acceleration(8.0)
        assert runtime.get_max_acceleration() == pytest.approx(8.0)
        runtime.set_max_acceleration(4.56)
        assert runtime.get_max_acceleration() == pytest.approx(4.56)
        for value in (0.0, 8.01, float("nan"), float("inf")):
            with pytest.raises(RuntimeCallError, match="within"):
                runtime.set_max_acceleration(value)
        with pytest.raises(RuntimeCallError, match="0.01 physical-unit resolution"):
            runtime.set_max_acceleration(4.565)
    finally:
        runtime.disconnect()


def test_native_ordinary_pv_acceleration_is_not_a_mit_setting() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.MIT, with_grippers=False
    )
    try:
        with pytest.raises(RuntimeCallError, match="only in product PV mode"):
            runtime.set_max_acceleration(4.0)
        with pytest.raises(RuntimeCallError, match="only in product PV mode"):
            runtime.get_max_acceleration()
    finally:
        runtime.disconnect()


def test_native_pv_command_speed_is_separate_from_acceleration_limit() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=False
    )
    try:
        runtime.set_max_acceleration(4.56)
        with pytest.raises(RuntimeCallError, match="not accepting motion commands"):
            runtime.set_joint_pv((0.0,) * 14, 80)
        assert runtime.get_max_acceleration() == pytest.approx(4.56)
    finally:
        runtime.disconnect()


def test_native_joint_limits_are_available_before_connect() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=False
    )
    try:
        limits = runtime.get_joint_limits()
        assert len(limits) == 14
        assert limits[0].min_angle_rad == pytest.approx(-2.745)
        assert limits[0].max_angle_rad == pytest.approx(2.745)
        assert limits[1].min_angle_rad == pytest.approx(-0.3489)
        assert limits[8].max_angle_rad == pytest.approx(0.3489)
        assert all(
            limit.max_velocity_rad_s == pytest.approx(5.0)
            for limit in limits
        )
    finally:
        runtime.disconnect()
