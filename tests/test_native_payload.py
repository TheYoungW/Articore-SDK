from __future__ import annotations

import importlib.util
import ctypes
import os
from importlib.metadata import distribution
from pathlib import Path

import pytest

from arx_d_can._motor_abi import ArticoreRuntime, RuntimeControlMode
from arx_d_can._motor_abi._runtime_abi import (
    ARTICORE_CAP_DIRECT_GRIPPER_GAIN_X10,
    ARTICORE_CAP_DIRECT_CPP_MOTOR_CORE,
    ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE,
    ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR,
    ARTICORE_CAP_PRODUCT_CARTESIAN_LINEAR,
    ARTICORE_CAP_PRODUCT_CARTESIAN_POINT_TO_POINT,
    ARTICORE_CAP_PRODUCT_MAX_SPEED_SETTING,
    ARTICORE_CAP_PRODUCT_TOOL_CENTER_POSE,
    ARTICORE_CAP_PRODUCT_GRIPPER_DIRECT_MODE,
    ARTICORE_CAP_PRODUCT_GRIPPER_FORCE_10_LEVELS,
    ARTICORE_CAP_PRODUCT_JOINT_ANGLE_VEL_LIMITS,
    ARTICORE_CAP_PRODUCT_PV_COMMAND_SPEED,
    CCartesianMotionStatus,
    CProductJointAngleVelLimits,
    MIN_RUNTIME_ABI_VERSION,
    RuntimeAbi,
    runtime_library_path,
)
from arx_d_can._motor_abi.errors import RuntimeCallError


def test_motor_distribution_contains_native_payload_without_python_module() -> None:
    assert importlib.util.find_spec("motor_drive_layer") is None
    package = distribution("motor-drive-layer")
    assert package.version == "0.13.1"
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
    assert runtime_abi.abi_version >= MIN_RUNTIME_ABI_VERSION
    assert (
        runtime_abi.capabilities
        & ARTICORE_CAP_PRODUCT_GRIPPER_FORCE_10_LEVELS
    )
    assert runtime_abi.capabilities & ARTICORE_CAP_PRODUCT_GRIPPER_DIRECT_MODE
    assert runtime_abi.capabilities & ARTICORE_CAP_FIXED_GRIPPER_MIT_MODE
    assert runtime_abi.capabilities & ARTICORE_CAP_DIRECT_GRIPPER_GAIN_X10
    assert (
        runtime_abi.capabilities
        & ARTICORE_CAP_PRODUCT_CARTESIAN_POINT_TO_POINT
    )
    assert runtime_abi.capabilities & ARTICORE_CAP_PRODUCT_CARTESIAN_LINEAR
    assert runtime_abi.capabilities & ARTICORE_CAP_PRODUCT_CARTESIAN_CIRCULAR
    assert runtime_abi.capabilities & ARTICORE_CAP_PRODUCT_MAX_SPEED_SETTING
    assert runtime_abi.capabilities & ARTICORE_CAP_PRODUCT_TOOL_CENTER_POSE
    assert (
        runtime_abi.capabilities
        & ARTICORE_CAP_PRODUCT_JOINT_ANGLE_VEL_LIMITS
    )
    assert runtime_abi.capabilities & ARTICORE_CAP_PRODUCT_PV_COMMAND_SPEED
    assert runtime_abi.capabilities & ARTICORE_CAP_DIRECT_CPP_MOTOR_CORE
    assert hasattr(runtime_abi.lib, "articore_runtime_set_joint_pv")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_joint_mit")
    assert not hasattr(runtime_abi.lib, "articore_runtime_set_joint_positions_v2")
    assert ctypes.sizeof(CCartesianMotionStatus) == 600
    assert ctypes.sizeof(CProductJointAngleVelLimits) == 176


def test_runtime_library_override_is_explicit(monkeypatch, tmp_path: Path) -> None:
    library = tmp_path / "libarticore_runtime.so"
    library.touch()
    monkeypatch.setenv("ARTICORE_RUNTIME_LIB", str(library))
    assert Path(runtime_library_path()) == library


def test_native_ordinary_motion_max_speed_default_and_range() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=False
    )
    try:
        assert runtime.get_max_speed() == pytest.approx(50.0)
        runtime.set_max_speed(0)
        assert runtime.get_max_speed() == pytest.approx(0.0)
        runtime.set_max_speed(100)
        assert runtime.get_max_speed() == pytest.approx(100.0)
        with pytest.raises(RuntimeCallError, match="within 0..100"):
            runtime.set_max_speed(-0.1)
        with pytest.raises(RuntimeCallError, match="within 0..100"):
            runtime.set_max_speed(100.1)
    finally:
        runtime.disconnect()


def test_native_max_speed_is_not_a_mit_setting() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.MIT, with_grippers=False
    )
    try:
        with pytest.raises(RuntimeCallError, match="only in product PV mode"):
            runtime.set_max_speed(50)
        with pytest.raises(RuntimeCallError, match="only in product PV mode"):
            runtime.get_max_speed()
    finally:
        runtime.disconnect()


def test_native_pv_command_speed_is_separate_from_global_maximum() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=False
    )
    try:
        runtime.set_max_speed(25)
        with pytest.raises(RuntimeCallError, match="not accepting motion commands"):
            runtime.set_joint_pv((0.0,) * 14, 80)
        assert runtime.get_max_speed() == pytest.approx(25.0)
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
