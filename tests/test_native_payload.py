from __future__ import annotations

import importlib.util
import ctypes
import math
import os
from importlib.metadata import distribution
from pathlib import Path
from types import SimpleNamespace

import pytest

from arx_d_can._motor_abi import ArticoreRuntime, RuntimeControlMode
from arx_d_can._motor_abi._runtime_abi import (
    CGravityCompensationStatus,
    CMotorFeedbackHealth,
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
    assert package.version == "0.24.0"
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
    assert RUNTIME_ABI_VERSION == 0x000C0000
    assert runtime_abi.abi_version == RUNTIME_ABI_VERSION
    assert not hasattr(runtime_abi, "capabilities")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_joint_pv")
    assert not hasattr(runtime_abi.lib, "articore_runtime_submit_pv_frame")
    assert not hasattr(runtime_abi.lib, "articore_runtime_submit_raw_pv")
    assert not hasattr(runtime_abi.lib, "articore_runtime_set_realtime_pv")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_joint_mit")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_joint_mit_direct")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_joint_mit_fast_follow")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_max_speed")
    assert hasattr(runtime_abi.lib, "articore_runtime_get_max_speed")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_max_acceleration")
    assert hasattr(runtime_abi.lib, "articore_runtime_get_max_acceleration")
    assert hasattr(runtime_abi.lib, "articore_runtime_set_pose")
    assert not hasattr(runtime_abi.lib, "articore_runtime_move_joint_trajectory")
    assert not hasattr(runtime_abi.lib, "articore_runtime_start_trajectory")
    assert hasattr(runtime_abi.lib, "articore_runtime_move_linear_trajectory")
    assert hasattr(
        runtime_abi.lib,
        "articore_runtime_move_linear_trajectory_with_point_count",
    )
    assert hasattr(runtime_abi.lib, "articore_runtime_move_circular_trajectory")
    assert not hasattr(runtime_abi.lib, "articore_runtime_move_linear")
    assert not hasattr(runtime_abi.lib, "articore_runtime_move_circular")
    assert hasattr(runtime_abi.lib, "articore_runtime_get_motion_status")
    assert hasattr(runtime_abi.lib, "articore_runtime_cancel_motion")
    assert hasattr(runtime_abi.lib, "articore_runtime_cancel_all_motions")
    assert hasattr(runtime_abi.lib, "articore_runtime_get_state")
    assert hasattr(runtime_abi.lib, "articore_runtime_get_health")
    assert hasattr(runtime_abi.lib, "articore_runtime_solve_ik")
    assert not hasattr(runtime_abi.lib, "articore_runtime_capabilities")
    assert not hasattr(runtime_abi.lib, "articore_runtime_set_poses")
    assert not hasattr(runtime_abi.lib, "articore_runtime_move_linear_trajectory_v2")
    assert not hasattr(runtime_abi.lib, "articore_runtime_get_state_v2")
    assert not hasattr(runtime_abi.lib, "articore_runtime_get_health_v2")
    assert not hasattr(runtime_abi.lib, "articore_runtime_get_trajectory_status")
    assert not hasattr(runtime_abi.lib, "articore_runtime_cancel_trajectory")
    assert not hasattr(runtime_abi.lib, "articore_runtime_get_cartesian_motion_status")
    assert not hasattr(runtime_abi.lib, "articore_runtime_cancel_cartesian_motion")
    assert not hasattr(runtime_abi.lib, "articore_runtime_set_joint_positions_v2")
    assert runtime_abi.lib.articore_runtime_set_joint_mit_direct.argtypes == [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_uint32,
    ]
    assert runtime_abi.lib.articore_runtime_set_joint_mit_fast_follow.argtypes == [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_uint32,
    ]
    assert runtime_abi.lib.articore_runtime_set_joint_mit.argtypes is None
    for name in ("max_speed", "max_acceleration"):
        assert getattr(runtime_abi.lib, f"articore_runtime_set_{name}").argtypes == [
            ctypes.c_void_p,
            ctypes.c_float,
        ]
        assert getattr(runtime_abi.lib, f"articore_runtime_get_{name}").argtypes == [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
        ]
    assert runtime_abi.lib.articore_runtime_solve_ik.argtypes == [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_uint32,
    ]
    assert runtime_abi.lib.articore_runtime_solve_ik.restype is ctypes.c_int32
    assert len(runtime_abi.lib.articore_runtime_move_linear_trajectory.argtypes) == 6
    linear_with_point_count = (
        runtime_abi.lib.articore_runtime_move_linear_trajectory_with_point_count
    )
    assert len(linear_with_point_count.argtypes) == 6
    assert len(
        runtime_abi.lib.articore_runtime_move_circular_trajectory.argtypes
    ) == 7
    assert runtime_abi.lib.articore_runtime_move_linear_trajectory.argtypes[-2:] == [
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_uint64),
    ]
    assert runtime_abi.lib.articore_runtime_move_circular_trajectory.argtypes[-2:] == [
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
    assert ctypes.sizeof(CMotorFeedbackHealth) == 136
    assert ctypes.sizeof(CSafetyHealth) == 13544
    assert {
        name: getattr(CMotorFeedbackHealth, name).offset
        for name, _ctype in CMotorFeedbackHealth._fields_
    } == {
        "side": 0,
        "can_id": 4,
        "can_id_valid": 8,
        "is_gripper": 12,
        "has_feedback": 16,
        "fresh": 20,
        "has_state": 24,
        "values_finite": 28,
        "status_code": 32,
        "issues": 36,
        "position": 40,
        "velocity": 44,
        "torque": 48,
        "feedback_age_ns": 56,
        "update_count": 64,
        "role": 72,
    }
    assert CSafetyHealth.motor_feedback_count.offset == 696
    assert CSafetyHealth.feedback_issue_count.offset == 700
    assert CSafetyHealth.feedback_issue_scope.offset == 704
    assert CSafetyHealth.motor_feedback.offset == 712
    assert CSafetyHealth.gripper_count.offset == 5064


def test_sdk_dependency_is_strictly_pinned_to_motor_0_24_0() -> None:
    pyproject = Path(__file__).resolve().parents[1] / "pyproject.toml"
    text = pyproject.read_text(encoding="utf-8")

    assert '"motor-drive-layer==0.24.0"' in text
    assert '"motor-drive-layer>=' not in text


def test_native_pv_global_limits_round_trip_clear_and_reject_invalid_values() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=False
    )
    try:
        assert runtime.get_max_speed() == 0.0
        assert runtime.get_max_acceleration() == 0.0

        runtime.set_max_speed(1.25)
        runtime.set_max_acceleration(2.5)
        assert runtime.get_max_speed() == pytest.approx(1.25)
        assert runtime.get_max_acceleration() == pytest.approx(2.5)

        runtime.set_max_speed(0.0)
        runtime.set_max_acceleration(0.0)
        assert runtime.get_max_speed() == 0.0
        assert runtime.get_max_acceleration() == 0.0

        for invalid in (-0.01, math.nan, math.inf, 3.15):
            with pytest.raises(RuntimeCallError, match="set_max_speed failed:"):
                runtime.set_max_speed(invalid)
        for invalid in (-0.01, math.nan, math.inf, 7.86):
            with pytest.raises(
                RuntimeCallError, match="set_max_acceleration failed:"
            ):
                runtime.set_max_acceleration(invalid)
    finally:
        runtime.disconnect()


def test_native_solve_ik_rejects_non_fourteen_output_count() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=False
    )
    try:
        pose = (ctypes.c_float * 6)()
        output = (ctypes.c_float * 14)()
        rc = runtime._runtime_abi.lib.articore_runtime_solve_ik(
            runtime._ptr,
            pose,
            pose,
            output,
            13,
        )

        assert rc != 0
        assert "output count must be 14" in runtime._last_error()
    finally:
        runtime.disconnect()


def test_native_solve_ik_without_planning_reference_or_feedback_fails() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=False
    )
    try:
        with pytest.raises(
            RuntimeCallError,
            match="requires fresh complete joint feedback",
        ):
            runtime.solve_ik((0.0,) * 6, (0.0,) * 6)
    finally:
        runtime.disconnect()


def test_runtime_library_override_is_explicit(monkeypatch, tmp_path: Path) -> None:
    library = tmp_path / "libarticore_runtime.so"
    library.touch()
    monkeypatch.setenv("ARTICORE_RUNTIME_LIB", str(library))
    assert Path(runtime_library_path()) == library


@pytest.mark.parametrize(
    "version", (0x000A0000, 0x000B0003, 0x000B0004, 0x000C0001, 0x000D0000)
)
def test_runtime_abi_requires_exact_twelve_zero(
    monkeypatch, version: int
) -> None:
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

    with pytest.raises(RuntimeError, match="exactly 12.0"):
        RuntimeAbi()


def test_native_pv_command_requires_connected_motion_state() -> None:
    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=False
    )
    try:
        with pytest.raises(RuntimeCallError, match="not accepting motion commands"):
            runtime.set_joint_pv((0.0,) * 14, 80)
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
