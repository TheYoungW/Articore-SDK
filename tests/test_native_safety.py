from __future__ import annotations

import ctypes
from types import SimpleNamespace
import pytest

from motor_drive_layer import articore_runtime_library_path

from arx_d_can.sdk.native_safety import (
    ARTICORE_CAP_COMMAND_LIFETIME,
    ARTICORE_CAP_DETERMINISTIC_DISABLE,
    ARTICORE_CAP_NONPREEMPTIVE_TRAJECTORY,
    ARTICORE_CAP_PROTECTIVE_FAULT_HOLD,
    ARTICORE_CAP_TRAJECTORY_MANAGEMENT,
    ARTICORE_CAP_TRAJECTORY_SETTLING,
    CommandLifetime,
    DisableReport,
    EnableReport,
    GripperControlState,
    NativeDisableError,
    NativeJointControlConfig,
    NativeMotorDescriptor,
    NativeEnableError,
    NativeSafetyRuntime,
    SafetyState,
    TrajectoryExecutionConfig,
    TrajectoryStatus,
    _DisableReport,
    _EnableReport,
    _SafetyHealth,
    _TrajectoryExecutionConfig,
    _TrajectoryInfo,
)


def test_packaged_runtime_exposes_required_abi_1_8_capabilities() -> None:
    library = ctypes.CDLL(articore_runtime_library_path())
    library.articore_runtime_abi_version.restype = ctypes.c_uint32
    library.articore_runtime_capabilities.restype = ctypes.c_uint64

    assert int(library.articore_runtime_abi_version()) >= (1 << 16) | 8
    required = (
        ARTICORE_CAP_COMMAND_LIFETIME
        | ARTICORE_CAP_NONPREEMPTIVE_TRAJECTORY
        | ARTICORE_CAP_PROTECTIVE_FAULT_HOLD
        | ARTICORE_CAP_DETERMINISTIC_DISABLE
        | ARTICORE_CAP_TRAJECTORY_MANAGEMENT
        | ARTICORE_CAP_TRAJECTORY_SETTLING
    )
    assert int(library.articore_runtime_capabilities()) & required == required


def test_native_submit_uses_abi_1_8_command_lifetime_entry_points() -> None:
    calls: list[tuple[str, int]] = []

    class Library:
        @staticmethod
        def articore_runtime_submit_pos_vel_ex(
            _runtime, _commands, _count, lifetime
        ) -> int:
            calls.append(("pv", int(lifetime)))
            return 0

        @staticmethod
        def articore_runtime_submit_mit_ex(
            _runtime, _commands, _count, lifetime
        ) -> int:
            calls.append(("mit", int(lifetime)))
            return 0

        @staticmethod
        def articore_runtime_last_error():
            return b"ok"

    motor = SimpleNamespace(_ptr=0x123)
    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = Library()
    runtime._ptr = 1
    runtime._pv_array = None
    runtime._arm_mit_array = None

    runtime.submit_pos_vel(
        [SimpleNamespace(motor=motor, pos=0.1, vlim=1.0)],
        lifetime=CommandLifetime.STREAMING,
    )
    runtime.submit_mit(
        [SimpleNamespace(motor=motor, pos=0.1, vel=0.0, kp=10.0, kd=1.0, tau=0.0)],
        lifetime=CommandLifetime.HOLD_UNTIL_REPLACED,
    )

    assert calls == [
        ("pv", int(CommandLifetime.STREAMING)),
        ("mit", int(CommandLifetime.HOLD_UNTIL_REPLACED)),
    ]


def test_trajectory_execution_configuration_uses_abi_1_8_structure() -> None:
    captured: list[_TrajectoryExecutionConfig] = []

    class Library:
        @staticmethod
        def articore_runtime_configure_trajectory_execution(
            _runtime,
            value,
        ) -> int:
            source = ctypes.cast(
                value,
                ctypes.POINTER(_TrajectoryExecutionConfig),
            ).contents
            copy = _TrajectoryExecutionConfig()
            ctypes.memmove(
                ctypes.byref(copy),
                ctypes.byref(source),
                ctypes.sizeof(copy),
            )
            captured.append(copy)
            return 0

        @staticmethod
        def articore_runtime_last_error():
            return b"ok"

    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = Library()
    runtime._ptr = 1
    config = TrajectoryExecutionConfig(
        position_tolerance=0.01,
        velocity_tolerance=0.02,
        following_error_limit=0.4,
        settling_stable_ms=150,
        settling_timeout_ms=2500,
        following_error_timeout_ms=120,
    )

    runtime.configure_trajectory_execution(config)

    assert captured[0].struct_size == ctypes.sizeof(_TrajectoryExecutionConfig)
    assert captured[0].position_tolerance == pytest.approx(0.01)
    assert captured[0].velocity_tolerance == pytest.approx(0.02)
    assert captured[0].following_error_limit == pytest.approx(0.4)
    assert captured[0].settling_stable_ms == 150
    assert captured[0].settling_timeout_ms == 2500
    assert captured[0].following_error_timeout_ms == 120
    assert runtime._trajectory_settling_timeout_s == 2.5


@pytest.mark.parametrize(
    ("duration_s", "elapsed_s", "settling_timeout_s", "expected_timeout_ms"),
    (
        (10.0, 4.0, 3.0, 10_000),
        (3.0, 3.0, 3.0, 4_000),
        (3.0, 3.0, 5.0, 6_000),
    ),
)
def test_wait_trajectory_includes_settling_timeout_after_reference(
    duration_s: float,
    elapsed_s: float,
    settling_timeout_s: float,
    expected_timeout_ms: int,
) -> None:
    timeouts: list[int] = []

    class Library:
        @staticmethod
        def articore_runtime_get_trajectory(
            _runtime,
            _trajectory_id,
            output,
        ) -> int:
            info = ctypes.cast(output, ctypes.POINTER(_TrajectoryInfo)).contents
            info.trajectory_id = 7
            info.status = 1
            info.profile = 1
            info.duration_ns = round(duration_s * 1_000_000_000)
            info.elapsed_ns = round(elapsed_s * 1_000_000_000)
            return 0

        @staticmethod
        def articore_runtime_wait_trajectory(
            _runtime, _trajectory_id, timeout_ms, output
        ) -> int:
            timeouts.append(int(timeout_ms))
            info = ctypes.cast(output, ctypes.POINTER(_TrajectoryInfo)).contents
            info.trajectory_id = 7
            info.status = 2
            info.profile = 1
            info.duration_ns = round(duration_s * 1_000_000_000)
            info.elapsed_ns = round(duration_s * 1_000_000_000)
            return 0

        @staticmethod
        def articore_runtime_last_error():
            return b"ok"

    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = Library()
    runtime._ptr = 1
    runtime._trajectory_settling_timeout_s = settling_timeout_s

    result = runtime.wait_trajectory(7)

    assert result.status is TrajectoryStatus.COMPLETED
    assert timeouts == [expected_timeout_ms]


def test_native_trajectory_supports_smooth_replace_and_cancel() -> None:
    calls: list[tuple[object, ...]] = []

    class Library:
        @staticmethod
        def articore_runtime_start_joint_trajectory_ex(
            _runtime,
            _targets,
            count,
            profile,
            replace_policy,
        ) -> int:
            calls.append(
                ("start-ex", int(count), int(profile), int(replace_policy))
            )
            return 41

        @staticmethod
        def articore_runtime_cancel_trajectory(_runtime, trajectory_id) -> int:
            calls.append(("cancel", int(trajectory_id)))
            return 0

        @staticmethod
        def articore_runtime_last_error():
            return b"ok"

    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = Library()
    runtime._ptr = 1
    runtime._trajectory_target_array = None
    motor = SimpleNamespace(_ptr=0x123)

    trajectory_id = runtime.start_joint_trajectory(
        ((motor, 0.5, 1.0),),
        profile="min_jerk",
        replace=True,
    )
    runtime.cancel_trajectory(trajectory_id)

    assert trajectory_id == 41
    assert calls == [("start-ex", 1, 1, 1), ("cancel", 41)]


class FakeRuntimeLibrary:
    def articore_runtime_get_health(self, _runtime, output) -> int:
        health = ctypes.cast(output, ctypes.POINTER(_SafetyHealth)).contents
        health.state = 4
        health.safe_holding = 1
        health.disable_confirmed = 0
        health.last_successful_command_age_ns = 25_000_000
        health.last_fresh_feedback_age_ns = 2_000_000
        health.consecutive_send_failures = 1
        health.consecutive_feedback_failures = 3
        health.left_transport.connected = 1
        health.left_transport.healthy = 1
        health.left_transport.last_feedback_age_ns = 2_000_000
        health.left_transport.tx_frames = 100
        health.left_transport.rx_frames = 99
        health.left_transport.last_tx_age_ns = 1_000_000
        health.left_transport.last_rx_age_ns = 2_000_000
        health.right_transport.connected = 0
        health.right_transport.healthy = 0
        health.right_transport.consecutive_feedback_failures = 3
        health.right_transport.last_feedback_age_ns = (1 << 64) - 1
        health.right_transport.send_errors = 2
        health.right_transport.receive_errors = 4
        health.right_transport.last_tx_age_ns = (1 << 64) - 1
        health.right_transport.last_rx_age_ns = (1 << 64) - 1
        health.right_transport.last_error = b"device disconnected"
        health.gripper_count = 1
        health.grippers[0].available = 1
        health.grippers[0].side = 0
        health.grippers[0].control_state = 4
        health.grippers[0].opening = 375.0
        health.grippers[0].motor_position = 1.25
        health.grippers[0].torque = 0.9
        health.grippers[0].contact_detected = 1
        health.grippers[0].stalled = 1
        health.grippers[0].has_hold_target = 1
        health.grippers[0].hold_target = 1.3
        health.grippers[0].feedback_age_ns = 3_000_000
        health.grippers[0].name = b"left/l-gripper"
        health.fault_reason = b"consecutive feedback failures"
        return 0

    def articore_runtime_last_error(self):
        return b"ok"


class FakeEnableRuntimeLibrary:
    def articore_runtime_enable(self, _runtime, _mode) -> int:
        return -1

    def articore_runtime_get_last_enable_report(self, _runtime, output) -> int:
        report = ctypes.cast(output, ctypes.POINTER(_EnableReport)).contents
        report.success = 0
        report.disable_confirmed = 1
        report.expected_count = 16
        report.enabled_count = 15
        report.missing_count = 1
        report.failure_count = 1
        report.missing_motor_sides[0] = 1
        report.missing_motor_ids[0] = 8
        report.motor_count = 2
        report.motors[0].side = 0
        report.motors[0].can_id = 9
        report.motors[0].status_code = 1
        report.motors[0].has_feedback = 1
        report.motors[0].feedback_fresh = 1
        report.motors[0].enabled = 1
        report.motors[0].name = b"left/l-joint1"
        report.motors[1].side = 1
        report.motors[1].can_id = 8
        report.motors[1].status_code = 0
        report.motors[1].has_feedback = 0
        report.motors[1].feedback_fresh = 0
        report.motors[1].enabled = 0
        report.motors[1].name = b"right/r-gripper"
        report.error = b"right gripper did not enable"
        return 0

    def articore_runtime_last_error(self):
        return b"must not parse this error"


class FakeDisableRuntimeLibrary:
    def __init__(self) -> None:
        self.close_result = -1
        self.calls: list[str] = []

    def articore_runtime_disable(self, _runtime) -> int:
        return -1

    def articore_runtime_close(self, _runtime) -> int:
        self.calls.append("close")
        return self.close_result

    def articore_runtime_free(self, _runtime) -> None:
        self.calls.append("free")

    def articore_runtime_get_last_disable_report(self, _runtime, output) -> int:
        report = ctypes.cast(output, ctypes.POINTER(_DisableReport)).contents
        report.success = 0
        report.barrier_confirmed = 1
        report.expected_count = 16
        report.disabled_count = 15
        report.missing_count = 1
        report.failure_count = 1
        report.retry_count = 1
        report.missing_motor_sides[0] = 1
        report.missing_motor_ids[0] = 8
        report.motor_count = 2
        report.motors[0].side = 0
        report.motors[0].can_id = 9
        report.motors[0].status_code = 0
        report.motors[0].has_feedback = 1
        report.motors[0].feedback_fresh = 1
        report.motors[0].disabled = 1
        report.motors[0].disable_sent = 1
        report.motors[0].retry_sent = 0
        report.motors[0].name = b"left/l-joint1"
        report.motors[1].side = 1
        report.motors[1].can_id = 8
        report.motors[1].status_code = 1
        report.motors[1].has_feedback = 1
        report.motors[1].feedback_fresh = 1
        report.motors[1].disabled = 0
        report.motors[1].disable_sent = 1
        report.motors[1].retry_sent = 1
        report.motors[1].name = b"right/r-gripper"
        report.error = b"right gripper did not confirm disable"
        return 0

    def articore_runtime_last_error(self):
        return b"must not parse this error"


def test_native_health_is_exposed_as_immutable_python_values() -> None:
    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = FakeRuntimeLibrary()
    runtime._ptr = 1

    health = runtime.health

    assert health.state is SafetyState.SAFE_HOLD
    assert health.safe_holding
    assert not health.disable_confirmed
    assert health.last_successful_command_age_s == 0.025
    assert health.left_transport.tx_frames == 100
    assert health.left_transport.last_rx_age_s == 0.002
    assert health.right_transport.connected is False
    assert health.right_transport.last_feedback_age_s is None
    assert health.right_transport.last_tx_age_s is None
    assert health.right_transport.last_error == "device disconnected"
    assert health.fault_reason == "consecutive feedback failures"
    assert health.left_gripper is not None
    assert health.left_gripper.control_state is GripperControlState.HOLDING
    assert health.left_gripper.opening == 375.0
    assert health.left_gripper.contact_detected
    assert health.left_gripper.stalled
    assert health.left_gripper.hold_target == pytest.approx(1.3)
    assert health.left_gripper.feedback_age_s == 0.003
    assert health.right_gripper is None


def test_atomic_enable_failure_exposes_structured_report() -> None:
    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = FakeEnableRuntimeLibrary()
    runtime._ptr = 1

    with pytest.raises(NativeEnableError) as captured:
        runtime.enable("mit")

    report = captured.value.report
    assert isinstance(report, EnableReport)
    assert not report.success
    assert report.disable_confirmed
    assert report.expected_count == 16
    assert report.enabled_count == 15
    assert report.failure_count == 1
    assert report.missing_motors[0].side == 1
    assert report.missing_motors[0].can_id == 8
    assert report.motors[0].name == "left/l-joint1"
    assert report.motors[0].feedback_fresh
    assert report.motors[1].name == "right/r-gripper"
    assert not report.motors[1].enabled
    assert str(captured.value) == "right gripper did not enable"


def test_deterministic_disable_failure_exposes_structured_report() -> None:
    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = FakeDisableRuntimeLibrary()
    runtime._ptr = 1

    with pytest.raises(NativeDisableError) as captured:
        runtime.disable()

    report = captured.value.report
    assert isinstance(report, DisableReport)
    assert not report.success
    assert report.barrier_confirmed
    assert report.expected_count == 16
    assert report.disabled_count == 15
    assert report.failure_count == 1
    assert report.retry_count == 1
    assert report.missing_motors[0].side == 1
    assert report.missing_motors[0].can_id == 8
    assert report.motors[0].name == "left/l-joint1"
    assert report.motors[0].disabled
    assert report.motors[1].name == "right/r-gripper"
    assert report.motors[1].retry_sent
    assert not report.motors[1].disabled
    assert captured.value.operation == "runtime disable"


def test_runtime_close_failure_retains_pointer_and_only_frees_after_success() -> None:
    library = FakeDisableRuntimeLibrary()
    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = library
    runtime._ptr = 0x123

    with pytest.raises(NativeDisableError) as captured:
        runtime.close()

    assert captured.value.operation == "runtime close"
    assert runtime._ptr == 0x123
    assert library.calls == ["close"]

    library.close_result = 0
    runtime.close()

    assert runtime._ptr is None
    assert library.calls == ["close", "close", "free"]


def test_packaged_runtime_creates_a_single_channel_without_python_extension() -> None:
    class Handle:
        def __init__(self, pointer: int) -> None:
            self._ptr = pointer

    motor = Handle(0x201)
    runtime = NativeSafetyRuntime(
        controller_group=Handle(0x100),
        left_controller=Handle(0x101),
        right_controller=None,
        motors=(
            NativeMotorDescriptor(
                motor=motor,
                side=0,
                name="joint1",
            ),
        ),
        joints=(
            NativeJointControlConfig(
                motor=motor,
                lower_position=-1.0,
                upper_position=1.0,
                velocity_limit=2.0,
                torque_limit=5.0,
                mit_kp=20.0,
                mit_kd=1.0,
            ),
        ),
    )
    try:
        runtime.connect()
        health = runtime.health
        assert health.state is SafetyState.READY
        assert health.left_transport.connected
        assert not health.right_transport.connected
    finally:
        runtime.close()
