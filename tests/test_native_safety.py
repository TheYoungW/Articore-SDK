from __future__ import annotations

import ctypes
from types import SimpleNamespace
import pytest

from motor_drive_layer import articore_runtime_library_path

from arx_d_can.sdk.native_safety import (
    ARTICORE_CAP_COMMAND_LIFETIME,
    ARTICORE_CAP_DETERMINISTIC_DISABLE,
    ARTICORE_CAP_EFFECTIVE_CONTROL_RATE,
    ARTICORE_CAP_GRIPPER_COMMAND_PROFILES,
    ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS,
    ARTICORE_CAP_JOINT_MIT_POSITION,
    ARTICORE_CAP_JOINT_PV_POSITION,
    ARTICORE_CAP_LAYERED_JOINT_LIMITS,
    ARTICORE_CAP_PROTECTIVE_FAULT_HOLD,
    DisableReport,
    EnableReport,
    GripperControlState,
    GripperForceLevel,
    NativeGripperForceProfile,
    NativeDisableError,
    NativeJointControlConfig,
    NativeJointSafetyLimits,
    NativeMotorDescriptor,
    NativeEnableError,
    NativeSafetyRuntime,
    SafetyState,
    _DisableReport,
    _EnableReport,
    _SafetyHealth,
    _GripperCommand,
    _GripperForceProfile,
    _JointSafetyLimits,
    _JointMitTarget,
    _JointPvTarget,
)


def test_packaged_runtime_exposes_required_abi_2_1_capabilities() -> None:
    library = ctypes.CDLL(articore_runtime_library_path())
    library.articore_runtime_abi_version.restype = ctypes.c_uint32
    library.articore_runtime_capabilities.restype = ctypes.c_uint64

    version = int(library.articore_runtime_abi_version())
    assert (version >> 16, version & 0xFFFF) == (2, 1)
    required = (
        ARTICORE_CAP_COMMAND_LIFETIME
        | ARTICORE_CAP_PROTECTIVE_FAULT_HOLD
        | ARTICORE_CAP_DETERMINISTIC_DISABLE
        | ARTICORE_CAP_LAYERED_JOINT_LIMITS
        | ARTICORE_CAP_GRIPPER_COMMAND_PROFILES
        | ARTICORE_CAP_GRIPPER_FORCE_10_LEVELS
        | ARTICORE_CAP_JOINT_MIT_POSITION
        | ARTICORE_CAP_JOINT_PV_POSITION
        | ARTICORE_CAP_EFFECTIVE_CONTROL_RATE
    )
    assert int(library.articore_runtime_capabilities()) & required == required


def test_packaged_runtime_abi_2_1_has_no_native_trajectory_api() -> None:
    library = ctypes.CDLL(articore_runtime_library_path())

    for name in (
        "articore_runtime_configure_trajectory_execution",
        "articore_runtime_start_joint_trajectory_report",
        "articore_runtime_get_trajectory",
        "articore_runtime_wait_trajectory",
        "articore_runtime_cancel_trajectory",
    ):
        assert not hasattr(library, name)


def test_native_raw_submit_is_always_streaming() -> None:
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

    runtime.submit_pos_vel([SimpleNamespace(motor=motor, pos=0.1, vlim=1.0)])
    runtime.submit_mit(
        [SimpleNamespace(motor=motor, pos=0.1, vel=0.0, kp=10.0, kd=1.0, tau=0.0)]
    )

    assert calls == [("pv", 1), ("mit", 1)]


def test_native_runtime_queries_effective_control_rate() -> None:
    class Library:
        @staticmethod
        def articore_runtime_get_control_hz(_runtime, output) -> int:
            output._obj.value = 400
            return 0

        @staticmethod
        def articore_runtime_last_error():
            return b"ok"

    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = Library()
    runtime._ptr = 1

    assert runtime._get_control_hz() == pytest.approx(400.0)


def test_native_ordinary_joint_positions_use_abi_2_1_entry_points() -> None:
    captured: list[tuple[str, list[tuple[int, float]], float]] = []

    class Library:
        @staticmethod
        def articore_runtime_set_joint_mit(_runtime, values, count, velocity) -> int:
            captured.append(
                (
                    "mit",
                    [
                        (int(values[index].struct_size), float(values[index].target_position))
                        for index in range(int(count))
                    ],
                    float(getattr(velocity, "value", velocity)),
                )
            )
            return 0

        @staticmethod
        def articore_runtime_set_joint_pv(_runtime, values, count, velocity) -> int:
            captured.append(
                (
                    "pv",
                    [
                        (int(values[index].struct_size), float(values[index].target_position))
                        for index in range(int(count))
                    ],
                    float(getattr(velocity, "value", velocity)),
                )
            )
            return 0

        @staticmethod
        def articore_runtime_last_error():
            return b"ok"

    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = Library()
    runtime._ptr = 1
    runtime._joint_mit_target_array = None
    runtime._joint_pv_target_array = None
    motors = (SimpleNamespace(_ptr=0x101), SimpleNamespace(_ptr=0x102))

    runtime.set_joint_mit(((motors[0], 0.2), (motors[1], -0.3)), 1.5)
    runtime.set_joint_pv(((motors[0], 0.4), (motors[1], -0.5)), 2.0)

    assert captured == [
        (
            "mit",
            [
                (ctypes.sizeof(_JointMitTarget), pytest.approx(0.2)),
                (ctypes.sizeof(_JointMitTarget), pytest.approx(-0.3)),
            ],
            pytest.approx(1.5),
        ),
        (
            "pv",
            [
                (ctypes.sizeof(_JointPvTarget), pytest.approx(0.4)),
                (ctypes.sizeof(_JointPvTarget), pytest.approx(-0.5)),
            ],
            pytest.approx(2.0),
        ),
    ]


def test_abi_2_1_joint_and_gripper_configuration_structures() -> None:
    captured: dict[str, list[ctypes.Structure]] = {"limits": [], "profiles": []}

    class Library:
        @staticmethod
        def articore_runtime_configure_joint_safety_limits(
            _runtime, values, count
        ) -> int:
            for index in range(int(count)):
                captured["limits"].append(values[index])
            return 0

        @staticmethod
        def articore_runtime_configure_gripper_force_profiles(
            _runtime, values, count
        ) -> int:
            for index in range(int(count)):
                captured["profiles"].append(values[index])
            return 0

        @staticmethod
        def articore_runtime_last_error():
            return b"ok"

    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = Library()
    runtime._ptr = 1
    joint = SimpleNamespace(_ptr=0x101)
    gripper = SimpleNamespace(_ptr=0x102)
    runtime._configure_joint_safety_limits(
        (
            NativeJointSafetyLimits(
                joint, -1.0, 1.0, -0.9, 0.9, 0.1, 2.0
            ),
        ),
    )
    runtime._configure_gripper_force_profiles(
        tuple(
            NativeGripperForceProfile(
                gripper,
                level,
                0.3 + 0.2 * int(level),
                1.2 + 0.2 * int(level),
                4.0,
                0.5,
                2.0,
                0.5,
            )
            for level in GripperForceLevel
        ),
    )

    assert captured["limits"][0].struct_size == ctypes.sizeof(_JointSafetyLimits)
    assert captured["limits"][0].soft_upper_position == pytest.approx(0.9)
    assert [value.force_level for value in captured["profiles"]] == list(
        range(1, 11)
    )
    assert all(
        value.struct_size == ctypes.sizeof(_GripperForceProfile)
        for value in captured["profiles"]
    )


def test_abi_2_1_gripper_command_is_atomic_profiled_submission() -> None:
    captured: list[_GripperCommand] = []

    class Library:
        @staticmethod
        def articore_runtime_set_gripper_commands(_runtime, values, count) -> int:
            captured.extend(values[index] for index in range(int(count)))
            return 0

        @staticmethod
        def articore_runtime_last_error():
            return b"ok"

    runtime = NativeSafetyRuntime.__new__(NativeSafetyRuntime)
    runtime._lib = Library()
    runtime._ptr = 1
    runtime._gripper_command_array = None
    runtime.set_gripper_commands(
        (
            (SimpleNamespace(_ptr=0x201), 0.0, 250.0, GripperForceLevel.LEVEL_2),
            (SimpleNamespace(_ptr=0x202), 800.0, 900.0, GripperForceLevel.LEVEL_9),
        )
    )

    assert [value.struct_size for value in captured] == [
        ctypes.sizeof(_GripperCommand),
        ctypes.sizeof(_GripperCommand),
    ]
    assert [value.opening for value in captured] == pytest.approx([0.0, 800.0])
    assert [value.speed for value in captured] == pytest.approx([250.0, 900.0])
    assert [value.force_level for value in captured] == [2, 9]


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
        joint_safety_limits=(
            NativeJointSafetyLimits(
                motor=motor,
                hard_lower_position=-1.0,
                hard_upper_position=1.0,
                soft_lower_position=-0.9,
                soft_upper_position=0.9,
                soft_limit_braking_zone=0.1,
                braking_acceleration=2.0,
            ),
        ),
    )
    try:
        runtime.connect()
        assert runtime._get_control_hz() == pytest.approx(500.0)
        health = runtime.health
        assert health.state is SafetyState.READY
        assert health.left_transport.connected
        assert not health.right_transport.connected
    finally:
        runtime.close()
