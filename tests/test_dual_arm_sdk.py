from __future__ import annotations

from types import SimpleNamespace
from threading import RLock
import ctypes

import pytest

from arx_d_can._motor_abi import (
    FeedbackIssueScope,
    JointLimit,
    MotorFeedbackIssue,
    MotionState,
    MotionStatus,
    MotionType,
    OperationError,
    ProductArmState,
    ProductGripperState,
    ProductPose,
    ProductState,
    RuntimeControlMode,
    RuntimeOperation,
    RuntimeTransportHealth,
    SafetyHealth,
    SafetyState,
)
from arx_d_can.sdk.dual_arm import ArxDCanDualArm
from arx_d_can._motor_abi._runtime_abi import (
    CGravityCompensationStatus,
    CMotionStatus,
    CProductState,
    CSafetyHealth,
)
from arx_d_can._motor_abi.runtime import ArticoreRuntime


def _transport(connected: bool = False) -> RuntimeTransportHealth:
    return RuntimeTransportHealth(
        connected=connected,
        healthy=connected,
        consecutive_send_failures=0,
        consecutive_feedback_failures=0,
        last_feedback_age_ns=None,
        tx_frames=0,
        rx_frames=0,
        send_errors=0,
        receive_errors=0,
        last_tx_age_ns=None,
        last_rx_age_ns=None,
        last_error=None,
    )


def _health(state: SafetyState = SafetyState.DISCONNECTED) -> SafetyHealth:
    connected = state is not SafetyState.DISCONNECTED
    return SafetyHealth(
        state=state,
        safe_holding=state is SafetyState.SAFE_HOLD,
        disable_confirmed=state in {
            SafetyState.DISCONNECTED, SafetyState.READY, SafetyState.FAULT,
        },
        last_successful_command_age_ns=None,
        last_fresh_feedback_age_ns=None,
        consecutive_send_failures=0,
        consecutive_feedback_failures=0,
        left_transport=_transport(connected),
        right_transport=_transport(connected),
        grippers=(),
        motor_faults=(),
        unconfirmed_disable=(),
        fault_reason=None,
        last_operation=RuntimeOperation.NONE,
        last_operation_code=OperationError.OK,
        operation_failed_motors=(),
        last_operation_error=None,
    )


def _product_state(with_grippers: bool = True) -> ProductState:
    left = ProductArmState(
        tuple(float(i) for i in range(7)),
        tuple(float(i + 10) for i in range(7)),
        tuple(float(i + 20) for i in range(7)),
        (True, True, True, None, False, False, True),
        tuple(float(i + 60) for i in range(7)),
        tuple(float(i + 70) for i in range(7)),
    )
    right = ProductArmState(
        tuple(float(i + 30) for i in range(7)),
        tuple(float(i + 40) for i in range(7)),
        tuple(float(i + 50) for i in range(7)),
        (False, False, None, True, True, True, True),
        tuple(float(i + 80) for i in range(7)),
        tuple(float(i + 90) for i in range(7)),
    )
    return ProductState(
        with_grippers,
        left, right,
        ProductGripperState(True, 750.0, 3, True, 36.0, 37.0)
        if with_grippers else None,
        ProductGripperState(True, 250.0, 3, None, 38.0, 39.0)
        if with_grippers else None,
        123456, 77,
    )


class FakeRuntime:
    def __init__(self, mode: RuntimeControlMode, with_grippers: bool) -> None:
        self._mode = mode
        self.has_grippers = with_grippers
        self.health = _health()
        self.state = _product_state(with_grippers)
        self.closed = False
        self.calls: list[tuple] = []
        self.gravity_compensation_status = SimpleNamespace(active=False)
        self.bimanual_follow_status = SimpleNamespace(
            active=False, leader="left", follower="right",
        )
        self._motion_status = MotionStatus(
            state=MotionState.RUNNING,
            motion_id=9,
            motion_type=MotionType.CARTESIAN_LINEAR,
            active_segment=0,
            waypoint_count=0,
            elapsed_s=0.5,
            duration_s=1.0,
            progress=0.5,
            error=None,
        )
        self.fps = 8120.0
        self.joint_limits = tuple(
            JointLimit(-float(index + 1), float(index + 1), 5.0)
            for index in range(14)
        )
        self.max_speed = 0.0
        self.max_acceleration = 0.0

    def get_motion_status(self, motion_id: int) -> MotionStatus:
        self.calls.append(("get_motion_status", motion_id))
        return MotionStatus(
            state=self._motion_status.state,
            motion_id=motion_id,
            motion_type=self._motion_status.motion_type,
            active_segment=self._motion_status.active_segment,
            waypoint_count=self._motion_status.waypoint_count,
            elapsed_s=self._motion_status.elapsed_s,
            duration_s=self._motion_status.duration_s,
            progress=self._motion_status.progress,
            error=self._motion_status.error,
        )

    @property
    def control_mode(self) -> RuntimeControlMode:
        return self._mode

    def connect(self) -> None:
        self.calls.append(("connect",))
        self.health = _health(SafetyState.READY)

    def disconnect(self) -> None:
        if self.closed:
            return
        self.calls.append(("disconnect",))
        self.health = _health()
        self.closed = True

    def enable(self, motors=None) -> bool:
        self.calls.append(("enable",) if motors is None else ("enable", tuple(motors)))
        self.health = _health(
            SafetyState.ENABLED if motors is None else SafetyState.PARTIALLY_ENABLED
        )
        return True

    def disable(self, motors=None) -> bool:
        self.calls.append(("disable",) if motors is None else ("disable", tuple(motors)))
        self.health = _health(
            SafetyState.READY if motors is None else SafetyState.PARTIALLY_ENABLED
        )
        return True

    def configure_mode(self, mode: RuntimeControlMode) -> None:
        self.calls.append(("configure_mode", mode))
        self._mode = mode

    def get_joint_limits(self) -> tuple[JointLimit, ...]:
        self.calls.append(("get_joint_limits",))
        return self.joint_limits

    def set_joint_pv(self, positions, velocity) -> None:
        self.calls.append(("pv", tuple(positions), velocity))

    def set_max_speed(self, value: float) -> None:
        self.calls.append(("set_max_speed", value))
        self.max_speed = value

    def get_max_speed(self) -> float:
        self.calls.append(("get_max_speed",))
        return self.max_speed

    def set_max_acceleration(self, value: float) -> None:
        self.calls.append(("set_max_acceleration", value))
        self.max_acceleration = value

    def get_max_acceleration(self) -> float:
        self.calls.append(("get_max_acceleration",))
        return self.max_acceleration

    def set_joint_mit_direct(self, positions) -> None:
        self.calls.append(("mit-direct", tuple(positions)))

    def set_joint_mit_fast_follow(self, positions) -> None:
        self.calls.append(("mit-fast-follow", tuple(positions)))

    def submit_mit_frame(self, *values) -> None:
        self.calls.append(("mit", *values))

    def set_product_grippers(self, *, left, right, gripper_level, mode) -> None:
        self.calls.append(("grippers", left, right, gripper_level, mode))

    def start_gravity_compensation(self, *, transition_ms: int) -> None:
        self.calls.append(("gravity-start", transition_ms))

    def stop_gravity_compensation(self) -> None:
        self.calls.append(("gravity-stop",))

    def start_bimanual_follow(self, leader_side: int) -> None:
        self.calls.append(("bimanual-start", leader_side))

    def stop_bimanual_follow(self) -> None:
        self.calls.append(("bimanual-stop",))

    def estop(self) -> None:
        self.calls.append(("estop",))

    def recover(self) -> None:
        self.calls.append(("recover",))

    def set_zero(self) -> bool:
        self.calls.append(("zero",))
        return True

    def clear_faults(self) -> None:
        self.calls.append(("clear",))

    def get_fps(self) -> float:
        return self.fps

    def get_pose_sample(self, side: int) -> ProductPose:
        values = (0.1 + side, 0.2, 0.3, 0.4, 0.5, 0.6)
        return ProductPose(side, values, 123456, 77)

    def get_pose(self, side: int) -> list[float]:
        return list(self.get_pose_sample(side).values)

    def set_tcp_offset(self, side: int, offset) -> None:
        self.calls.append(("set_tcp_offset", side, tuple(offset)))

    def get_tcp_offset(self, side: int) -> list[float]:
        self.calls.append(("get_tcp_offset", side))
        return [-0.004, 0.0, -0.178, 0.0, 0.0, 0.0]

    def reset_tcp_offset(self, side: int) -> None:
        self.calls.append(("reset_tcp_offset", side))

    def solve_ik(self, left_target_pose, right_target_pose) -> tuple[float, ...]:
        self.calls.append((
            "solve_ik", tuple(left_target_pose), tuple(right_target_pose),
        ))
        return tuple(float(index) for index in range(14))

    def set_pose(
        self, left_target_pose, right_target_pose, speed_percent=50.0
    ) -> None:
        self.calls.append((
            "set_pose", tuple(left_target_pose), tuple(right_target_pose),
            speed_percent,
        ))

    def move_linear_trajectory(
        self, side, start_pose, end_pose, duration_s
    ) -> int:
        self.calls.append((
            "move_linear_trajectory", side, tuple(start_pose), tuple(end_pose),
            duration_s,
        ))
        return 11

    def move_linear_path_trajectory(
        self, side, poses, segment_duration_s
    ) -> int:
        self.calls.append((
            "move_linear_path_trajectory", side,
            tuple(tuple(pose) for pose in poses), segment_duration_s,
        ))
        return 13

    def move_circular_trajectory(
        self, side, start_pose, via_pose, end_pose, duration_s
    ) -> int:
        self.calls.append((
            "move_circular_trajectory", side, tuple(start_pose), tuple(via_pose),
            tuple(end_pose), duration_s,
        ))
        return 12

    def cancel_motion(self, motion_id: int) -> None:
        self.calls.append(("cancel_motion", motion_id))

    def cancel_all_motions(self) -> None:
        self.calls.append(("cancel_all_motions",))


@pytest.fixture
def product_factory(monkeypatch):
    created: list[tuple[RuntimeControlMode, bool, FakeRuntime]] = []

    def create(mode, *, with_grippers=True):
        runtime = FakeRuntime(RuntimeControlMode(mode), bool(with_grippers))
        created.append((RuntimeControlMode(mode), bool(with_grippers), runtime))
        return runtime

    monkeypatch.setattr(
        "arx_d_can.sdk.dual_arm.ArticoreRuntime.create_yunyi", create
    )
    return created


def test_constructor_immediately_owns_one_product_runtime(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    assert product_factory[0][:2] == (RuntimeControlMode.MIT, True)
    assert robot._runtime is product_factory[0][2]
    assert not hasattr(robot, "_left")
    assert not hasattr(robot, "_right")
    assert not hasattr(robot, "_safety_runtime")
    assert not hasattr(robot, "_controller_group")


def test_constructor_rejects_unknown_mode_before_native_create(product_factory) -> None:
    with pytest.raises(ValueError, match="control_mode"):
        ArxDCanDualArm(control_mode="velocity")
    assert product_factory == []


def test_native_factory_uses_abi3_output_pointer(monkeypatch) -> None:
    calls: list[tuple] = []

    def create_yunyi(mode, with_grippers, output):
        calls.append((mode, with_grippers))
        ctypes.cast(output, ctypes.POINTER(ctypes.c_void_p)).contents.value = 456
        return 0

    fake_abi = SimpleNamespace(
        lib=SimpleNamespace(
            articore_runtime_create_yunyi=create_yunyi,
        ),
    )
    monkeypatch.setattr(
        "arx_d_can._motor_abi.runtime.get_runtime_abi", lambda: fake_abi
    )

    runtime = ArticoreRuntime.create_yunyi(
        RuntimeControlMode.PV, with_grippers=True
    )
    try:
        assert runtime._ptr == 456
        assert calls == [(1, 1)]
    finally:
        runtime._ptr = None


def test_lifecycle_is_forwarded_to_the_same_runtime(product_factory) -> None:
    robot = ArxDCanDualArm()
    runtime = robot._runtime
    robot.connect()
    assert robot.connected
    assert robot.enable()
    assert robot.enabled
    assert robot.disable()
    robot.disconnect()
    assert not robot.connected
    assert runtime.calls == [
        ("connect",), ("enable",), ("disable",), ("disconnect",),
    ]
    assert robot._runtime is runtime


def test_ctypes_enable_uses_the_abi_six_single_argument_signature() -> None:
    calls: list[int] = []

    def enable(pointer) -> int:
        calls.append(int(pointer))
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 123
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(articore_runtime_enable=enable)
    )

    assert runtime.enable()
    assert calls == [123]


def test_ctypes_health_maps_per_motor_feedback_diagnostics() -> None:
    def get_health(_pointer, output) -> int:
        native = ctypes.cast(output, ctypes.POINTER(CSafetyHealth)).contents
        native.state = int(SafetyState.FAULT)
        native.safe_holding = 1
        native.disable_confirmed = 0
        native.left_transport.connected = 1
        native.left_transport.healthy = 0
        native.left_transport.last_error = b"left feedback incomplete"
        native.motor_feedback_count = 1
        native.feedback_issue_count = 1
        native.feedback_issue_scope = int(FeedbackIssueScope.SINGLE_MOTOR)
        motor = native.motor_feedback[0]
        motor.side = 0
        motor.can_id = 5
        motor.can_id_valid = 1
        motor.has_feedback = 1
        motor.fresh = 0
        motor.has_state = 1
        motor.values_finite = 1
        motor.status_code = 3
        motor.issues = int(
            MotorFeedbackIssue.STALE | MotorFeedbackIssue.MOTOR_FAULT
        )
        motor.position = 0.25
        motor.velocity = -0.5
        motor.torque = 1.5
        motor.feedback_age_ns = 125_000_000
        motor.update_count = 77
        motor.role = b"left/l-joint5"
        native.motor_fault_count = 1
        native.motor_faults[0].value = b"left/l-joint2"
        native.fault_reason = b"connect detected motor fault"
        native.last_operation = int(RuntimeOperation.SET_POSE)
        native.last_operation_code = int(OperationError.INVALID_STATE)
        native.operation_failed_motor_count = 1
        native.operation_failed_motors[0].value = b"left/l-joint2"
        native.last_operation_error = b"current_state=FAULT"
        native.safety_reason = b"motion rejected"
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(articore_runtime_get_health=get_health)
    )

    health = runtime.health
    assert health.state is SafetyState.FAULT
    assert health.left_transport.last_error == "left feedback incomplete"
    assert health.motor_feedback_count == 1
    assert health.feedback_issue_count == 1
    assert health.feedback_issue_scope is FeedbackIssueScope.SINGLE_MOTOR
    motor = health.motor_feedback[0]
    assert motor.role == "left/l-joint5"
    assert motor.side == 0
    assert motor.can_id == 5
    assert motor.feedback_age_ns == 125_000_000
    assert motor.status_code == 3
    assert motor.issues == (
        MotorFeedbackIssue.STALE | MotorFeedbackIssue.MOTOR_FAULT
    )
    assert motor.position == pytest.approx(0.25)
    assert motor.velocity == pytest.approx(-0.5)
    assert motor.torque == pytest.approx(1.5)
    assert motor.update_count == 77
    assert health.motor_faults == ("left/l-joint2",)
    assert health.fault_reason == "connect detected motor fault"
    assert health.last_operation is RuntimeOperation.SET_POSE
    assert health.last_operation_code is OperationError.INVALID_STATE
    assert health.operation_failed_motors == ("left/l-joint2",)
    assert health.last_operation_error == "current_state=FAULT"
    assert health.safety_reason == "motion rejected"


@pytest.mark.parametrize(
    ("scope", "issue_count"),
    (
        (FeedbackIssueScope.SINGLE_MOTOR, 1),
        (FeedbackIssueScope.MULTIPLE_MOTORS, 2),
        (FeedbackIssueScope.LEFT_CHANNEL, 8),
        (FeedbackIssueScope.RIGHT_CHANNEL, 8),
        (FeedbackIssueScope.BOTH_CHANNELS, 16),
    ),
)
def test_ctypes_health_preserves_native_feedback_issue_scope(
    scope: FeedbackIssueScope,
    issue_count: int,
) -> None:
    def get_health(_pointer, output) -> int:
        native = ctypes.cast(output, ctypes.POINTER(CSafetyHealth)).contents
        native.state = int(SafetyState.DEGRADED)
        native.feedback_issue_count = issue_count
        native.feedback_issue_scope = int(scope)
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(articore_runtime_get_health=get_health)
    )

    health = runtime.health

    assert health.feedback_issue_count == issue_count
    assert health.feedback_issue_scope is scope


def test_ctypes_gravity_status_uses_fixed_fourteen_joint_payload() -> None:
    def get_status(_pointer, output) -> int:
        native = ctypes.cast(
            output, ctypes.POINTER(CGravityCompensationStatus)
        ).contents
        native.phase = 2
        native.active = 1
        native.transition_progress = 1.0
        native.control_cycles = 500
        native.joint_count = 14
        native.gravity_feedforward_torque[:] = tuple(range(14))
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(
            articore_runtime_get_gravity_compensation_status=get_status
        )
    )

    status = runtime.gravity_compensation_status
    assert status.joint_count == 14
    assert status.gravity_feedforward_torque == tuple(float(i) for i in range(14))


def test_mode_and_health_are_read_only_runtime_state(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    runtime = robot._runtime
    assert robot.control_mode == "pv"
    robot.configure_mode("mit")
    assert robot.control_mode == "mit"
    assert robot.get_health() is runtime.health
    assert not hasattr(robot, "safety_health")
    assert runtime.calls == [("configure_mode", RuntimeControlMode.MIT)]


def test_subset_motor_power_is_forwarded_as_one_product_transaction(
    product_factory,
) -> None:
    robot = ArxDCanDualArm()
    roles = ["l-joint4", "r-joint4"]
    assert robot.enable(motors=roles)
    assert robot.enabled
    assert robot.disable(motors=roles)
    assert robot._runtime.calls == [
        ("enable", ("l-joint4", "r-joint4")),
        ("disable", ("l-joint4", "r-joint4")),
    ]


def test_get_fps_returns_latest_runtime_sample(product_factory) -> None:
    robot = ArxDCanDualArm()

    assert robot.get_fps() == 8120.0


def test_pv_global_limits_are_forwarded_without_sdk_scaling(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="pv")

    robot.set_max_speed(1.25)
    assert robot.get_max_speed() == 1.25
    robot.set_max_acceleration(2.5)
    assert robot.get_max_acceleration() == 2.5
    robot.set_max_speed(0.0)
    robot.set_max_acceleration(0.0)

    assert robot._runtime.calls == [
        ("set_max_speed", 1.25),
        ("get_max_speed",),
        ("set_max_acceleration", 2.5),
        ("get_max_acceleration",),
        ("set_max_speed", 0.0),
        ("set_max_acceleration", 0.0),
    ]
    assert not hasattr(robot, "set_speed")
    assert not hasattr(robot, "get_speed")
    assert not hasattr(ArticoreRuntime, "set_joint_mit")


def test_joint_limits_use_fixed_product_names_and_native_values(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(control_mode="mit", with_grippers=True)

    limits = robot.get_joint_limits()

    assert tuple(limits) == tuple(
        [f"l-joint{index}" for index in range(1, 8)]
        + [f"r-joint{index}" for index in range(1, 8)]
    )
    assert limits["l-joint1"] is robot._runtime.joint_limits[0]
    assert limits["r-joint7"] is robot._runtime.joint_limits[13]
    assert limits["l-joint1"].min_angle_rad == -1.0
    assert limits["r-joint7"].max_angle_rad == 14.0
    assert limits["r-joint7"].max_velocity_rad_s == 5.0
    assert robot._runtime.calls == [("get_joint_limits",)]


def test_ordinary_position_is_one_fixed_fourteen_axis_frame(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.set_joint_mit(left=range(7), right=range(7, 14))
    assert robot._runtime.calls == [
        ("mit-direct", tuple(float(i) for i in range(14)))
    ]


def test_fast_follow_position_is_one_fixed_fourteen_axis_frame(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.set_joint_mit_fast_follow(left=(0,) * 7, right=(1,) * 7)
    assert robot._runtime.calls == [
        ("mit-fast-follow", (0.0,) * 7 + (1.0,) * 7)
    ]


@pytest.mark.parametrize("method", ("set_joint_mit", "set_joint_mit_fast_follow"))
def test_public_mit_modes_require_mit_control_mode(product_factory, method) -> None:
    robot = ArxDCanDualArm(control_mode="pv")

    with pytest.raises(RuntimeError, match="requires MIT mode"):
        getattr(robot, method)(left=(0.0,) * 7, right=(0.0,) * 7)

    assert robot._runtime.calls == []


def test_pv_position_forwards_per_command_speed(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    robot.set_joint_pv(left=(0,) * 7, right=(1,) * 7)
    assert robot._runtime.calls == [
        ("pv", (0.0,) * 7 + (1.0,) * 7, 50.0)
    ]
    assert not hasattr(robot, "submit_raw_pv")
    assert not hasattr(robot, "set_realtime_pv")
    robot.set_joint_pv(
        left=(0,) * 7,
        right=(1,) * 7,
        velocity=80,
    )
    assert robot._runtime.calls[-1] == (
        "pv", (0.0,) * 7 + (1.0,) * 7, 80,
    )


def test_ctypes_ordinary_position_uses_explicit_pv_and_new_mit_symbols() -> None:
    calls: list[tuple] = []

    def set_joint_pv(_runtime, positions, count, speed) -> int:
        calls.append(("pv", tuple(positions[:count]), float(speed)))
        return 0

    def set_joint_mit_direct(_runtime, positions, count) -> int:
        calls.append(("mit-direct", tuple(positions[:count])))
        return 0

    def set_joint_mit_fast_follow(_runtime, positions, count) -> int:
        calls.append(("mit-fast-follow", tuple(positions[:count])))
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(lib=SimpleNamespace(
        articore_runtime_set_joint_pv=set_joint_pv,
        articore_runtime_set_joint_mit_direct=set_joint_mit_direct,
        articore_runtime_set_joint_mit_fast_follow=set_joint_mit_fast_follow,
    ))

    runtime.set_joint_pv(range(14), 37)
    runtime.set_joint_mit_direct(range(14))
    runtime.set_joint_mit_fast_follow(range(14))

    expected = tuple(float(value) for value in range(14))
    assert calls == [
        ("pv", expected, 37.0),
        ("mit-direct", expected),
        ("mit-fast-follow", expected),
    ]


def test_ctypes_pv_global_limits_use_scalar_setters_and_float_outputs() -> None:
    values = {"speed": 0.0, "acceleration": 0.0}

    def set_max_speed(_runtime, value) -> int:
        values["speed"] = float(value)
        return 0

    def get_max_speed(_runtime, output) -> int:
        ctypes.cast(output, ctypes.POINTER(ctypes.c_float)).contents.value = (
            values["speed"]
        )
        return 0

    def set_max_acceleration(_runtime, value) -> int:
        values["acceleration"] = float(value)
        return 0

    def get_max_acceleration(_runtime, output) -> int:
        ctypes.cast(output, ctypes.POINTER(ctypes.c_float)).contents.value = (
            values["acceleration"]
        )
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(lib=SimpleNamespace(
        articore_runtime_set_max_speed=set_max_speed,
        articore_runtime_get_max_speed=get_max_speed,
        articore_runtime_set_max_acceleration=set_max_acceleration,
        articore_runtime_get_max_acceleration=get_max_acceleration,
    ))

    runtime.set_max_speed(1.25)
    runtime.set_max_acceleration(2.5)

    assert runtime.get_max_speed() == pytest.approx(1.25)
    assert runtime.get_max_acceleration() == pytest.approx(2.5)


def test_raw_mit_forwards_product_arrays_without_motor_mapping(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.submit_raw_mit(
        left_positions=range(7),
        right_positions=range(7, 14),
        left_velocities=(1,) * 7,
        right_velocities=(2,) * 7,
        kp=3.0,
        kd=(0.5,) * 7,
    )
    call = robot._runtime.calls[0]
    assert call[0] == "mit"
    assert call[1] == tuple(float(i) for i in range(14))
    assert call[2] == (1.0,) * 7 + (2.0,) * 7
    assert call[3] is None
    assert call[4] == (3.0,) * 14
    assert call[5] == (0.5,) * 14


def test_joint_trajectory_interfaces_are_not_public(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="pv")

    assert not hasattr(robot, "start_trajectory")
    assert not hasattr(robot, "move_joint_trajectory")
    assert not hasattr(robot, "get_trajectory_status")
    assert not hasattr(robot, "cancel_trajectory")
    assert not hasattr(ArticoreRuntime, "move_joint_trajectory")


def test_read_state_uses_one_native_product_snapshot(product_factory) -> None:
    robot = ArxDCanDualArm()
    state = robot.read_state()
    assert state.left.arm.positions == tuple(float(i) for i in range(7))
    assert state.left.arm.enabled == (True, True, True, None, False, False, True)
    assert state.left.arm.mos_temperatures == tuple(float(i + 60) for i in range(7))
    assert state.left.arm.rotor_temperatures == tuple(float(i + 70) for i in range(7))
    assert state.right.arm.positions == tuple(float(i + 30) for i in range(7))
    assert state.right.arm.enabled == (False, False, None, True, True, True, True)
    assert state.left.gripper is not None
    assert state.left.gripper.opening == pytest.approx(750.0)
    assert state.left.gripper.gripper_level == 3
    assert state.left.gripper.enabled is True
    assert state.left.gripper.mos_temperature == pytest.approx(36.0)
    assert state.left.gripper.rotor_temperature == pytest.approx(37.0)
    assert state.right.gripper is not None
    assert state.right.gripper.opening == pytest.approx(250.0)
    assert state.right.gripper.enabled is None
    assert state.timestamp_ns == 123456
    assert state.sequence == 77


def test_ctypes_state_maps_feedback_and_temperature_masks() -> None:
    calls = 0

    def get_state(_pointer, output) -> int:
        nonlocal calls
        calls += 1
        native = ctypes.cast(
            output, ctypes.POINTER(CProductState)
        ).contents
        native.has_grippers = 1
        native.left.enabled_mask = 0b0000101
        native.left.enabled_valid_mask = 0b0000111
        native.left.temperature_valid_mask = 0b0000101
        native.left.mos_temperatures[:] = (30, 31, 32, 33, 34, 35, 36)
        native.left.rotor_temperatures[:] = (40, 41, 42, 43, 44, 45, 46)
        native.right.enabled_mask = 0b0000010
        native.right.enabled_valid_mask = 0b0000011
        native.left_gripper_available = 1
        native.left_gripper_enabled = 0
        native.left_gripper_enabled_valid = 1
        native.left_gripper_temperature_valid = 1
        native.left_gripper_mos_temperature = 38.0
        native.left_gripper_rotor_temperature = 39.0
        native.right_gripper_available = 1
        native.right_gripper_enabled_valid = 0
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(articore_runtime_get_state=get_state)
    )
    state = runtime.state

    assert calls == 1
    assert state.left.enabled == (True, False, True, None, None, None, None)
    assert state.left.mos_temperatures == (30.0, None, 32.0, None, None, None, None)
    assert state.left.rotor_temperatures == (40.0, None, 42.0, None, None, None, None)
    assert state.right.enabled == (False, True, None, None, None, None, None)
    assert state.left_gripper is not None
    assert state.left_gripper.enabled is False
    assert state.left_gripper.mos_temperature == pytest.approx(38.0)
    assert state.left_gripper.rotor_temperature == pytest.approx(39.0)
    assert state.right_gripper is not None
    assert state.right_gripper.enabled is None


def test_get_pose_returns_six_values_and_optional_sample_metadata(
    product_factory,
) -> None:
    robot = ArxDCanDualArm()
    assert robot.get_pose("left") == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    right = robot.get_pose_sample("right")
    assert right.values == (1.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    assert right.timestamp_ns == 123456
    assert right.sequence == 77
    with pytest.raises(ValueError, match="side"):
        robot.get_pose("center")


def test_tcp_offset_is_forwarded_to_the_native_runtime(product_factory) -> None:
    robot = ArxDCanDualArm(with_grippers=True)
    custom = (0.01, -0.02, 0.12, 0.1, -0.2, 0.3)
    robot.set_tcp_offset(side="left", offset=custom)
    assert robot.get_tcp_offset(side="right") == [
        -0.004, 0.0, -0.178, 0.0, 0.0, 0.0,
    ]
    robot.reset_tcp_offset(side="left")
    assert robot._runtime.calls == [
        ("set_tcp_offset", 0, custom),
        ("get_tcp_offset", 1),
        ("reset_tcp_offset", 0),
    ]
    with pytest.raises(ValueError, match="side"):
        robot.set_tcp_offset(side="center", offset=custom)


def test_cartesian_motion_is_forwarded_as_native_operation(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    target = (0.3, 0.2, 0.4, 0.0, 0.1, 0.2)
    start = (0.2, 0.1, 0.3, 0.0, 0.0, 0.1)
    via = (0.25, 0.15, 0.35, 0.0, 0.05, 0.1)

    assert robot.solve_ik(
        left_target_pose=target,
        right_target_pose=via,
    ) == (
        tuple(float(index) for index in range(7)),
        tuple(float(index) for index in range(7, 14)),
    )
    assert robot.set_pose(
        left_target_pose=target,
        right_target_pose=via,
        speed_percent=25,
    ) is None
    assert robot.move_linear_trajectory(
        side="right", start_pose=start, end_pose=target, duration_s=20,
    ) == 11
    assert robot.move_linear_trajectory(
        side="left", poses=(start, via, target), duration_s=10,
    ) == 13
    with pytest.raises(ValueError, match="cannot be combined"):
        robot.move_linear_trajectory(
            side="left", start_pose=start, end_pose=target,
            poses=(start, via, target), duration_s=10,
        )
    with pytest.raises(ValueError, match="required when poses is omitted"):
        robot.move_linear_trajectory(side="left", duration_s=10)
    assert robot.move_circular_trajectory(
        side="left", start_pose=start, via_pose=via, end_pose=target,
        duration_s=30,
    ) == 12
    assert robot.get_motion_status(10).motion_id == 10
    robot.cancel_motion(10)
    robot.cancel_all_motions()

    assert robot._runtime.calls == [
        ("solve_ik", target, via),
        ("set_pose", target, via, 25),
        ("move_linear_trajectory", 1, start, target, 20),
        ("move_linear_path_trajectory", 0, (start, via, target), 10),
        ("move_circular_trajectory", 0, start, via, target, 30),
        ("get_motion_status", 10),
        ("cancel_motion", 10),
        ("cancel_all_motions",),
    ]


def test_cartesian_sdk_keeps_one_linear_method_and_native_path_planning(
    product_factory,
) -> None:
    import inspect

    robot = ArxDCanDualArm(control_mode="pv")
    solve_ik = inspect.signature(robot.solve_ik)
    ptp = inspect.signature(robot.set_pose)
    assert not hasattr(robot, "set_poses")
    assert not hasattr(robot, "move_pose")
    assert not hasattr(robot, "move_linear")
    assert not hasattr(robot, "move_circular")
    linear = inspect.signature(robot.move_linear_trajectory)
    circular = inspect.signature(robot.move_circular_trajectory)
    joint_pv = inspect.signature(robot.set_joint_pv)

    assert tuple(joint_pv.parameters) == ("left", "right", "velocity")
    assert joint_pv.parameters["velocity"].default == 50.0
    joint_mit = inspect.signature(robot.set_joint_mit)
    assert tuple(joint_mit.parameters) == ("left", "right")
    joint_mit_fast_follow = inspect.signature(robot.set_joint_mit_fast_follow)
    assert tuple(joint_mit_fast_follow.parameters) == ("left", "right")

    assert tuple(solve_ik.parameters) == (
        "left_target_pose", "right_target_pose",
    )

    assert tuple(ptp.parameters) == (
        "left_target_pose", "right_target_pose", "speed_percent",
    )
    assert ptp.parameters["speed_percent"].default == 50.0
    assert tuple(linear.parameters) == (
        "side", "start_pose", "end_pose", "poses", "duration_s",
    )
    assert tuple(circular.parameters) == (
        "side", "start_pose", "via_pose", "end_pose", "duration_s",
    )


def test_linear_path_rejects_pose_counts_outside_native_2_to_64() -> None:
    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    pose = (0.1, 0.2, 0.3, 0.0, 0.1, 0.2)

    for poses in ((pose,), (pose,) * 65):
        with pytest.raises(ValueError, match="2..64 poses"):
            runtime.move_linear_path_trajectory(0, poses, 3.0)


def test_ctypes_cartesian_paths_forward_explicit_start_poses() -> None:
    calls: list[tuple] = []

    def set_pose(_runtime, left, right, speed) -> int:
        calls.append(("ptp", tuple(left), tuple(right), float(speed)))
        return 0

    def move_linear_trajectory(_runtime, side, start, end, speed, output) -> int:
        calls.append((
            "linear", side, tuple(start), tuple(end), float(speed),
        ))
        ctypes.cast(output, ctypes.POINTER(ctypes.c_uint64)).contents.value = 21
        return 0

    def move_linear_path_trajectory(
        _runtime, side, poses, pose_count, duration, output
    ) -> int:
        calls.append((
            "linear_path", side,
            tuple(poses[index] for index in range(pose_count * 6)),
            pose_count, float(duration),
        ))
        ctypes.cast(output, ctypes.POINTER(ctypes.c_uint64)).contents.value = 23
        return 0

    def move_circular_trajectory(
        _runtime, side, start, via, end, speed, output
    ) -> int:
        calls.append((
            "circular", side, tuple(start), tuple(via), tuple(end),
            float(speed),
        ))
        ctypes.cast(output, ctypes.POINTER(ctypes.c_uint64)).contents.value = 22
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(lib=SimpleNamespace(
        articore_runtime_set_pose=set_pose,
        articore_runtime_move_linear_trajectory=move_linear_trajectory,
        articore_runtime_move_linear_trajectory_with_point_count=(
            move_linear_path_trajectory
        ),
        articore_runtime_move_circular_trajectory=move_circular_trajectory,
    ))
    start = (0.1, 0.2, 0.3, 0.0, 0.1, 0.2)
    via = (0.2, 0.3, 0.4, 0.1, 0.2, 0.3)
    end = (0.3, 0.4, 0.5, 0.2, 0.3, 0.4)

    assert runtime.set_pose(start, end, 7) is None
    assert runtime.move_linear_trajectory(0, start, end, 10) == 21
    assert runtime.move_linear_path_trajectory(0, (start, via, end), 10) == 23
    assert runtime.move_circular_trajectory(1, start, via, end, 20) == 22
    assert calls[0][0] == "ptp"
    assert calls[0][1] == pytest.approx(start)
    assert calls[0][2] == pytest.approx(end)
    assert calls[0][3] == 7.0
    assert calls[1][:2] == ("linear", 0)
    assert calls[1][2] == pytest.approx(start)
    assert calls[1][3] == pytest.approx(end)
    assert calls[1][4] == 10.0
    assert calls[2][:2] == ("linear_path", 0)
    assert calls[2][2] == pytest.approx(start + via + end)
    assert calls[2][3:] == (3, 10.0)
    assert calls[3][:2] == ("circular", 1)
    assert calls[3][2] == pytest.approx(start)
    assert calls[3][3] == pytest.approx(via)
    assert calls[3][4] == pytest.approx(end)
    assert calls[3][5] == 20.0


def test_ctypes_solve_ik_forwards_two_poses_and_returns_fourteen_joints() -> None:
    calls: list[tuple] = []

    def solve_ik(_runtime, left, right, output, count) -> int:
        calls.append((tuple(left), tuple(right), int(count)))
        for index in range(int(count)):
            output[index] = float(index) / 10.0
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(articore_runtime_solve_ik=solve_ik)
    )
    left = (0.1, 0.2, 0.3, 0.0, 0.1, 0.2)
    right = (0.1, -0.2, 0.3, 0.0, 0.1, -0.2)

    result = runtime.solve_ik(left, right)

    assert result == pytest.approx(tuple(index / 10.0 for index in range(14)))
    assert len(calls) == 1
    assert calls[0][0] == pytest.approx(left)
    assert calls[0][1] == pytest.approx(right)
    assert calls[0][2] == 14


@pytest.mark.parametrize(
    "invalid_pose",
    (
        (0.0,) * 5,
        (0.0,) * 7,
        (0.0, 0.0, 0.0, 0.0, 0.0, float("nan")),
        (0.0, 0.0, 0.0, 0.0, float("inf"), 0.0),
    ),
)
def test_solve_ik_rejects_invalid_pose_before_native_call(
    invalid_pose,
) -> None:
    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(
            articore_runtime_solve_ik=lambda *_args: pytest.fail(
                "native solve_ik must not be called"
            )
        )
    )

    with pytest.raises(ValueError, match="pose"):
        runtime.solve_ik(invalid_pose, (0.0,) * 6)


def test_product_solve_ik_requires_exactly_fourteen_native_positions(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    robot._runtime.solve_ik = lambda _left, _right: (0.0,) * 13

    with pytest.raises(RuntimeError, match="returned 13 IK positions; expected 14"):
        robot.solve_ik(
            left_target_pose=(0.0,) * 6,
            right_target_pose=(0.0,) * 6,
        )


def test_product_solve_ik_only_forwards_one_non_motion_call(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    left = (0.4, 0.2, 0.3, 0.0, -1.5, 0.0)
    right = (0.4, -0.2, 0.3, 0.0, -1.5, 0.0)

    left_q, right_q = robot.solve_ik(
        left_target_pose=left,
        right_target_pose=right,
    )

    assert left_q == tuple(float(index) for index in range(7))
    assert right_q == tuple(float(index) for index in range(7, 14))
    assert robot._runtime.calls == [("solve_ik", left, right)]


def test_native_motion_fifo_states_need_no_python_planning_token() -> None:
    next_motion_id = 30
    states = {
        31: (1, 2),
        32: (5, 2),
        33: (5, 2),
        34: (2, 3),
    }

    def move_linear_trajectory(_runtime, _side, _start, _end, _duration, output) -> int:
        nonlocal next_motion_id
        next_motion_id += 1
        ctypes.cast(output, ctypes.POINTER(ctypes.c_uint64)).contents.value = (
            next_motion_id
        )
        return 0

    def move_circular_trajectory(
        _runtime, _side, _start, _via, _end, _duration, output
    ) -> int:
        nonlocal next_motion_id
        next_motion_id += 1
        ctypes.cast(output, ctypes.POINTER(ctypes.c_uint64)).contents.value = (
            next_motion_id
        )
        return 0

    def get_status(_runtime, motion_id, output) -> int:
        state, motion_type = states[int(motion_id)]
        native = ctypes.cast(output, ctypes.POINTER(CMotionStatus)).contents
        native.motion_id = int(motion_id)
        native.motion_type = motion_type
        native.state = state
        native.progress = 1.0 if state == 2 else 0.0
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(lib=SimpleNamespace(
        articore_runtime_move_linear_trajectory=move_linear_trajectory,
        articore_runtime_move_circular_trajectory=move_circular_trajectory,
        articore_runtime_get_motion_status=get_status,
    ))
    start = (0.1, 0.2, 0.3, 0.0, 0.1, 0.2)
    via = (0.2, 0.3, 0.4, 0.1, 0.2, 0.3)
    end = (0.3, 0.4, 0.5, 0.2, 0.3, 0.4)

    linear_ids = [runtime.move_linear_trajectory(0, start, end, 10.0) for _ in range(3)]
    linear_states = [runtime.get_motion_status(item).state for item in linear_ids]
    circular_id = runtime.move_circular_trajectory(1, start, via, end, 20.0)

    assert linear_ids == [31, 32, 33]
    assert linear_states == [
        MotionState.RUNNING,
        MotionState.QUEUED,
        MotionState.QUEUED,
    ]
    assert runtime.get_motion_status(circular_id).state is MotionState.COMPLETED


def test_cartesian_motion_does_not_duplicate_native_mode_or_speed_checks(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.set_pose(
        left_target_pose=(0.0,) * 6,
        right_target_pose=(0.0,) * 6,
        speed_percent=0,
    )
    assert robot._runtime.calls == [
        ("set_pose", (0.0,) * 6, (0.0,) * 6, 0),
    ]


def test_ctypes_motion_status_maps_all_native_fields() -> None:
    calls: list[int] = []

    def get_status(_pointer, motion_id, output) -> int:
        calls.append(int(motion_id))
        native = ctypes.cast(
            output, ctypes.POINTER(CMotionStatus)
        ).contents
        native.state = 1
        native.motion_id = 42
        native.motion_type = 3
        native.active_segment = 2
        native.waypoint_count = 8
        native.elapsed_s = 2.0
        native.duration_s = 2.0
        native.progress = 1.0
        native.error = b"waiting for physical settling"
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(
            articore_runtime_get_motion_status=get_status
        )
    )

    status = runtime.get_motion_status(42)

    assert calls == [42]
    assert status.state == "running"
    assert status.progress == pytest.approx(1.0)
    assert status.state != "completed"
    assert status.motion_id == 42
    assert status.motion_type is MotionType.CARTESIAN_CIRCULAR
    assert status.active_segment == 2
    assert status.waypoint_count == 8
    assert status.error == "waiting for physical settling"


def test_ctypes_motion_status_queries_one_fifo_motion_id() -> None:
    calls: list[int] = []

    def get_status(_pointer, motion_id, output) -> int:
        calls.append(int(motion_id))
        native = ctypes.cast(
            output, ctypes.POINTER(CMotionStatus)
        ).contents
        native.state = 5
        native.motion_id = int(motion_id)
        native.motion_type = 1
        native.duration_s = 3.0
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(
            articore_runtime_get_motion_status=get_status
        )
    )

    status = runtime.get_motion_status(27)
    assert calls == [27]
    assert status.state is MotionState.QUEUED
    assert status.motion_type is MotionType.JOINT_TRAJECTORY
    assert status.motion_id == 27
    with pytest.raises(ValueError, match="motion_id"):
        runtime.get_motion_status(0)


def test_gripperless_product_returns_arm_state_and_none_grippers(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(with_grippers=False)
    state = robot.read_state()
    assert not robot.has_grippers
    assert not state.has_grippers
    assert state.left.arm.positions == tuple(float(i) for i in range(7))
    assert state.right.arm.positions == tuple(float(i + 30) for i in range(7))
    assert state.left.gripper is None
    assert state.right.gripper is None


def test_maintenance_and_gravity_only_delegate(product_factory) -> None:
    robot = ArxDCanDualArm()
    assert robot.set_zero()
    robot.clear_motor_faults()
    robot.start_gravity_compensation(transition_ms=250)
    robot.stop_gravity_compensation()
    robot.estop()
    with pytest.raises(TypeError):
        robot.estop("operator")  # type: ignore[call-arg]
    robot.recover()
    assert robot._runtime.calls == [
        ("zero",), ("clear",), ("gravity-start", 250),
        ("gravity-stop",), ("estop",), ("recover",),
    ]


def test_bimanual_follow_only_selects_leader_and_delegates(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.start_bimanual_follow(leader="left")
    robot.stop_bimanual_follow()
    robot.start_bimanual_follow(leader="RIGHT")
    assert robot.bimanual_follow_status.leader == "left"
    assert robot._runtime.calls == [
        ("bimanual-start", 0),
        ("bimanual-stop",),
        ("bimanual-start", 1),
    ]
    with pytest.raises(ValueError, match="leader"):
        robot.start_bimanual_follow(leader="middle")


def test_grippers_are_submitted_as_one_two_side_product_frame(product_factory) -> None:
    robot = ArxDCanDualArm()
    robot.set_grippers(left=100, right=900, gripper_level=10)
    robot.set_grippers(left=0, right=0, gripper_level=0, mode="direct")
    assert robot._runtime.calls == [
        ("grippers", 100, 900, 10, 0),
        ("grippers", 0, 0, 0, 1),
    ]


def test_gripper_mode_rejects_unknown_value(product_factory) -> None:
    robot = ArxDCanDualArm()
    with pytest.raises(ValueError, match="protected.*direct"):
        robot.set_grippers(
            left=100, right=900, gripper_level=5, mode="automatic"
        )


def test_gripperless_product_still_forwards_safe_noop_to_native(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(with_grippers=False)
    robot.set_grippers(left=100, right=900, gripper_level=3)
    assert robot._runtime.calls == [("grippers", 100, 900, 3, 0)]


def test_disconnect_terminally_closes_the_product_runtime(product_factory) -> None:
    robot = ArxDCanDualArm()
    runtime = robot._runtime
    robot.disconnect()
    robot.disconnect()
    assert runtime.calls == [("disconnect",)]
    assert runtime.closed
    assert not robot.connected
    assert not hasattr(robot, "close")


def test_removed_python_runtime_assembly_helpers_stay_absent(product_factory) -> None:
    robot = ArxDCanDualArm()
    for name in (
        "_runtime_motors", "_runtime_joint_configs",
        "_runtime_gripper_bindings", "_runtime_gravity_bindings",
        "_create_safety_runtime", "_release_runtime",
        "_sync_python_safety_flags", "_prepare_parallel_joint_positions",
        "_ordinary_joint_position_targets",
    ):
        assert not hasattr(robot, name)
