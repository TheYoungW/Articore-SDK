from __future__ import annotations

from types import SimpleNamespace
from threading import RLock
import ctypes

import pytest

from arx_d_can._motor_abi import (
    CartesianInterpolation,
    CartesianMotionState,
    CartesianMotionStatus,
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
    CCartesianMotionStatus,
    CProductStateV2,
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
    )
    right = ProductArmState(
        tuple(float(i + 30) for i in range(7)),
        tuple(float(i + 40) for i in range(7)),
        tuple(float(i + 50) for i in range(7)),
        (False, False, None, True, True, True, True),
    )
    return ProductState(
        with_grippers,
        left, right,
        ProductGripperState(True, 750.0, 3, True) if with_grippers else None,
        ProductGripperState(True, 250.0, 3, None) if with_grippers else None,
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
        self._cartesian_motion_status = CartesianMotionStatus(
            state=CartesianMotionState.RUNNING,
            motion_id=9,
            superseded_motion_id=0,
            side="left",
            interpolation=CartesianInterpolation.LINEAR,
            speed_percent=20.0,
            elapsed_s=0.5,
            duration_s=1.0,
            progress=0.5,
            target_pose=(0.1, 0.2, 0.3, 0.0, 0.0, 0.0),
            error=None,
        )
        self.fps = 8120.0
        self.max_speed = 50.0

    @property
    def cartesian_motion_status(self) -> CartesianMotionStatus:
        return self._cartesian_motion_status

    def get_cartesian_motion_status(
        self, motion_id: int
    ) -> CartesianMotionStatus:
        self.calls.append(("get_cartesian_motion_status", motion_id))
        return self._cartesian_motion_status

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

    def set_max_speed(self, max_speed_percent) -> None:
        self.max_speed = float(max_speed_percent)
        self.calls.append(("set_max_speed", float(max_speed_percent)))

    def get_max_speed(self) -> float:
        self.calls.append(("get_max_speed",))
        return self.max_speed

    def set_joint_positions(self, positions, velocity=None) -> None:
        if velocity is None:
            self.calls.append(("positions", tuple(positions)))
        else:
            self.calls.append(("positions", tuple(positions), velocity))

    def submit_mit_frame(self, *values) -> None:
        self.calls.append(("mit", *values))

    def set_product_grippers(self, *, left, right, gripper_level, mode) -> None:
        self.calls.append(("grippers", left, right, gripper_level, mode))

    def start_gravity_compensation(self, *, transition_ms: int) -> None:
        self.calls.append(("gravity-start", transition_ms))

    def stop_gravity_compensation(self) -> None:
        self.calls.append(("gravity-stop",))

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

    def move_pose(self, side, target_pose, speed_percent=50.0) -> None:
        self.calls.append(("move_pose", side, tuple(target_pose), speed_percent))

    def move_poses(
        self, left_target_pose, right_target_pose, speed_percent=50.0
    ) -> None:
        self.calls.append((
            "move_poses", tuple(left_target_pose), tuple(right_target_pose),
            speed_percent,
        ))

    def move_linear(self, side, start_pose, end_pose, speed_percent) -> int:
        self.calls.append((
            "move_linear", side, tuple(start_pose), tuple(end_pose),
            speed_percent,
        ))
        return 11

    def move_circular(
        self, side, start_pose, via_pose, end_pose, speed_percent
    ) -> int:
        self.calls.append((
            "move_circular", side, tuple(start_pose), tuple(via_pose),
            tuple(end_pose), speed_percent,
        ))
        return 12

    def cancel_cartesian_motion(self) -> None:
        self.calls.append(("cancel_cartesian_motion",))


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


def test_ordinary_motion_max_speed_uses_canonical_runtime_names(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(control_mode="pv")

    assert robot.get_max_speed() == pytest.approx(50.0)
    robot.set_max_speed(0)
    robot.set_max_speed(100)
    assert robot.get_max_speed() == 100.0
    assert not hasattr(robot, "set_speed")
    assert not hasattr(robot, "get_speed")
    assert robot._runtime.calls == [
        ("get_max_speed",),
        ("set_max_speed", 0.0),
        ("set_max_speed", 100.0),
        ("get_max_speed",),
    ]


def test_max_speed_does_not_apply_to_mit(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")

    with pytest.raises(RuntimeError, match="requires PV mode"):
        robot.set_max_speed(50)
    with pytest.raises(RuntimeError, match="requires PV mode"):
        robot.get_max_speed()
    assert robot._runtime.calls == []


def test_ordinary_position_is_one_fixed_fourteen_axis_frame(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.set_joint_mit(left=range(7), right=range(7, 14), velocity=50)
    assert robot._runtime.calls == [
        ("positions", tuple(float(i) for i in range(14)), 50)
    ]


def test_ordinary_position_default_is_selected_by_native_product(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.set_joint_mit(left=(0,) * 7, right=(0,) * 7)
    assert robot._runtime.calls == [("positions", (0.0,) * 14, 100.0)]


def test_pv_position_uses_only_the_persistent_native_max_speed(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    robot.set_joint_pv(left=(0,) * 7, right=(1,) * 7)
    assert robot._runtime.calls == [
        ("positions", (0.0,) * 7 + (1.0,) * 7)
    ]
    assert not hasattr(robot, "submit_raw_pv")
    with pytest.raises(TypeError, match="velocity"):
        robot.set_joint_pv(
            left=(0,) * 7,
            right=(1,) * 7,
            velocity=50,
        )


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


def test_read_state_uses_one_native_product_snapshot(product_factory) -> None:
    robot = ArxDCanDualArm()
    state = robot.read_state()
    assert state.left.arm.positions == tuple(float(i) for i in range(7))
    assert state.left.arm.enabled == (True, True, True, None, False, False, True)
    assert state.right.arm.positions == tuple(float(i + 30) for i in range(7))
    assert state.right.arm.enabled == (False, False, None, True, True, True, True)
    assert state.left.gripper is not None
    assert state.left.gripper.opening == pytest.approx(750.0)
    assert state.left.gripper.gripper_level == 3
    assert state.left.gripper.enabled is True
    assert state.right.gripper is not None
    assert state.right.gripper.opening == pytest.approx(250.0)
    assert state.right.gripper.enabled is None
    assert state.timestamp_ns == 123456
    assert state.sequence == 77


def test_ctypes_state_v2_maps_feedback_masks_without_per_motor_queries() -> None:
    calls = 0

    def get_state_v2(_pointer, output) -> int:
        nonlocal calls
        calls += 1
        native = ctypes.cast(
            output, ctypes.POINTER(CProductStateV2)
        ).contents
        native.has_grippers = 1
        native.left.enabled_mask = 0b0000101
        native.left.enabled_valid_mask = 0b0000111
        native.right.enabled_mask = 0b0000010
        native.right.enabled_valid_mask = 0b0000011
        native.left_gripper_available = 1
        native.left_gripper_enabled = 0
        native.left_gripper_enabled_valid = 1
        native.right_gripper_available = 1
        native.right_gripper_enabled_valid = 0
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(articore_runtime_get_state_v2=get_state_v2)
    )
    state = runtime.state

    assert calls == 1
    assert state.left.enabled == (True, False, True, None, None, None, None)
    assert state.right.enabled == (False, True, None, None, None, None, None)
    assert state.left_gripper is not None
    assert state.left_gripper.enabled is False
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


def test_cartesian_motion_is_forwarded_as_one_side_native_operation(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    target = (0.3, 0.2, 0.4, 0.0, 0.1, 0.2)
    start = (0.2, 0.1, 0.3, 0.0, 0.0, 0.1)
    via = (0.25, 0.15, 0.35, 0.0, 0.05, 0.1)

    assert robot.move_pose(side="left", target_pose=target) is None
    assert robot.move_poses(
        left_target_pose=target,
        right_target_pose=via,
        speed_percent=25,
    ) is None
    assert robot.move_linear(
        side="right", start_pose=start, end_pose=target, speed_percent=20
    ) == 11
    assert robot.move_circular(
        side="left", start_pose=start, via_pose=via, end_pose=target,
        speed_percent=30,
    ) == 12
    assert robot.cartesian_motion_status.state == "running"
    assert robot.get_cartesian_motion_status(10).motion_id == 9
    robot.cancel_cartesian_motion()

    assert robot._runtime.calls == [
        ("move_pose", 0, target, 50.0),
        ("move_poses", target, via, 25),
        ("move_linear", 1, start, target, 20),
        ("move_circular", 0, start, via, target, 30),
        ("get_cartesian_motion_status", 10),
        ("cancel_cartesian_motion",),
    ]


def test_cartesian_sdk_exposes_no_python_path_or_interpolation_arguments(
    product_factory,
) -> None:
    import inspect

    robot = ArxDCanDualArm(control_mode="pv")
    ptp = inspect.signature(robot.move_pose)
    dual_ptp = inspect.signature(robot.move_poses)
    linear = inspect.signature(robot.move_linear)
    circular = inspect.signature(robot.move_circular)

    assert tuple(ptp.parameters) == ("side", "target_pose", "speed_percent")
    assert ptp.parameters["speed_percent"].default == 50.0
    assert tuple(dual_ptp.parameters) == (
        "left_target_pose", "right_target_pose", "speed_percent",
    )
    assert tuple(linear.parameters) == (
        "side", "start_pose", "end_pose", "speed_percent",
    )
    assert tuple(circular.parameters) == (
        "side", "start_pose", "via_pose", "end_pose", "speed_percent",
    )


def test_ctypes_cartesian_paths_forward_explicit_start_poses() -> None:
    calls: list[tuple] = []

    def move_pose(_runtime, side, target, speed) -> int:
        calls.append(("ptp", side, tuple(target), float(speed)))
        return 0

    def move_poses(_runtime, left, right, speed) -> int:
        calls.append(("dual_ptp", tuple(left), tuple(right), float(speed)))
        return 0

    def move_linear(_runtime, side, start, end, speed, output) -> int:
        calls.append((
            "linear", side, tuple(start), tuple(end), float(speed),
        ))
        ctypes.cast(output, ctypes.POINTER(ctypes.c_uint64)).contents.value = 21
        return 0

    def move_circular(
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
        articore_runtime_move_pose=move_pose,
        articore_runtime_move_poses=move_poses,
        articore_runtime_move_linear_v2=move_linear,
        articore_runtime_move_circular=move_circular,
    ))
    start = (0.1, 0.2, 0.3, 0.0, 0.1, 0.2)
    via = (0.2, 0.3, 0.4, 0.1, 0.2, 0.3)
    end = (0.3, 0.4, 0.5, 0.2, 0.3, 0.4)

    assert runtime.move_pose(0, end, 5) is None
    assert runtime.move_poses(start, end, 7) is None
    assert runtime.move_linear(0, start, end, 10) == 21
    assert runtime.move_circular(1, start, via, end, 20) == 22
    assert calls[0][:2] == ("ptp", 0)
    assert calls[0][2] == pytest.approx(end)
    assert calls[0][3] == 5.0
    assert calls[1][0] == "dual_ptp"
    assert calls[1][1] == pytest.approx(start)
    assert calls[1][2] == pytest.approx(end)
    assert calls[1][3] == 7.0
    assert calls[2][:2] == ("linear", 0)
    assert calls[2][2] == pytest.approx(start)
    assert calls[2][3] == pytest.approx(end)
    assert calls[2][4] == 10.0
    assert calls[3][:2] == ("circular", 1)
    assert calls[3][2] == pytest.approx(start)
    assert calls[3][3] == pytest.approx(via)
    assert calls[3][4] == pytest.approx(end)
    assert calls[3][5] == 20.0


def test_cartesian_motion_does_not_duplicate_native_mode_or_speed_checks(
    product_factory,
) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.move_pose(
        side="left", target_pose=(0.0,) * 6, speed_percent=0
    )
    assert robot._runtime.calls == [
        ("move_pose", 0, (0.0,) * 6, 0),
    ]


def test_ctypes_cartesian_status_maps_all_native_fields() -> None:
    def get_status(_pointer, output) -> int:
        native = ctypes.cast(
            output, ctypes.POINTER(CCartesianMotionStatus)
        ).contents
        native.state = 1
        native.motion_id = 42
        native.superseded_motion_id = 41
        native.side = 1
        native.interpolation = 3
        native.speed_percent = 25.0
        native.elapsed_s = 2.0
        native.duration_s = 2.0
        native.progress = 1.0
        native.target_pose[:] = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
        native.error = b"waiting for physical settling"
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(
            articore_runtime_get_cartesian_motion_status=get_status
        )
    )

    status = runtime.cartesian_motion_status

    assert status.state == "running"
    assert status.progress == pytest.approx(1.0)
    assert status.state != "completed"
    assert status.motion_id == 42
    assert status.superseded_motion_id == 41
    assert status.side == "right"
    assert status.interpolation == "circular"
    assert status.target_pose == pytest.approx((0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    assert status.error == "waiting for physical settling"


def test_ctypes_cartesian_status_queries_one_fifo_motion_id() -> None:
    calls: list[int] = []

    def get_status_v2(_pointer, motion_id, output) -> int:
        calls.append(int(motion_id))
        native = ctypes.cast(
            output, ctypes.POINTER(CCartesianMotionStatus)
        ).contents
        native.state = 5
        native.motion_id = int(motion_id)
        native.side = 0
        native.interpolation = 2
        native.speed_percent = 10.0
        native.duration_s = 3.0
        return 0

    runtime = ArticoreRuntime.__new__(ArticoreRuntime)
    runtime._lock = RLock()
    runtime._ptr = 1
    runtime._runtime_abi = SimpleNamespace(
        lib=SimpleNamespace(
            articore_runtime_get_cartesian_motion_status_v2=get_status_v2
        )
    )

    status = runtime.get_cartesian_motion_status(27)
    assert calls == [27]
    assert status.state is CartesianMotionState.QUEUED
    assert status.interpolation is CartesianInterpolation.LINEAR
    assert status.motion_id == 27
    with pytest.raises(ValueError, match="motion_id"):
        runtime.get_cartesian_motion_status(0)


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
