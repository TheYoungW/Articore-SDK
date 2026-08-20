from __future__ import annotations

from types import SimpleNamespace
from threading import RLock
import ctypes

import pytest

from arx_d_can._motor_abi import (
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
from arx_d_can._motor_abi._runtime_abi import CProductStateV2
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
        self.fps = 8120.0

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

    def set_joint_positions(self, positions, velocity) -> None:
        self.calls.append(("positions", tuple(positions), velocity))

    def submit_mit_frame(self, *values) -> None:
        self.calls.append(("mit", *values))

    def submit_pv_frame(self, *values) -> None:
        self.calls.append(("pv", *values))

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
    assert robot.safety_health is runtime.health
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


def test_ordinary_position_is_one_fixed_fourteen_axis_frame(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.set_joint_mit(left=range(7), right=range(7, 14), velocity=50)
    assert robot._runtime.calls == [
        ("positions", tuple(float(i) for i in range(14)), 50.0)
    ]


def test_ordinary_position_default_is_selected_by_native_product(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    robot.set_joint_mit(left=(0,) * 7, right=(0,) * 7)
    assert robot._runtime.calls == [("positions", (0.0,) * 14, 100.0)]


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


def test_raw_pv_is_one_fixed_product_frame(product_factory) -> None:
    robot = ArxDCanDualArm(control_mode="pv")
    robot.submit_raw_pv(
        left_positions=(0,) * 7,
        right_positions=(1,) * 7,
        left_velocity_limits=(2,) * 7,
        right_velocity_limits=(3,) * 7,
    )
    assert robot._runtime.calls == [
        ("pv", (0.0,) * 7 + (1.0,) * 7, (2.0,) * 7 + (3.0,) * 7)
    ]


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
