from __future__ import annotations

from types import SimpleNamespace

import pytest

from arx_d_can import (
    ArxDCanDualArm,
    SafetyHealth,
    SafetyState,
    TransportError,
    TransportHealth,
)
from arx_d_can.driver import CallError
from arx_d_can.sdk.arm import _PreparedJointPositionBatch


def _health(
    state: SafetyState = SafetyState.RUNNING,
    *,
    disable_confirmed: bool = False,
) -> SafetyHealth:
    transport = TransportHealth(True, True, 0, 0, 0.001, None)
    return SafetyHealth(
        state=state,
        fault_reason="injected fault" if state is SafetyState.FAULT else None,
        last_successful_command_age_s=0.001,
        last_fresh_feedback_age_s=0.001,
        consecutive_send_failures=0,
        consecutive_feedback_failures=0,
        left_transport=transport,
        right_transport=transport,
        motor_faults=(),
        unconfirmed_disable_motors=(),
        safe_holding=state is SafetyState.SAFE_HOLD,
        disable_confirmed=disable_confirmed,
    )


def test_default_dual_arm_uses_two_yunyi_profiles() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)

    assert len(robot.left.joint_names) == 7
    assert len(robot.right.joint_names) == 7
    assert robot.left.config.transport == "dm-device"
    assert robot.right.config.transport == "dm-device"
    assert robot.left.config.port == "0"
    assert robot.right.config.port == "1"
    assert robot.left.config.hardware_config_path == robot.right.config.hardware_config_path


def test_dual_arm_only_exposes_tested_modes() -> None:
    assert ArxDCanDualArm(
        control_mode="pv", left_gripper=False, right_gripper=False
    ).left._mode == "pv"
    assert ArxDCanDualArm(
        control_mode="mit", left_gripper=False, right_gripper=False
    ).left._mode == "mit"
    with pytest.raises(ValueError, match="control_mode"):
        ArxDCanDualArm(control_mode="velocity")


def test_explicit_dm_device_uses_physical_channels_zero_and_one() -> None:
    robot = ArxDCanDualArm(
        transport="dm-device",
        left_gripper=False,
        right_gripper=False,
    )

    assert robot.left.config.port == "0"
    assert robot.right.config.port == "1"


def test_dual_send_keeps_left_and_right_commands_separate() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    sent: list[tuple[str, tuple[float, ...]]] = []
    robot.left.send_joint_positions = lambda positions: sent.append(  # type: ignore[method-assign]
        ("left", tuple(positions))
    )
    robot.right.send_joint_positions = lambda positions: sent.append(  # type: ignore[method-assign]
        ("right", tuple(positions))
    )

    robot.send_joint_positions(left=range(7), right=range(7, 14))

    assert sent == [
        ("left", tuple(float(value) for value in range(7))),
        ("right", tuple(float(value) for value in range(7, 14))),
    ]


def test_dual_validates_both_sides_before_sending() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    called = SimpleNamespace(left=False, right=False)
    robot.left.send_joint_positions = lambda _positions: setattr(  # type: ignore[method-assign]
        called, "left", True
    )
    robot.right.send_joint_positions = lambda _positions: setattr(  # type: ignore[method-assign]
        called, "right", True
    )

    with pytest.raises(ValueError, match="right"):
        robot.send_joint_positions(left=range(7), right=range(6))
    assert not called.left
    assert not called.right


def test_connect_creates_one_native_group_and_closes_it_before_arms(
    monkeypatch,
) -> None:
    events: list[object] = []

    class FakeControllerGroup:
        def __init__(self, controllers) -> None:
            events.append(("group-created", tuple(controllers)))

        def close(self) -> None:
            events.append("group-closed")

    monkeypatch.setattr(
        "arx_d_can.sdk.dual_arm.ControllerGroup",
        FakeControllerGroup,
    )
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    left_controller = object()
    right_controller = object()

    def connect(arm, name: str) -> None:
        events.append(f"{name}-connected")
        arm._connected = True

    def close(arm, name: str, *, disable: bool) -> None:
        events.append((f"{name}-closed", disable))
        arm._connected = False

    robot.left.connect = lambda: connect(robot.left, "left")  # type: ignore[method-assign]
    robot.right.connect = lambda: connect(robot.right, "right")  # type: ignore[method-assign]
    robot.left.close = lambda *, disable=True: close(  # type: ignore[method-assign]
        robot.left, "left", disable=disable
    )
    robot.right.close = lambda *, disable=True: close(  # type: ignore[method-assign]
        robot.right, "right", disable=disable
    )
    robot.left._controller_for_parallel_batch = lambda: left_controller  # type: ignore[method-assign]
    robot.right._controller_for_parallel_batch = lambda: right_controller  # type: ignore[method-assign]

    robot.connect()
    robot.connect()
    robot.close(disable=False)

    assert events == [
        "left-connected",
        "right-connected",
        ("group-created", (left_controller, right_controller)),
        "group-closed",
        ("left-closed", False),
        ("right-closed", False),
    ]


@pytest.mark.parametrize(
    ("mode", "method_name"),
    (("pv", "send_pos_vel"), ("mit", "send_mit")),
)
def test_dual_send_uses_one_native_parallel_batch(mode: str, method_name: str) -> None:
    robot = ArxDCanDualArm(
        control_mode=mode,
        left_gripper=False,
        right_gripper=False,
    )
    calls: list[tuple[str, tuple[object, ...]]] = []
    completed: list[str] = []

    class FakeControllerGroup:
        def send_pos_vel(self, commands) -> None:
            calls.append(("send_pos_vel", tuple(commands)))

        def send_mit(self, commands) -> None:
            calls.append(("send_mit", tuple(commands)))

    robot._controller_group = FakeControllerGroup()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = lambda _positions: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch(mode, tuple(range(7)), ("l0", "l1"))
    )
    robot.right._prepare_parallel_joint_positions = lambda _positions: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch(mode, tuple(range(7)), ("r0", "r1"))
    )
    robot.left._complete_parallel_joint_positions = lambda _batch: completed.append(  # type: ignore[method-assign]
        "left"
    )
    robot.right._complete_parallel_joint_positions = lambda _batch: completed.append(  # type: ignore[method-assign]
        "right"
    )

    robot.send_joint_positions(left=range(7), right=range(7, 14))

    assert calls == [(method_name, ("l0", "l1", "r0", "r1"))]
    assert completed == ["left", "right"]


def test_parallel_send_converts_native_error_for_both_arms() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    handled: list[tuple[str, Exception]] = []

    class FailingControllerGroup:
        def send_pos_vel(self, _commands) -> None:
            raise CallError("CH1 motor ID 4 failed")

    batch = _PreparedJointPositionBatch("pv", tuple(range(7)), (object(),))
    robot._controller_group = FailingControllerGroup()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = lambda _positions: batch  # type: ignore[method-assign]
    robot.right._prepare_parallel_joint_positions = lambda _positions: batch  # type: ignore[method-assign]
    robot.left._handle_joint_command_failure = lambda error: (  # type: ignore[method-assign]
        handled.append(("left", error)) or False
    )
    robot.right._handle_joint_command_failure = lambda error: (  # type: ignore[method-assign]
        handled.append(("right", error)) or False
    )

    with pytest.raises(TransportError, match="CH1 motor ID 4") as caught:
        robot.send_joint_positions(left=range(7), right=range(7))

    assert caught.value.operation == "send_pv"
    assert [name for name, _ in handled] == ["left", "right"]
    assert all(error is caught.value for _, error in handled)


def test_dual_send_routes_through_native_safety_runtime() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    calls: list[tuple[object, ...]] = []

    class Runtime:
        health = _health()

        def submit_pos_vel(self, commands) -> None:
            calls.append(tuple(commands))

    class Group:
        def send_pos_vel(self, _commands) -> None:
            raise AssertionError("native safety runtime must own the hot send path")

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot._controller_group = Group()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = lambda _positions: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch("pv", tuple(range(7)), ("l0", "l1"))
    )
    robot.right._prepare_parallel_joint_positions = lambda _positions: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch("pv", tuple(range(7)), ("r0", "r1"))
    )
    robot.left._complete_parallel_joint_positions = lambda _batch: None  # type: ignore[method-assign]
    robot.right._complete_parallel_joint_positions = lambda _batch: None  # type: ignore[method-assign]

    robot.send_joint_positions(left=range(7), right=range(7))

    assert calls == [("l0", "l1", "r0", "r1")]
    assert robot.safety_health.state is SafetyState.RUNNING


def test_native_fault_is_synchronized_before_preparing_next_command() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)

    class Runtime:
        health = _health(SafetyState.FAULT, disable_confirmed=True)

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot._controller_group = object()  # type: ignore[assignment]
    robot.left._connected = True
    robot.right._connected = True

    with pytest.raises(RuntimeError, match="faulted"):
        robot.send_joint_positions(left=range(7), right=range(7))
    assert robot.left.faulted and robot.right.faulted
    assert not robot.left.enabled and not robot.right.enabled


def test_managed_arm_rejects_a_command_that_bypasses_dual_batch() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    robot.left._set_dual_runtime_managed(True)

    with pytest.raises(RuntimeError, match="complete left/right command"):
        robot.left.send_joint_positions(range(7))


def test_dual_gripper_openings_are_one_native_atomic_submission() -> None:
    robot = ArxDCanDualArm()
    calls: list[tuple[tuple[object, float], ...]] = []
    left_motor = object()
    right_motor = object()
    robot.left.robot._motor_map[robot.left.config.gripper.name] = left_motor
    robot.right.robot._motor_map[robot.right.config.gripper.name] = right_motor

    class Runtime:
        def set_gripper_openings(self, targets) -> None:
            calls.append(tuple(targets))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]

    robot.set_grippers(left=-10.0, right=1200.0)

    assert len(calls) == 1
    assert [value for _, value in calls[0]] == [0.0, 1000.0]
    assert calls[0][0][0] is left_motor
    assert calls[0][1][0] is right_motor


def test_dual_gripper_convenience_methods_keep_simple_user_scale() -> None:
    robot = ArxDCanDualArm()
    calls: list[tuple[float, float]] = []
    robot.set_grippers = lambda *, left, right: calls.append((left, right))  # type: ignore[method-assign]

    robot.open_grippers()
    robot.close_grippers()

    assert calls == [(1000.0, 1000.0), (0.0, 0.0)]


def test_custom_end_does_not_submit_disabled_gripper() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=True)
    calls = []
    right_motor = object()
    robot.right.robot._motor_map[robot.right.config.gripper.name] = right_motor

    class Runtime:
        def set_gripper_openings(self, targets) -> None:
            calls.append(tuple(targets))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.set_grippers(left=100.0, right=600.0)

    assert len(calls[0]) == 1
    assert calls[0][0][1] == 600.0
    assert calls[0][0][0] is right_motor


def test_managed_single_arm_gripper_cannot_bypass_cpp_runtime() -> None:
    robot = ArxDCanDualArm()
    robot.left._connected = True
    robot.left._set_dual_runtime_managed(True)

    with pytest.raises(RuntimeError, match="set_grippers"):
        robot.left.set_gripper(500.0)


def test_close_stops_native_runtime_before_group_and_controllers() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    events: list[str] = []

    class Runtime:
        def close(self) -> None:
            events.append("runtime")

    class Group:
        def close(self) -> None:
            events.append("group")

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot._controller_group = Group()  # type: ignore[assignment]
    robot.left._connected = True
    robot.right._connected = True
    robot.left.close = lambda *, disable=True: events.append("left")  # type: ignore[method-assign]
    robot.right.close = lambda *, disable=True: events.append("right")  # type: ignore[method-assign]

    robot.close(disable=False)

    assert events == ["runtime", "group", "left", "right"]
