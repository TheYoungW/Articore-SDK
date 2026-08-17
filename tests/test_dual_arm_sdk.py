from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from arx_d_can import (
    ArxDCanDualArm,
    GripperForceLevel,
    RuntimeTransportHealth,
    SafetyHealth,
    SafetyState,
)
from arx_d_can.driver import CallError, damiao_model_limits
from arx_d_can.sdk.arm import _PreparedJointPositionBatch


VALID_POSITIONS = (0.0,) * 7
VALID_LEFT_POSITIONS = (0.0, 0.1, 0.2, 0.1, 0.2, 0.1, 0.2)
VALID_RIGHT_POSITIONS = (0.0, -0.1, -0.2, 0.1, -0.2, -0.1, -0.2)


def _health(
    state: SafetyState = SafetyState.RUNNING,
    *,
    disable_confirmed: bool = False,
) -> SafetyHealth:
    transport = RuntimeTransportHealth(
        True, True, 0, 0, 1_000_000, 0, 0, 0, 0, None, None, None
    )
    return SafetyHealth(
        state=state,
        safe_holding=state is SafetyState.SAFE_HOLD,
        disable_confirmed=disable_confirmed,
        last_successful_command_age_ns=1_000_000,
        last_fresh_feedback_age_ns=1_000_000,
        consecutive_send_failures=0,
        consecutive_feedback_failures=0,
        left_transport=transport,
        right_transport=transport,
        grippers=(),
        motor_faults=(),
        unconfirmed_disable=(),
        fault_reason="injected fault" if state is SafetyState.FAULT else None,
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
    assert robot.left._mode == "mit"
    assert robot.right._mode == "mit"


def test_dual_arm_keeps_runtime_effective_control_rate_internal() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    assert not hasattr(robot, "control_hz")
    assert robot._effective_control_hz == pytest.approx(500.0)

    class Runtime:
        control_hz = 400

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    assert robot._effective_control_hz == pytest.approx(400.0)


def test_dual_arm_only_exposes_tested_modes() -> None:
    assert ArxDCanDualArm(
        control_mode="pv", left_gripper=False, right_gripper=False
    ).left._mode == "pv"
    assert ArxDCanDualArm(
        control_mode="mit", left_gripper=False, right_gripper=False
    ).left._mode == "mit"
    with pytest.raises(ValueError, match="control_mode"):
        ArxDCanDualArm(control_mode="velocity")


def test_dual_configure_mode_switches_both_disabled_arms() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    calls: list[tuple[str, str]] = []

    class Runtime:
        health = _health(SafetyState.READY, disable_confirmed=True)

        def close(self) -> None:
            calls.append(("runtime", "close"))

    robot.left._connected = True
    robot.right._connected = True
    robot._controller_group = object()  # type: ignore[assignment]
    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left.configure_mode = lambda mode: calls.append(("left", mode))  # type: ignore[method-assign]
    robot.right.configure_mode = lambda mode: calls.append(("right", mode))  # type: ignore[method-assign]
    robot.left._controller_for_parallel_batch = lambda: object()  # type: ignore[method-assign]
    robot.right._controller_for_parallel_batch = lambda: object()  # type: ignore[method-assign]
    robot._create_safety_runtime = lambda *_args: Runtime()  # type: ignore[method-assign]

    robot.configure_mode("mit")

    assert calls == [
        ("runtime", "close"),
        ("left", "mit"),
        ("right", "mit"),
    ]


def test_dual_configure_mode_rejects_switch_while_enabled() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)

    class Runtime:
        health = _health(SafetyState.RUNNING)

    robot.left._connected = True
    robot.right._connected = True
    robot._safety_runtime = Runtime()  # type: ignore[assignment]

    with pytest.raises(RuntimeError, match="while the dual arm is enabled"):
        robot.configure_mode("pv")


def test_explicit_dm_device_uses_physical_channels_zero_and_one() -> None:
    robot = ArxDCanDualArm(
        transport="dm-device",
        left_gripper=False,
        right_gripper=False,
    )

    assert robot.left.config.port == "0"
    assert robot.right.config.port == "1"


def test_dual_mit_enable_configures_both_sides_before_atomic_runtime_enable() -> None:
    robot = ArxDCanDualArm(
        control_mode="mit",
        left_gripper=False,
        right_gripper=False,
    )
    calls: list[tuple[str, object]] = []

    class Runtime:
        health = _health(SafetyState.READY, disable_confirmed=True)

        def enable(self, mode: str) -> None:
            calls.append(("runtime", mode))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._configure = lambda: calls.append(("configure-left", None))  # type: ignore[method-assign]
    robot.right._configure = lambda: calls.append(("configure-right", None))  # type: ignore[method-assign]
    robot.enable()

    assert calls == [
        ("configure-left", None),
        ("configure-right", None),
        ("runtime", 2),
    ]


def test_connect_creates_one_native_group_and_closes_it_before_arms(
    monkeypatch,
) -> None:
    events: list[object] = []

    class FakeControllerGroup:
        def __init__(self, controllers) -> None:
            events.append(("group-created", tuple(controllers)))

        def close(self) -> None:
            events.append("group-closed")

    class Runtime:
        def close(self) -> None:
            events.append("runtime-closed")

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

    def close(arm, name: str) -> None:
        events.append(f"{name}-closed")
        arm._connected = False

    robot.left.connect = lambda: connect(robot.left, "left")  # type: ignore[method-assign]
    robot.right.connect = lambda: connect(robot.right, "right")  # type: ignore[method-assign]
    robot.left.close = lambda: close(robot.left, "left")  # type: ignore[method-assign]
    robot.right.close = lambda: close(robot.right, "right")  # type: ignore[method-assign]
    robot.left._controller_for_parallel_batch = lambda: left_controller  # type: ignore[method-assign]
    robot.right._controller_for_parallel_batch = lambda: right_controller  # type: ignore[method-assign]
    robot._create_safety_runtime = lambda *_args: Runtime()  # type: ignore[method-assign]

    robot.connect()
    robot.connect()
    robot.close()

    assert events == [
        "left-connected",
        "right-connected",
        ("group-created", (left_controller, right_controller)),
        "runtime-closed",
        "group-closed",
        "left-closed",
        "right-closed",
    ]


@pytest.mark.parametrize(
    ("mode", "method_name"),
    (("pv", "submit_pv"), ("mit", "submit_mit")),
)
def test_dual_send_uses_one_native_parallel_batch(mode: str, method_name: str) -> None:
    robot = ArxDCanDualArm(
        control_mode=mode,
        left_gripper=False,
        right_gripper=False,
    )
    calls: list[tuple[str, tuple[object, ...]]] = []

    class Runtime:
        health = _health()

        def submit_pv(self, commands) -> None:
            calls.append(("submit_pv", tuple(commands)))

        def submit_mit(self, commands) -> None:
            calls.append(("submit_mit", tuple(commands)))

    robot._controller_group = object()  # type: ignore[assignment]
    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = lambda _positions, **_kwargs: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch(
            mode,
            tuple(
                SimpleNamespace(
                    motor=object(), pos=0.0, vlim=1.0,
                    vel=0.0, kp=1.0, kd=0.1, tau=0.0,
                )
                for _ in range(2)
            ),
        )
    )
    robot.right._prepare_parallel_joint_positions = lambda _positions, **_kwargs: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch(
            mode,
            tuple(
                SimpleNamespace(
                    motor=object(), pos=0.0, vlim=1.0,
                    vel=0.0, kp=1.0, kd=0.1, tau=0.0,
                )
                for _ in range(2)
            ),
        )
    )
    robot._submit_joint_positions(left=VALID_POSITIONS, right=VALID_POSITIONS)

    assert len(calls) == 1
    assert calls[0][0] == method_name
    assert len(calls[0][1]) == 4


def test_dual_runtime_connect_failure_releases_runtime_lease(monkeypatch) -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    events: list[str] = []

    class Runtime:
        def __init__(self, **_kwargs) -> None:
            events.append("create")

        def configure_joints(self, _configs) -> None:
            events.append("joints")

        def configure_joint_safety_limits(self, _limits) -> None:
            events.append("limits")

        def configure_gripper_products(self, _bindings) -> None:
            events.append("grippers")

        def connect(self) -> None:
            events.append("connect")
            raise RuntimeError("injected connect failure")

        def close(self) -> None:
            events.append("close")

    monkeypatch.setattr("arx_d_can.sdk.dual_arm.ArticoreRuntime", Runtime)
    robot._runtime_motors = lambda: ()  # type: ignore[method-assign]
    robot._runtime_joint_configs = lambda: ()  # type: ignore[method-assign]
    robot._runtime_joint_limits = lambda: ()  # type: ignore[method-assign]
    robot._runtime_gripper_bindings = lambda: ()  # type: ignore[method-assign]
    group = SimpleNamespace(_ptr=1)
    left_controller = SimpleNamespace(_ptr=1)
    right_controller = SimpleNamespace(_ptr=1)

    with pytest.raises(RuntimeError, match="injected connect failure"):
        robot._create_safety_runtime(
            group,
            left_controller,
            right_controller,
        )

    assert events == [
        "create",
        "joints",
        "limits",
        "grippers",
        "connect",
        "close",
    ]


def test_dual_raw_send_validates_both_sides_before_submission() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    submitted: list[object] = []

    class Runtime:
        health = _health()

        @staticmethod
        def submit_mit(commands, **_kwargs) -> None:
            submitted.append(commands)

    robot._controller_group = object()  # type: ignore[assignment]
    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = lambda *_args, **_kwargs: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch("mit", ("left",))
    )
    robot.right._prepare_parallel_joint_positions = lambda *_args, **_kwargs: (  # type: ignore[method-assign]
        (_ for _ in ()).throw(ValueError("r-joint6 exceeds allowed limit"))
    )
    right = list(VALID_POSITIONS)
    right[5] = 0.8

    with pytest.raises(ValueError, match=r"r-joint6.*allowed"):
        robot._submit_joint_positions(left=VALID_POSITIONS, right=right)

    assert submitted == []


def test_dual_mit_send_forwards_each_side_parameters() -> None:
    robot = ArxDCanDualArm(
        control_mode="mit",
        left_gripper=False,
        right_gripper=False,
    )
    prepared: dict[str, tuple[tuple[float, ...], dict[str, object]]] = {}
    submitted: list[tuple[object, ...]] = []

    class Runtime:
        health = _health()

        def submit_mit(self, commands) -> None:
            submitted.append(tuple(commands))

    def prepare(side: str, positions, **kwargs):
        prepared[side] = (tuple(positions), kwargs)
        return _PreparedJointPositionBatch(
            "mit",
            (
                SimpleNamespace(
                    motor=object(), pos=0.0, vel=0.0,
                    kp=1.0, kd=0.1, tau=0.0,
                ),
            ),
        )

    robot._controller_group = object()  # type: ignore[assignment]
    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = (  # type: ignore[method-assign]
        lambda positions, **kwargs: prepare("left", positions, **kwargs)
    )
    robot.right._prepare_parallel_joint_positions = (  # type: ignore[method-assign]
        lambda positions, **kwargs: prepare("right", positions, **kwargs)
    )

    robot._submit_joint_positions(
        left=VALID_POSITIONS,
        right=VALID_POSITIONS,
        left_velocities=range(10, 17),
        right_velocities=range(20, 27),
        left_torques=range(30, 37),
        right_torques=range(40, 47),
        left_mit_kp=range(50, 57),
        right_mit_kp=range(60, 67),
        left_mit_kd=range(70, 77),
        right_mit_kd=range(80, 87),
    )

    assert prepared["left"] == (
        VALID_POSITIONS,
        {
            "velocities": range(10, 17),
            "velocity_limits": None,
            "torques": range(30, 37),
            "mit_kp": range(50, 57),
            "mit_kd": range(70, 77),
            "enforce_position_limits": True,
        },
    )
    assert prepared["right"] == (
        VALID_POSITIONS,
        {
            "velocities": range(20, 27),
            "velocity_limits": None,
            "torques": range(40, 47),
            "mit_kp": range(60, 67),
            "mit_kd": range(80, 87),
            "enforce_position_limits": True,
        },
    )
    assert len(submitted) == 1
    assert len(submitted[0]) == 2
    assert all(command.position == 0.0 for command in submitted[0])


def test_public_dual_raw_mit_limits_resultant_torque_to_80_percent() -> None:
    robot = ArxDCanDualArm(
        control_mode="mit",
        left_gripper=False,
        right_gripper=False,
    )
    captured: dict[str, object] = {}
    robot._submit_joint_positions = (  # type: ignore[method-assign]
        lambda **kwargs: captured.update(kwargs)
    )
    feedback_arm = SimpleNamespace(
        arm=SimpleNamespace(
            positions=(0.0,) * 7,
            velocities=(0.0,) * 7,
        )
    )
    robot.read_cached_state = lambda: SimpleNamespace(  # type: ignore[method-assign]
        left=feedback_arm,
        right=feedback_arm,
    )
    targets = (0.3,) * 7
    zeros = (0.0,) * 7

    robot.submit_raw_mit(
        left_positions=targets,
        right_positions=targets,
        left_velocities=zeros,
        right_velocities=zeros,
        kp=400.0,
        kd=0.0,
        left_feedforward_torques=zeros,
        right_feedforward_torques=zeros,
    )

    assert captured["left"] == targets
    assert captured["right"] == targets
    assert captured["left_velocities"] == zeros
    assert captured["right_velocities"] == zeros
    assert captured["left_torques"] == zeros
    assert captured["right_torques"] == zeros

    for arm, prefix in ((robot.left, "left"), (robot.right, "right")):
        sent_kp = captured[f"{prefix}_mit_kp"]
        sent_kd = captured[f"{prefix}_mit_kd"]
        assert sent_kd == zeros
        assert any(value < 400.0 for value in sent_kp)
        for index, joint in enumerate(arm.config.arm_joints):
            _, _, native_torque = damiao_model_limits(joint.model)
            configured_torque = joint.torque_range or native_torque
            resultant = (
                configured_torque
                / native_torque
                * sent_kp[index]
                * targets[index]
            )
            effort = joint.effort_limit or configured_torque
            assert abs(resultant) <= 0.8 * effort + 1e-9


def test_public_dual_raw_mit_limits_feedforward_before_resultant() -> None:
    robot = ArxDCanDualArm(
        control_mode="mit",
        left_gripper=False,
        right_gripper=False,
    )
    captured: dict[str, object] = {}
    robot._submit_joint_positions = (  # type: ignore[method-assign]
        lambda **kwargs: captured.update(kwargs)
    )
    feedback_arm = SimpleNamespace(
        arm=SimpleNamespace(
            positions=(0.0,) * 7,
            velocities=(0.0,) * 7,
        )
    )
    robot.read_cached_state = lambda: SimpleNamespace(  # type: ignore[method-assign]
        left=feedback_arm,
        right=feedback_arm,
    )

    # joint1 上，P 项约为 -100 N·m，而请求的 +100 N·m tau_ff 几乎抵消它。
    # 如果先按抵消后的合力判断、再由下游单独把 tau_ff 裁到 effort=40 N·m，
    # 最终合力会重新超限。因此必须先约束 tau_ff，再限制完整合力。
    targets = (-0.3375,) * 7
    zeros = (0.0,) * 7
    robot.submit_raw_mit(
        left_positions=targets,
        right_positions=targets,
        left_velocities=zeros,
        right_velocities=zeros,
        kp=400.0,
        kd=0.0,
        left_feedforward_torques=(100.0,) * 7,
        right_feedforward_torques=(100.0,) * 7,
    )

    for arm, prefix in ((robot.left, "left"), (robot.right, "right")):
        sent_kp = captured[f"{prefix}_mit_kp"]
        sent_tau = captured[f"{prefix}_torques"]
        for index, joint in enumerate(arm.config.arm_joints):
            _, _, native_torque = damiao_model_limits(joint.model)
            configured_torque = joint.torque_range or native_torque
            effort = joint.effort_limit or configured_torque
            resultant = (
                configured_torque
                / native_torque
                * sent_kp[index]
                * targets[index]
                + sent_tau[index]
            )
            assert abs(sent_tau[index]) <= effort + 1e-9
            assert abs(resultant) <= 0.8 * effort + 1e-9


def test_public_dual_raw_mit_is_not_used_by_examples() -> None:
    examples = Path(__file__).parents[1] / "arx_d_can" / "examples"
    assert all(
        "submit_raw_mit" not in path.read_text(encoding="utf-8")
        for path in examples.rglob("*.py")
    )


def test_dual_ordinary_pv_submits_one_atomic_batch_and_shared_velocity() -> None:
    robot = ArxDCanDualArm(
        control_mode="pv",
        left_gripper=False,
        right_gripper=False,
    )
    calls: list[tuple[tuple[object, ...], float]] = []

    class Runtime:
        health = _health()

        @staticmethod
        def set_joint_pv(targets, velocity) -> None:
            calls.append((tuple(targets), float(velocity)))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._ordinary_joint_position_targets = (  # type: ignore[method-assign]
        lambda positions: (("left-motor", positions[0]),)
    )
    robot.right._ordinary_joint_position_targets = (  # type: ignore[method-assign]
        lambda positions: (("right-motor", positions[0]),)
    )
    robot.left._ordinary_joint_velocity = lambda value, **_kwargs: float(value)  # type: ignore[method-assign]
    robot.right._ordinary_joint_velocity = lambda value, **_kwargs: float(value)  # type: ignore[method-assign]

    robot.set_joint_pv(
        left=VALID_POSITIONS,
        right=VALID_POSITIONS,
        velocity=1.5,
    )

    assert calls == [
        (
            (("left-motor", VALID_POSITIONS[0]), ("right-motor", VALID_POSITIONS[0])),
            1.5,
        )
    ]


def test_parallel_send_preserves_native_error() -> None:
    robot = ArxDCanDualArm(
        control_mode="pv",
        left_gripper=False,
        right_gripper=False,
    )

    class Runtime:
        health = _health()

        def submit_pv(self, _commands, **_kwargs) -> None:
            raise CallError("CH1 motor ID 4 failed")

    batch = _PreparedJointPositionBatch(
        "pv",
        (SimpleNamespace(motor=object(), pos=0.0, vlim=1.0),),
    )
    robot._controller_group = object()  # type: ignore[assignment]
    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = lambda _positions, **_kwargs: batch  # type: ignore[method-assign]
    robot.right._prepare_parallel_joint_positions = lambda _positions, **_kwargs: batch  # type: ignore[method-assign]
    with pytest.raises(CallError, match="CH1 motor ID 4"):
        robot._submit_joint_positions(left=VALID_POSITIONS, right=VALID_POSITIONS)


def test_dual_ordinary_pv_synchronizes_native_health() -> None:
    robot = ArxDCanDualArm(
        control_mode="pv",
        left_gripper=False,
        right_gripper=False,
    )
    calls: list[tuple[object, ...]] = []

    class Runtime:
        health = _health()

        def set_joint_pv(self, targets, _velocity) -> None:
            calls.append(tuple(targets))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._ordinary_joint_position_targets = lambda _positions: (("l0", 0.0),)  # type: ignore[method-assign]
    robot.right._ordinary_joint_position_targets = lambda _positions: (("r0", 0.0),)  # type: ignore[method-assign]
    robot.left._ordinary_joint_velocity = lambda value, **_kwargs: float(value)  # type: ignore[method-assign]
    robot.right._ordinary_joint_velocity = lambda value, **_kwargs: float(value)  # type: ignore[method-assign]
    robot.set_joint_pv(left=VALID_POSITIONS, right=VALID_POSITIONS)

    assert calls == [(('l0', 0.0), ('r0', 0.0))]
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
        robot._submit_joint_positions(left=VALID_POSITIONS, right=VALID_POSITIONS)
    assert robot.left.faulted and robot.right.faulted
    assert not robot.left.enabled and not robot.right.enabled


@pytest.mark.parametrize(
    ("initial_state", "expected_calls"),
    (
        (SafetyState.READY, ()),
        (SafetyState.SAFE_HOLD, ("disable",)),
        (SafetyState.FAULT, ("disable", "recover")),
    ),
)
def test_dual_recover_uses_native_fault_recovery_contract(
    initial_state: SafetyState,
    expected_calls: tuple[str, ...],
) -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    calls: list[str] = []

    class Runtime:
        state = initial_state

        @property
        def health(self) -> SafetyHealth:
            return _health(
                self.state,
                disable_confirmed=self.state in {SafetyState.READY, SafetyState.FAULT},
            )

        def disable(self) -> None:
            calls.append("disable")
            if self.state is SafetyState.SAFE_HOLD:
                self.state = SafetyState.READY

        def recover(self) -> None:
            calls.append("recover")
            self.state = SafetyState.READY

    robot._safety_runtime = Runtime()  # type: ignore[assignment]

    robot.recover()

    assert tuple(calls) == expected_calls
    assert robot.safety_health.state is SafetyState.READY


def test_managed_arm_rejects_a_command_that_bypasses_dual_batch() -> None:
    robot = ArxDCanDualArm(
        control_mode="pv",
        left_gripper=False,
        right_gripper=False,
    )
    robot.left._set_dual_runtime_managed(True)

    with pytest.raises(RuntimeError, match="complete left/right set_joint_pv"):
        robot.left.set_joint_pv(range(7))


def test_dual_pv_position_rejects_mit_mode() -> None:
    robot = ArxDCanDualArm(
        control_mode="mit",
        left_gripper=False,
        right_gripper=False,
    )

    with pytest.raises(RuntimeError, match="requires dual-arm PV mode"):
        robot.set_joint_pv(
            left=VALID_POSITIONS,
            right=VALID_POSITIONS,
        )


def test_dual_arm_exposes_only_ordinary_position_interfaces() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)

    assert hasattr(robot, "set_joint_mit")
    assert hasattr(robot, "set_joint_pv")
    assert not hasattr(robot, "move_joint_positions")
    assert not hasattr(robot, "start_joint_trajectory")
    assert not hasattr(robot, "get_trajectory")
    assert not hasattr(robot, "wait_trajectory")
    assert not hasattr(robot, "cancel_trajectory")
    assert not hasattr(robot, "stream_joint_positions")
    assert not hasattr(robot, "stream_mit_joint_commands")
    assert not hasattr(robot, "set_joint_positions")


def test_dual_ordinary_mit_submits_only_positions_and_shared_velocity() -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    calls: list[tuple[tuple[object, ...], float]] = []

    class Runtime:
        health = _health()

        def set_joint_mit(self, targets, velocity) -> None:
            calls.append((tuple(targets), float(velocity)))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._ordinary_joint_position_targets = lambda _positions: (("left", 0.2),)  # type: ignore[method-assign]
    robot.right._ordinary_joint_position_targets = lambda _positions: (("right", -0.3),)  # type: ignore[method-assign]
    robot.left._ordinary_joint_velocity = lambda value, **_kwargs: float(value)  # type: ignore[method-assign]
    robot.right._ordinary_joint_velocity = lambda value, **_kwargs: float(value)  # type: ignore[method-assign]

    robot.set_joint_mit(
        left=VALID_POSITIONS,
        right=VALID_POSITIONS,
        velocity=2.0,
    )

    assert calls == [
        ((('left', 0.2), ('right', -0.3)), 2.0)
    ]


def test_dual_mit_position_rejects_pv_mode() -> None:
    robot = ArxDCanDualArm(control_mode="pv")

    with pytest.raises(RuntimeError, match="requires dual-arm MIT mode"):
        robot.set_joint_mit(
            left=VALID_POSITIONS,
            right=VALID_POSITIONS,
        )


def test_dual_gripper_openings_are_one_native_atomic_submission() -> None:
    robot = ArxDCanDualArm()
    calls: list[tuple[tuple[object, float, float, GripperForceLevel], ...]] = []
    left_motor = object()
    right_motor = object()
    robot.left.robot._motor_map[robot.left.config.gripper.name] = left_motor
    robot.right.robot._motor_map[robot.right.config.gripper.name] = right_motor

    class Runtime:
        def set_grippers(self, targets) -> None:
            calls.append(tuple(targets))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]

    robot.set_gripper_openings(left=-10.0, right=1200.0)

    assert len(calls) == 1
    assert [command.opening for command in calls[0]] == [0.0, 1000.0]
    assert [command.speed for command in calls[0]] == [1000.0, 1000.0]
    assert all(command.force_level == 5 for command in calls[0])
    assert calls[0][0].motor is left_motor
    assert calls[0][1].motor is right_motor


def test_dual_gripper_exposes_simple_profiled_interfaces() -> None:
    robot = ArxDCanDualArm()
    assert hasattr(robot, "set_gripper_openings")
    assert not hasattr(robot, "send_joint_positions")
    assert not hasattr(robot, "hold_joint_positions")
    assert not hasattr(robot, "set_grippers")
    assert not hasattr(robot, "move_grippers")
    assert not hasattr(robot, "open_grippers")
    assert not hasattr(robot, "close_grippers")


def test_custom_end_does_not_submit_disabled_gripper() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=True)
    calls = []
    right_motor = object()
    robot.right.robot._motor_map[robot.right.config.gripper.name] = right_motor

    class Runtime:
        def set_grippers(self, targets) -> None:
            calls.append(tuple(targets))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.set_gripper_openings(left=100.0, right=600.0)

    assert len(calls[0]) == 1
    assert calls[0][0].opening == 600.0
    assert calls[0][0].motor is right_motor


def test_dual_set_zero_releases_and_recreates_runtime_ownership() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    events: list[str] = []

    class Runtime:
        health = _health(SafetyState.READY, disable_confirmed=True)

    runtime = Runtime()
    robot._safety_runtime = runtime  # type: ignore[assignment]
    robot._controller_group = object()  # type: ignore[assignment]
    robot.left._connected = True
    robot.right._connected = True
    robot._release_runtime = lambda: (events.append("release"), setattr(robot, "_safety_runtime", None))  # type: ignore[method-assign]
    robot.left.robot.set_zero = lambda **_kwargs: (events.append("left"), ("l",))[1]  # type: ignore[method-assign]
    robot.right.robot.set_zero = lambda **_kwargs: (events.append("right"), ("r",))[1]  # type: ignore[method-assign]
    robot.left._controller_for_parallel_batch = lambda: object()  # type: ignore[method-assign]
    robot.right._controller_for_parallel_batch = lambda: object()  # type: ignore[method-assign]
    robot._create_safety_runtime = lambda *_args: (events.append("recreate"), runtime)[1]  # type: ignore[method-assign]

    assert robot.set_zero() == (("l",), ("r",))
    assert events == ["release", "left", "right", "recreate"]
    assert robot._safety_runtime is runtime


def test_dual_clear_faults_releases_and_recreates_runtime_ownership() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    events: list[str] = []

    class Runtime:
        health = _health(SafetyState.READY, disable_confirmed=True)

    runtime = Runtime()
    robot._safety_runtime = runtime  # type: ignore[assignment]
    robot._controller_group = object()  # type: ignore[assignment]
    robot.left._connected = True
    robot.right._connected = True
    robot._release_runtime = lambda: (events.append("release"), setattr(robot, "_safety_runtime", None))  # type: ignore[method-assign]
    robot.left.robot.clear_errors = lambda **_kwargs: (events.append("left"), ("l",))[1]  # type: ignore[method-assign]
    robot.right.robot.clear_errors = lambda **_kwargs: (events.append("right"), ("r",))[1]  # type: ignore[method-assign]
    robot.left._controller_for_parallel_batch = lambda: object()  # type: ignore[method-assign]
    robot.right._controller_for_parallel_batch = lambda: object()  # type: ignore[method-assign]
    robot._create_safety_runtime = lambda *_args: (events.append("recreate"), runtime)[1]  # type: ignore[method-assign]

    assert robot.clear_motor_faults() == (("l",), ("r",))
    assert events == ["release", "left", "right", "recreate"]
    assert robot._safety_runtime is runtime


def test_dual_clear_faults_uses_unconfigured_maintenance_connection() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    events: list[str] = []

    for label, arm in (("left", robot.left), ("right", robot.right)):
        arm.robot.connect = lambda side=label: events.append(f"{side}-connect")  # type: ignore[method-assign]
        arm.robot.clear_errors = lambda *, joint_names, side=label: (  # type: ignore[method-assign]
            events.append(f"{side}-clear"),
            (side,),
        )[1]
        arm.robot.disconnect = lambda *, disable, side=label: events.append(  # type: ignore[method-assign]
            f"{side}-close-{disable}"
        )

    assert robot.clear_motor_faults() == (("left",), ("right",))
    assert events == [
        "left-connect",
        "right-connect",
        "left-clear",
        "right-clear",
        "right-close-False",
        "left-close-False",
    ]
    assert not robot.connected


def test_dual_maintenance_clear_attempts_both_sides_and_closes_them() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    events: list[str] = []

    for label, arm in (("left", robot.left), ("right", robot.right)):
        arm.robot.connect = lambda side=label: events.append(f"{side}-connect")  # type: ignore[method-assign]
        arm.robot.disconnect = lambda *, disable, side=label: events.append(  # type: ignore[method-assign]
            f"{side}-close"
        )
    robot.left.robot.clear_errors = lambda **_kwargs: (_ for _ in ()).throw(  # type: ignore[method-assign]
        RuntimeError("left injected failure")
    )
    robot.right.robot.clear_errors = lambda **_kwargs: (  # type: ignore[method-assign]
        events.append("right-clear"),
        ("right",),
    )[1]

    with pytest.raises(RuntimeError, match="left injected failure"):
        robot.clear_motor_faults()

    assert "right-clear" in events
    assert events[-2:] == ["right-close", "left-close"]


def test_managed_single_arm_gripper_cannot_bypass_native_runtime() -> None:
    robot = ArxDCanDualArm()
    robot.left._connected = True
    robot.left._set_dual_runtime_managed(True)

    with pytest.raises(RuntimeError, match="set_gripper_openings"):
        robot.left.set_gripper_opening(500.0)


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
    robot.left.close = lambda: events.append("left")  # type: ignore[method-assign]
    robot.right.close = lambda: events.append("right")  # type: ignore[method-assign]

    robot.close()

    assert events == ["runtime", "group", "left", "right"]


def test_close_failure_still_releases_official_runtime_ownership() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    events: list[str] = []

    class Runtime:
        def close(self) -> None:
            events.append("runtime")
            raise RuntimeError("one motor did not confirm disable")

    class Group:
        def close(self) -> None:
            events.append("group")

    runtime = Runtime()
    group = Group()
    robot._safety_runtime = runtime  # type: ignore[assignment]
    robot._controller_group = group  # type: ignore[assignment]
    robot.left._connected = True
    robot.right._connected = True
    robot.left._set_dual_runtime_managed(True)
    robot.right._set_dual_runtime_managed(True)
    robot.left.close = lambda *, disable=True: events.append("left")  # type: ignore[method-assign]
    robot.right.close = lambda *, disable=True: events.append("right")  # type: ignore[method-assign]

    with pytest.raises(RuntimeError, match="did not confirm disable"):
        robot.close()

    assert events == ["runtime", "group", "left", "right"]
    assert robot._safety_runtime is None
    assert robot._controller_group is None
    assert not robot.left._dual_runtime_managed
    assert not robot.right._dual_runtime_managed
