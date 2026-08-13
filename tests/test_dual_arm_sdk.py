from __future__ import annotations

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
from arx_d_can.sdk.native_safety import (
    CommandLifetime,
    TrajectoryInfo,
    TrajectoryStatus,
)


VALID_POSITIONS = (0.0,) * 7
VALID_LEFT_POSITIONS = (0.0, 0.1, 0.2, 0.1, 0.2, 0.1, 0.2)
VALID_RIGHT_POSITIONS = (0.0, -0.1, -0.2, 0.1, -0.2, -0.1, -0.2)


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
    assert robot.left._mode == "mit"
    assert robot.right._mode == "mit"


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

    robot.left._connected = True
    robot.right._connected = True
    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left.configure_mode = lambda mode: calls.append(("left", mode))  # type: ignore[method-assign]
    robot.right.configure_mode = lambda mode: calls.append(("right", mode))  # type: ignore[method-assign]

    robot.configure_mode("mit")

    assert calls == [("left", "mit"), ("right", "mit")]


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


def test_dual_targets_keep_left_and_right_commands_separate() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    left, right = robot._targets(VALID_LEFT_POSITIONS, VALID_RIGHT_POSITIONS)
    assert left == VALID_LEFT_POSITIONS
    assert right == VALID_RIGHT_POSITIONS


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
    robot.left.configure = lambda: calls.append(("configure-left", None))  # type: ignore[method-assign]
    robot.right.configure = lambda: calls.append(("configure-right", None))  # type: ignore[method-assign]
    robot._submit_joint_positions = lambda **kwargs: calls.append(("target", kwargs))  # type: ignore[method-assign]

    left = VALID_LEFT_POSITIONS
    right = VALID_RIGHT_POSITIONS
    robot.enable(left_initial_positions=left, right_initial_positions=right)

    assert calls == [
        ("configure-left", None),
        ("configure-right", None),
        ("runtime", "mit"),
        (
            "target",
            {
                "left": left,
                "right": right,
                "lifetime": CommandLifetime.HOLD_UNTIL_REPLACED,
            },
        ),
    ]


def test_dual_validates_both_sides_before_sending() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    with pytest.raises(ValueError, match="right"):
        robot._targets(VALID_LEFT_POSITIONS, range(6))


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
    robot._create_safety_runtime = lambda *_args: Runtime()  # type: ignore[method-assign]

    robot.connect()
    robot.connect()
    robot.close(disable=False)

    assert events == [
        "left-connected",
        "right-connected",
        ("group-created", (left_controller, right_controller)),
        "runtime-closed",
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

    class Runtime:
        health = _health()

        def submit_pos_vel(self, commands, *, lifetime) -> None:
            calls.append(("send_pos_vel", tuple(commands), lifetime))

        def submit_mit(self, commands, *, lifetime) -> None:
            calls.append(("send_mit", tuple(commands), lifetime))

    robot._controller_group = object()  # type: ignore[assignment]
    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = lambda _positions, **_kwargs: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch(mode, ("l0", "l1"))
    )
    robot.right._prepare_parallel_joint_positions = lambda _positions, **_kwargs: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch(mode, ("r0", "r1"))
    )
    robot._submit_joint_positions(left=VALID_POSITIONS, right=VALID_POSITIONS)

    assert calls == [
        (
            method_name,
            ("l0", "l1", "r0", "r1"),
            CommandLifetime.STREAMING,
        )
    ]


def test_dual_send_rejects_both_sides_before_preparing_an_out_of_limit_target() -> None:
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
        pytest.fail("越界时不应开始准备任意一侧的发送命令")
    )
    right = list(VALID_POSITIONS)
    right[5] = 0.8

    with pytest.raises(ValueError, match=r"r-joint6=.*allowed"):
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

        def submit_mit(self, commands, *, lifetime) -> None:
            assert lifetime is CommandLifetime.STREAMING
            submitted.append(tuple(commands))

    def prepare(side: str, positions, **kwargs):
        prepared[side] = (tuple(positions), kwargs)
        return _PreparedJointPositionBatch("mit", (f"{side}-command",))

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
        },
    )
    assert submitted == [("left-command", "right-command")]


def test_dual_pv_send_forwards_each_side_velocity_limits() -> None:
    robot = ArxDCanDualArm(
        control_mode="pv",
        left_gripper=False,
        right_gripper=False,
    )
    prepared: dict[str, dict[str, object]] = {}

    class Runtime:
        health = _health()

        @staticmethod
        def submit_pos_vel(_commands, *, lifetime) -> None:
            assert lifetime is CommandLifetime.STREAMING
            pass

    def prepare(side: str, _positions, **kwargs):
        prepared[side] = kwargs
        return _PreparedJointPositionBatch("pv", (f"{side}-command",))

    robot._controller_group = object()  # type: ignore[assignment]
    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = (  # type: ignore[method-assign]
        lambda positions, **kwargs: prepare("left", positions, **kwargs)
    )
    robot.right._prepare_parallel_joint_positions = (  # type: ignore[method-assign]
        lambda positions, **kwargs: prepare("right", positions, **kwargs)
    )

    robot.stream_joint_positions(
        left=VALID_POSITIONS,
        right=VALID_POSITIONS,
        left_velocity_limits=range(1, 8),
        right_velocity_limits=range(2, 9),
    )

    assert prepared["left"]["velocity_limits"] == range(1, 8)
    assert prepared["right"]["velocity_limits"] == range(2, 9)


def test_parallel_send_converts_native_error_for_both_arms() -> None:
    robot = ArxDCanDualArm(
        control_mode="pv",
        left_gripper=False,
        right_gripper=False,
    )

    class Runtime:
        health = _health()

        def submit_pos_vel(self, _commands, **_kwargs) -> None:
            raise CallError("CH1 motor ID 4 failed")

    batch = _PreparedJointPositionBatch("pv", (object(),))
    robot._controller_group = object()  # type: ignore[assignment]
    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = lambda _positions, **_kwargs: batch  # type: ignore[method-assign]
    robot.right._prepare_parallel_joint_positions = lambda _positions, **_kwargs: batch  # type: ignore[method-assign]
    with pytest.raises(TransportError, match="CH1 motor ID 4") as caught:
        robot.stream_joint_positions(left=VALID_POSITIONS, right=VALID_POSITIONS)

    assert caught.value.operation == "send_pv"


def test_dual_send_routes_through_native_safety_runtime() -> None:
    robot = ArxDCanDualArm(
        control_mode="pv",
        left_gripper=False,
        right_gripper=False,
    )
    calls: list[tuple[object, ...]] = []

    class Runtime:
        health = _health()

        def submit_pos_vel(self, commands, *, lifetime) -> None:
            assert lifetime is CommandLifetime.STREAMING
            calls.append(tuple(commands))

    class Group:
        def send_pos_vel(self, _commands) -> None:
            raise AssertionError("native safety runtime must own the hot send path")

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot._controller_group = Group()  # type: ignore[assignment]
    robot.left._prepare_parallel_joint_positions = lambda _positions, **_kwargs: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch("pv", ("l0", "l1"))
    )
    robot.right._prepare_parallel_joint_positions = lambda _positions, **_kwargs: (  # type: ignore[method-assign]
        _PreparedJointPositionBatch("pv", ("r0", "r1"))
    )
    robot.stream_joint_positions(left=VALID_POSITIONS, right=VALID_POSITIONS)

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

    with pytest.raises(RuntimeError, match="complete left/right command"):
        robot.left.stream_joint_positions(range(7))


def test_dual_public_stream_rejects_mit_mode() -> None:
    robot = ArxDCanDualArm(
        control_mode="mit",
        left_gripper=False,
        right_gripper=False,
    )

    with pytest.raises(RuntimeError, match="only available in PV mode"):
        robot.stream_joint_positions(
            left=VALID_POSITIONS,
            right=VALID_POSITIONS,
        )


def test_dual_arm_exposes_only_move_and_stream_position_interfaces() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)

    assert hasattr(robot, "move_joint_positions")
    assert hasattr(robot, "stream_joint_positions")
    assert not hasattr(robot, "set_joint_positions")


def test_dual_move_runs_one_blocking_native_trajectory() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    calls: list[tuple[object, ...]] = []
    final_state = object()
    left_targets = ((object(), 0.1, 0.5),)
    right_targets = ((object(), -0.1, 0.5),)

    class Runtime:
        def start_joint_trajectory(
            self,
            targets,
            *,
            profile: str,
            replace: bool,
        ) -> int:
            calls.append(("start", tuple(targets), profile, replace))
            return 23

        def wait_trajectory(self, trajectory_id: int) -> TrajectoryInfo:
            calls.append(("wait", trajectory_id))
            return TrajectoryInfo(
                trajectory_id=trajectory_id,
                status=TrajectoryStatus.COMPLETED,
                profile="min_jerk",
                duration_s=1.0,
                elapsed_s=1.0,
                error=None,
            )

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._prepare_joint_trajectory_targets = lambda *_args, **_kwargs: left_targets  # type: ignore[method-assign]
    robot.right._prepare_joint_trajectory_targets = lambda *_args, **_kwargs: right_targets  # type: ignore[method-assign]
    robot.read_state = lambda: final_state  # type: ignore[method-assign]

    result = robot.move_joint_positions(
        left=VALID_POSITIONS,
        right=VALID_POSITIONS,
        velocity=0.5,
    )

    assert result is final_state
    assert calls == [
        ("start", left_targets + right_targets, "min_jerk", False),
        ("wait", 23),
    ]


def test_dual_nonblocking_trajectory_exposes_replace_query_wait_and_cancel() -> None:
    robot = ArxDCanDualArm(left_gripper=False, right_gripper=False)
    calls: list[tuple[object, ...]] = []
    left_targets = ((object(), 0.1, 0.5),)
    right_targets = ((object(), -0.1, 0.5),)
    running = TrajectoryInfo(
        31,
        TrajectoryStatus.RUNNING,
        "min_jerk",
        1.0,
        0.2,
        None,
    )
    completed = TrajectoryInfo(
        31,
        TrajectoryStatus.COMPLETED,
        "min_jerk",
        1.0,
        1.0,
        None,
    )

    class Runtime:
        def start_joint_trajectory(
            self,
            targets,
            *,
            profile: str,
            replace: bool,
        ) -> int:
            calls.append(("start", tuple(targets), profile, replace))
            return 31

        def get_trajectory(self, trajectory_id: int) -> TrajectoryInfo:
            calls.append(("get", trajectory_id))
            return running

        def wait_trajectory(self, trajectory_id: int) -> TrajectoryInfo:
            calls.append(("wait", trajectory_id))
            return completed

        def cancel_trajectory(self, trajectory_id: int) -> None:
            calls.append(("cancel", trajectory_id))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.left._prepare_joint_trajectory_targets = lambda *_args, **_kwargs: left_targets  # type: ignore[method-assign]
    robot.right._prepare_joint_trajectory_targets = lambda *_args, **_kwargs: right_targets  # type: ignore[method-assign]

    trajectory_id = robot.start_joint_trajectory(
        left=VALID_POSITIONS,
        right=VALID_POSITIONS,
        velocity=0.5,
        replace=True,
    )
    assert robot.get_trajectory(trajectory_id) is running
    assert robot.wait_trajectory(trajectory_id) is completed
    robot.cancel_trajectory(trajectory_id)

    assert calls == [
        ("start", left_targets + right_targets, "min_jerk", True),
        ("get", 31),
        ("wait", 31),
        ("cancel", 31),
    ]


def test_dual_mit_stream_exposes_complete_advanced_command_without_example() -> None:
    robot = ArxDCanDualArm(control_mode="mit")
    calls: list[dict[str, object]] = []
    robot._submit_joint_positions = lambda **kwargs: calls.append(kwargs)  # type: ignore[method-assign]

    robot.stream_mit_joint_commands(
        left_positions=VALID_POSITIONS,
        right_positions=VALID_POSITIONS,
        left_velocities=(0.1,) * 7,
        right_velocities=(-0.1,) * 7,
        left_torques=(0.2,) * 7,
        right_torques=(-0.2,) * 7,
        left_kp=20.0,
        right_kp=21.0,
        left_kd=1.0,
        right_kd=1.1,
    )

    assert calls == [
        {
            "left": VALID_POSITIONS,
            "right": VALID_POSITIONS,
            "left_velocities": (0.1,) * 7,
            "right_velocities": (-0.1,) * 7,
            "left_torques": (0.2,) * 7,
            "right_torques": (-0.2,) * 7,
            "left_mit_kp": 20.0,
            "right_mit_kp": 21.0,
            "left_mit_kd": 1.0,
            "right_mit_kd": 1.1,
            "lifetime": CommandLifetime.STREAMING,
        }
    ]


def test_dual_rejects_mit_stream_in_pv_mode() -> None:
    robot = ArxDCanDualArm(control_mode="pv")

    with pytest.raises(RuntimeError, match="requires dual-arm MIT mode"):
        robot.stream_mit_joint_commands(
            left_positions=VALID_POSITIONS,
            right_positions=VALID_POSITIONS,
        )


def test_dual_smooth_replace_requires_minimum_jerk() -> None:
    robot = ArxDCanDualArm()

    with pytest.raises(ValueError, match="requires min_jerk"):
        robot.start_joint_trajectory(
            left=VALID_POSITIONS,
            right=VALID_POSITIONS,
            profile="linear",
            replace=True,
        )


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

    robot.set_gripper_openings(left=-10.0, right=1200.0)

    assert len(calls) == 1
    assert [value for _, value in calls[0]] == [0.0, 1000.0]
    assert calls[0][0][0] is left_motor
    assert calls[0][1][0] is right_motor


def test_dual_gripper_has_one_explicit_opening_api() -> None:
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
        def set_gripper_openings(self, targets) -> None:
            calls.append(tuple(targets))

    robot._safety_runtime = Runtime()  # type: ignore[assignment]
    robot.set_gripper_openings(left=100.0, right=600.0)

    assert len(calls[0]) == 1
    assert calls[0][0][1] == 600.0
    assert calls[0][0][0] is right_motor


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
    robot.left.close = lambda *, disable=True: events.append("left")  # type: ignore[method-assign]
    robot.right.close = lambda *, disable=True: events.append("right")  # type: ignore[method-assign]

    robot.close(disable=False)

    assert events == ["runtime", "group", "left", "right"]


def test_close_failure_retains_runtime_group_and_both_transports() -> None:
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

    assert events == ["runtime"]
    assert robot._safety_runtime is runtime
    assert robot._controller_group is group
    assert robot.left.connected
    assert robot.right.connected
    assert robot.left._dual_runtime_managed
    assert robot.right._dual_runtime_managed
