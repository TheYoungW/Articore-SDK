from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import pytest

from arx_d_can import (
    ArxDCanArm,
    ArxDCanConfig,
    JointMotorConfig,
    SafetyHealth,
    SafetyState,
    TransportHealth,
)
from arx_d_can.sdk.native_safety import (
    CommandLifetime,
    TrajectoryInfo,
    TrajectoryStartOutcome,
    TrajectoryStartReport,
    TrajectoryStatus,
)


JOINT = JointMotorConfig(
    name="joint1",
    motor_id=1,
    feedback_id=0x11,
    model="4340P",
    mit_kp=20.0,
    mit_kd=1.0,
    pv_vel_kp=0.01,
    pv_vel_ki=0.001,
    pv_pos_kp=20.0,
    pv_pos_ki=0.0,
    pv_vlim=2.0,
)
GRIPPER = JointMotorConfig(
    name="gripper",
    motor_id=2,
    feedback_id=0x12,
    model="4310",
    mit_kp=4.0,
    mit_kd=0.5,
    pv_vel_kp=0.0,
    pv_vel_ki=0.0,
    pv_pos_kp=0.0,
    pv_pos_ki=0.0,
    pv_vlim=1.0,
)


def _started(trajectory_id: int) -> TrajectoryStartReport:
    return TrajectoryStartReport(
        TrajectoryStartOutcome.STARTED,
        None,
        trajectory_id,
        False,
        None,
        None,
        None,
        None,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        True,
    )


def _health() -> SafetyHealth:
    transport = TransportHealth(True, True, 0, 0, 0.001, None)
    inactive = TransportHealth(False, False, 0, 0, None, None)
    return SafetyHealth(
        state=SafetyState.RUNNING,
        fault_reason=None,
        last_successful_command_age_s=0.001,
        last_fresh_feedback_age_s=0.001,
        consecutive_send_failures=0,
        consecutive_feedback_failures=0,
        left_transport=transport,
        right_transport=inactive,
        motor_faults=(),
        unconfirmed_disable_motors=(),
        safe_holding=False,
        disable_confirmed=False,
    )


def _ready_arm(
    *,
    joint: JointMotorConfig = JOINT,
) -> tuple[ArxDCanArm, list[tuple[object, ...]], list[object]]:
    config = ArxDCanConfig(arm_joints=(joint,), gripper=GRIPPER)
    arm = ArxDCanArm(config=config, enable_gripper=True)
    joint_calls: list[tuple[object, ...]] = []
    gripper_calls: list[object] = []

    class ArmGroup:
        @staticmethod
        def _make_pos_vel_batch_commands(target, *, vlim=None):
            return tuple((float(value), vlim) for value in target)

        @staticmethod
        def send_pos_vel(*_args, **_kwargs) -> None:
            raise AssertionError("关节命令不应绕过原生运行时")

    motor = object()
    arm.robot = SimpleNamespace(
        arm=ArmGroup(),
        _motor_map={joint.name: object(), "gripper": motor},
    )

    class Runtime:
        health = _health()

        @staticmethod
        def submit_pos_vel(commands, *, lifetime) -> None:
            assert lifetime is CommandLifetime.STREAMING
            joint_calls.append(tuple(commands))

        @staticmethod
        def set_gripper_commands(targets) -> None:
            gripper_calls.extend(targets)

    arm._connected = True
    arm._configured = True
    arm._enabled = True
    arm._single_safety_runtime = Runtime()  # type: ignore[assignment]
    return arm, joint_calls, gripper_calls


def test_single_arm_commands_use_native_runtime() -> None:
    arm, joint_calls, gripper_calls = _ready_arm()

    arm.stream_joint_positions([0.25])
    arm.set_gripper_opening(750)

    assert len(joint_calls) == 1
    assert joint_calls[0][0][0] == pytest.approx(0.25)
    assert tuple(joint_calls[0][0][1]) == pytest.approx((1.0,))
    assert gripper_calls[0][1] == 750.0


@pytest.mark.parametrize("target", (-0.2001, 0.4001))
def test_single_arm_rejects_positions_outside_urdf_limits(target: float) -> None:
    joint = replace(JOINT, lower_limit=-0.2, upper_limit=0.4)
    arm, joint_calls, _ = _ready_arm(joint=joint)

    with pytest.raises(ValueError, match=r"joint1=.*allowed"):
        arm.stream_joint_positions([target])

    assert joint_calls == []


def test_single_public_stream_rejects_mit_mode() -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_control_mode="mit", arm_joints=(JOINT,)),
        enable_gripper=False,
    )

    with pytest.raises(RuntimeError, match="only available in PV mode"):
        arm.stream_joint_positions([0.25])


def test_single_arm_exposes_only_move_and_stream_position_interfaces() -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_joints=(JOINT,)),
        enable_gripper=False,
    )

    assert hasattr(arm, "move_joint_positions")
    assert hasattr(arm, "stream_joint_positions")
    assert not hasattr(arm, "set_joint_positions")


def test_single_arm_enable_does_not_physically_enable_before_runtime() -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_control_mode="mit", arm_joints=(JOINT,)),
        enable_gripper=False,
    )
    events: list[tuple[str, object]] = []

    class ArmGroup:
        @staticmethod
        def enable(*_args, **_kwargs) -> None:
            raise AssertionError("Python 不应在 Runtime 前物理使能")

    class Runtime:
        health = _health()

        @staticmethod
        def enable(mode: str) -> None:
            events.append(("runtime-enable", mode))

    arm.robot = SimpleNamespace(arm=ArmGroup())
    arm._connected = True
    arm._configured = True
    arm._single_safety_runtime = Runtime()  # type: ignore[assignment]

    arm.enable()

    assert events == [("runtime-enable", "mit")]
    assert arm.enabled


@pytest.mark.parametrize(
    ("initial_velocities", "initial_torques", "expected_lifetime"),
    (
        (None, None, CommandLifetime.HOLD_UNTIL_REPLACED),
        ((0.1,), None, CommandLifetime.STREAMING),
        (None, (0.1,), CommandLifetime.STREAMING),
    ),
)
def test_single_mit_initial_command_selects_safe_lifetime(
    initial_velocities,
    initial_torques,
    expected_lifetime: CommandLifetime,
) -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_control_mode="mit", arm_joints=(JOINT,)),
        enable_gripper=False,
    )
    lifetimes: list[CommandLifetime] = []

    class ArmGroup:
        @staticmethod
        def _make_mit_batch_commands(*_args, **_kwargs):
            return (object(),)

    class Runtime:
        health = _health()

        @staticmethod
        def enable(_mode: str) -> None:
            pass

        @staticmethod
        def submit_mit(_commands, *, lifetime) -> None:
            lifetimes.append(lifetime)

    arm.robot = SimpleNamespace(arm=ArmGroup())
    arm._connected = True
    arm._configured = True
    arm._single_safety_runtime = Runtime()  # type: ignore[assignment]

    arm.enable(
        initial_positions=(0.0,),
        initial_velocities=initial_velocities,
        initial_torques=initial_torques,
    )

    assert lifetimes == [expected_lifetime]


def test_single_arm_exposes_no_user_hold_or_open_close_aliases() -> None:
    arm, _, _ = _ready_arm()

    assert not hasattr(arm, "send_joint_positions")
    assert not hasattr(arm, "hold_joint_positions")
    assert not hasattr(arm, "hold_current_position")
    assert not hasattr(arm, "move_gripper")
    assert not hasattr(arm, "open_gripper")
    assert not hasattr(arm, "close_gripper")


def test_single_arm_close_failure_retains_runtime_group_and_transport() -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_joints=(JOINT,)),
        enable_gripper=False,
    )
    events: list[str] = []

    class Runtime:
        def close(self) -> None:
            events.append("runtime")
            raise RuntimeError("joint1 did not confirm disable")

    class Group:
        def close(self) -> None:
            events.append("group")

    runtime = Runtime()
    group = Group()
    arm._single_safety_runtime = runtime  # type: ignore[assignment]
    arm._single_controller_group = group  # type: ignore[assignment]
    arm.robot = SimpleNamespace(
        disconnect=lambda **_kwargs: events.append("transport")
    )
    arm._connected = True
    arm._enabled = True

    with pytest.raises(RuntimeError, match="did not confirm disable"):
        arm.close()

    assert events == ["runtime"]
    assert arm._single_safety_runtime is runtime
    assert arm._single_controller_group is group
    assert arm.connected
    assert arm._enabled
    assert arm._faulted


@pytest.mark.parametrize(
    ("mode", "expected_velocity"),
    (("pv", 1.0), ("mit", 2.0)),
)
def test_trajectory_targets_use_motor_coordinates(
    mode: str,
    expected_velocity: float,
) -> None:
    scaled_joint = replace(
        JOINT,
        direction=-1.0,
        velocity_range=5.0,
        lower_limit=-0.2,
        upper_limit=0.4,
    )
    arm = ArxDCanArm(
        config=ArxDCanConfig(
            arm_control_mode=mode,
            arm_joints=(scaled_joint,),
        ),
        control_mode=mode,
        enable_gripper=False,
    )
    motor = object()
    arm.robot = SimpleNamespace(_motor_map={"joint1": motor})
    arm._connected = True
    arm._enabled = True

    targets = arm._prepare_joint_trajectory_targets([0.25], velocity=None)

    assert targets[0][0] is motor
    assert targets[0][1] == pytest.approx(-0.25)
    assert targets[0][2] == pytest.approx(expected_velocity)


@pytest.mark.parametrize("target", (-0.2001, 0.4001))
def test_trajectory_rejects_positions_outside_urdf_limits(target: float) -> None:
    joint = replace(JOINT, lower_limit=-0.2, upper_limit=0.4)
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_joints=(joint,)),
        enable_gripper=False,
    )
    arm.robot = SimpleNamespace(_motor_map={"joint1": object()})
    arm._connected = True
    arm._enabled = True

    with pytest.raises(ValueError, match=r"joint1=.*allowed"):
        arm._prepare_joint_trajectory_targets([target], velocity=None)


def test_default_velocity_never_exceeds_yaml_vlim() -> None:
    slow_joint = JointMotorConfig(
        name="joint1",
        motor_id=1,
        feedback_id=0x11,
        model="4340P",
        mit_kp=20.0,
        mit_kd=1.0,
        pv_vel_kp=0.01,
        pv_vel_ki=0.001,
        pv_pos_kp=20.0,
        pv_pos_ki=0.0,
        pv_vlim=0.2,
    )
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_joints=(slow_joint,)),
        enable_gripper=False,
    )

    assert tuple(arm._default_joint_velocity_limits()) == pytest.approx((0.1,))


def test_move_joint_positions_blocks_on_one_native_trajectory() -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_joints=(JOINT,)),
        enable_gripper=False,
    )
    calls: list[tuple[object, ...]] = []
    final_state = object()

    class Runtime:
        def start_joint_trajectory(self, targets, *, profile: str) -> int:
            calls.append(("start", tuple(targets), profile))
            return _started(17)

        def wait_trajectory(self, trajectory_id: int) -> TrajectoryInfo:
            calls.append(("wait", trajectory_id))
            return TrajectoryInfo(
                trajectory_id=trajectory_id,
                status=TrajectoryStatus.COMPLETED,
                profile="linear",
                duration_s=0.5,
                elapsed_s=0.5,
                error=None,
            )

    targets = ((object(), 0.25, 0.8),)
    arm._single_safety_runtime = Runtime()  # type: ignore[assignment]
    arm._prepare_joint_trajectory_targets = lambda *_args, **_kwargs: targets  # type: ignore[method-assign]
    arm.read_state = lambda: final_state  # type: ignore[method-assign]

    result = arm.move_joint_positions([0.25], velocity=0.8, profile="linear")

    assert result is final_state
    assert calls == [("start", targets, "linear"), ("wait", 17)]


def test_move_joint_positions_reports_direct_command_preemption() -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_joints=(JOINT,)),
        enable_gripper=False,
    )

    class Runtime:
        @staticmethod
        def start_joint_trajectory(_targets, *, profile: str) -> int:
            del profile
            return _started(18)

        @staticmethod
        def wait_trajectory(trajectory_id: int) -> TrajectoryInfo:
            return TrajectoryInfo(
                trajectory_id=trajectory_id,
                status=TrajectoryStatus.PREEMPTED,
                profile="min_jerk",
                duration_s=1.0,
                elapsed_s=0.2,
                error="preempted by a direct joint command",
            )

    arm._single_safety_runtime = Runtime()  # type: ignore[assignment]
    arm._prepare_joint_trajectory_targets = lambda *_args, **_kwargs: (  # type: ignore[method-assign]
        (object(), 0.25, 0.8),
    )

    with pytest.raises(RuntimeError, match="PREEMPTED.*direct joint command"):
        arm.move_joint_positions([0.25])


def test_parallel_pv_batch_accepts_velocity_limits() -> None:
    arm, _, _ = _ready_arm()

    batch = arm._prepare_parallel_joint_positions(
        [0.25],
        velocity_limits=[0.5],
    )

    assert batch is not None
    assert batch.mode == "pv"
    assert batch.commands[0][0] == pytest.approx(0.25)
    assert tuple(batch.commands[0][1]) == pytest.approx((0.5,))


def test_parallel_mit_batch_uses_defaults_and_accepts_overrides() -> None:
    config = ArxDCanConfig(
        arm_control_mode="mit",
        arm_joints=(JOINT,),
    )
    arm = ArxDCanArm(config=config, enable_gripper=False)
    calls: list[dict[str, tuple[float, ...]]] = []

    class ArmGroup:
        @staticmethod
        def _make_mit_batch_commands(target, *, vel, kp, kd, tau):
            calls.append(
                {
                    "positions": tuple(target),
                    "velocities": tuple(vel),
                    "kp": tuple(kp),
                    "kd": tuple(kd),
                    "torques": tuple(tau),
                }
            )
            return (object(),)

    arm.robot = SimpleNamespace(arm=ArmGroup())
    arm._connected = True
    arm._enabled = True

    arm._prepare_parallel_joint_positions([0.25])
    arm._prepare_parallel_joint_positions(
        [0.5],
        velocities=[0.1],
        torques=[0.2],
        mit_kp=[30.0],
        mit_kd=[1.5],
    )

    assert calls == [
        {
            "positions": (0.25,),
            "velocities": (0.0,),
            "kp": (20.0,),
            "kd": (1.0,),
            "torques": (0.0,),
        },
        {
            "positions": (0.5,),
            "velocities": (0.1,),
            "kp": (30.0,),
            "kd": (1.5,),
            "torques": (0.2,),
        },
    ]


def test_single_arm_exposes_native_transport_health() -> None:
    arm, _, _ = _ready_arm()

    assert arm.communication_health is arm._single_safety_runtime.health.left_transport


def test_read_cached_state_uses_native_motor_cache_without_feedback_request() -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_joints=(JOINT,)),
        enable_gripper=False,
    )
    calls: list[bool] = []

    class Robot:
        @staticmethod
        def get_state(*, request_feedback, require_complete, joint_names):
            calls.append(request_feedback)
            assert require_complete
            assert joint_names == ["joint1"]
            return ([0.25], [0.1], [0.2])

        @staticmethod
        def get_status_codes(*, joint_names):
            assert joint_names == ["joint1"]
            return {"joint1": 1}

    arm.robot = Robot()  # type: ignore[assignment]
    arm._connected = True

    state = arm.read_cached_state()

    assert calls == [False]
    assert state.arm.positions == pytest.approx((0.25,))


def test_builtin_model_rejects_connection_without_native_runtime() -> None:
    arm = ArxDCanArm(model="yunyi_v1_0_left", enable_gripper=False)

    class Robot:
        arm = object()

        @staticmethod
        def connect() -> None:
            pass

        @staticmethod
        def disconnect(*, disable: bool = True) -> None:
            assert not disable

    arm.robot = Robot()  # type: ignore[assignment]

    with pytest.raises(RuntimeError, match="native safety runtime is unavailable"):
        arm.connect()
    assert not arm.connected
