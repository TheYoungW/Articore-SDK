from __future__ import annotations

from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from arx_d_can import (
    ArxDCanArm,
    ArxDCanConfig,
    JointMotorConfig,
    RuntimeTransportHealth,
    SafetyHealth,
    SafetyState,
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


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("mit_kp", 500.01, r"MIT\.kp must be in \[0, 500\]"),
        ("mit_kd", 5.01, r"MIT\.kd must be in \[0, 5\]"),
    ),
)
def test_arm_rejects_mit_gains_above_protocol_range(
    field: str,
    value: float,
    message: str,
) -> None:
    joint = replace(JOINT, **{field: value})

    with pytest.raises(ValueError, match=message):
        ArxDCanArm(
            config=ArxDCanConfig(arm_joints=(joint,)),
            enable_gripper=False,
        )


def _health() -> SafetyHealth:
    transport = RuntimeTransportHealth(
        True, True, 0, 0, 1_000_000, 0, 0, 0, 0, None, None, None
    )
    inactive = RuntimeTransportHealth(
        False, False, 0, 0, None, 0, 0, 0, 0, None, None, None
    )
    return SafetyHealth(
        state=SafetyState.RUNNING,
        safe_holding=False,
        disable_confirmed=False,
        last_successful_command_age_ns=1_000_000,
        last_fresh_feedback_age_ns=1_000_000,
        consecutive_send_failures=0,
        consecutive_feedback_failures=0,
        left_transport=transport,
        right_transport=inactive,
        grippers=(),
        motor_faults=(),
        unconfirmed_disable=(),
        fault_reason=None,
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
        def set_joint_pv(targets, velocity) -> None:
            joint_calls.append((tuple(targets), float(velocity)))

        @staticmethod
        def set_grippers(targets) -> None:
            gripper_calls.extend(targets)

    arm._connected = True
    arm._configured = True
    arm._enabled = True
    arm._single_safety_runtime = Runtime()  # type: ignore[assignment]
    return arm, joint_calls, gripper_calls


def test_single_arm_commands_use_native_runtime() -> None:
    arm, joint_calls, gripper_calls = _ready_arm()

    arm.set_joint_pv([0.25])
    arm.set_gripper_opening(750)

    assert len(joint_calls) == 1
    assert joint_calls[0][0][0].position == pytest.approx(0.25)
    assert joint_calls[0][1] == pytest.approx(1.0)
    assert gripper_calls[0].opening == pytest.approx(750.0)
    assert gripper_calls[0].speed == pytest.approx(1000.0)
    assert gripper_calls[0].force_level == 5


def test_single_arm_mit_uses_direction_and_one_shared_velocity() -> None:
    joint = replace(JOINT, direction=-1.0)
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_control_mode="mit", arm_joints=(joint,)),
        enable_gripper=False,
    )
    motor = object()
    calls = []

    class Runtime:
        health = _health()

        @staticmethod
        def set_joint_mit(targets, velocity) -> None:
            calls.append((tuple(targets), float(velocity)))

    arm.robot = SimpleNamespace(_motor_map={joint.name: motor})
    arm._connected = True
    arm._configured = True
    arm._enabled = True
    arm._single_safety_runtime = Runtime()  # type: ignore[assignment]

    arm.set_joint_mit([0.25], velocity=1.5)

    assert len(calls) == 1
    assert calls[0][0][0].motor is motor
    assert calls[0][0][0].position == pytest.approx(-0.25)
    assert calls[0][1] == pytest.approx(1.5)


def test_single_arm_mit_velocity_is_capped_at_200_degrees_per_second() -> None:
    joint = replace(JOINT, pv_vlim=5.0)
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_control_mode="mit", arm_joints=(joint,)),
        enable_gripper=False,
    )

    maximum = math.radians(200.0)
    assert arm._ordinary_joint_velocity(maximum, mode="mit") == pytest.approx(
        maximum
    )
    with pytest.raises(ValueError, match="200 deg/s"):
        arm._ordinary_joint_velocity(math.radians(200.01), mode="mit")

    assert arm._ordinary_joint_velocity(
        math.radians(250.0), mode="pv"
    ) == pytest.approx(math.radians(250.0))


@pytest.mark.parametrize("velocity", (0.0, -1.0, float("nan"), 2.1))
def test_single_arm_rejects_invalid_shared_velocity(velocity: float) -> None:
    arm, joint_calls, _ = _ready_arm()

    with pytest.raises(ValueError, match="velocity must be"):
        arm.set_joint_pv([0.25], velocity=velocity)

    assert joint_calls == []


@pytest.mark.parametrize("target", (-0.2001, 0.4001))
def test_single_arm_rejects_positions_outside_urdf_limits(target: float) -> None:
    joint = replace(JOINT, lower_limit=-0.2, upper_limit=0.4)
    arm, joint_calls, _ = _ready_arm(joint=joint)

    with pytest.raises(ValueError, match=r"joint1=.*allowed"):
        arm.set_joint_pv([target])

    assert joint_calls == []


def test_single_pv_position_rejects_mit_mode() -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_control_mode="mit", arm_joints=(JOINT,)),
        enable_gripper=False,
    )

    with pytest.raises(RuntimeError, match="requires PV mode"):
        arm.set_joint_pv([0.25])


def test_single_arm_exposes_only_ordinary_position_interfaces() -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_joints=(JOINT,)),
        enable_gripper=False,
    )

    assert hasattr(arm, "set_joint_mit")
    assert hasattr(arm, "set_joint_pv")
    assert not hasattr(arm, "move_joint_positions")
    assert not hasattr(arm, "stream_joint_positions")
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

    assert events == [("runtime-enable", 2)]
    assert arm.enabled


def test_single_arm_exposes_no_user_hold_or_open_close_aliases() -> None:
    arm, _, _ = _ready_arm()

    assert not hasattr(arm, "send_joint_positions")
    assert not hasattr(arm, "hold_joint_positions")
    assert not hasattr(arm, "hold_current_position")
    assert not hasattr(arm, "move_gripper")
    assert not hasattr(arm, "open_gripper")
    assert not hasattr(arm, "close_gripper")


def test_single_arm_close_failure_still_releases_official_runtime_ownership() -> None:
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

    assert events == ["runtime", "group", "transport"]
    assert arm._single_safety_runtime is None
    assert arm._single_controller_group is None
    assert not arm.connected


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


@pytest.mark.parametrize(
    ("kwargs", "message"),
    (
        ({"mit_kp": [500.01]}, r"MIT Kp values must be finite and in \[0, 500\]"),
        ({"mit_kd": [5.01]}, r"MIT Kd values must be finite and in \[0, 5\]"),
    ),
)
def test_parallel_mit_batch_rejects_gains_above_protocol_range(
    kwargs: dict[str, list[float]],
    message: str,
) -> None:
    arm = ArxDCanArm(
        config=ArxDCanConfig(arm_control_mode="mit", arm_joints=(JOINT,)),
        enable_gripper=False,
    )
    arm._connected = True
    arm._enabled = True

    with pytest.raises(ValueError, match=message):
        arm._prepare_parallel_joint_positions([0.25], **kwargs)


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
    arm._configure = lambda: setattr(arm, "_configured", True)  # type: ignore[method-assign]

    with pytest.raises(RuntimeError, match="ArticoreRuntime is unavailable"):
        arm.connect()
    assert not arm.connected
