import threading
import time

import numpy as np
import pytest

from arx_d_can import (
    ArxDCanArm,
    ArxDCanConfig,
    CommunicationError,
    FeedbackTimeoutError,
    JointMotorConfig,
    MotorFaultError,
    TransportError,
)


JOINT = JointMotorConfig(
    name="joint1",
    motor_id=1,
    feedback_id=0x11,
    model="4340P",
    mit_kp=10.0,
    mit_kd=1.0,
    pv_vel_kp=0.01,
    pv_vel_ki=0.001,
    pv_pos_kp=50.0,
    pv_pos_ki=0.5,
    pv_vlim=3.0,
)

GRIPPER = JointMotorConfig(
    name="gripper",
    motor_id=7,
    feedback_id=0x17,
    model="4310",
    mit_kp=4.0,
    mit_kd=0.5,
    pv_vel_kp=0.001,
    pv_vel_ki=0.001,
    pv_pos_kp=50.0,
    pv_pos_ki=0.5,
    pv_vlim=3.0,
)


class FakeGroup:
    def __init__(self) -> None:
        self.send_error = None
        self.enabled = False
        self.mode_calls: list[str] = []
        self.sent_pos_vel: list[np.ndarray] = []
        self.sent_pos_vel_limits: list[np.ndarray | None] = []
        self.sent_mit: list[np.ndarray] = []
        self.sent_mit_velocities: list[np.ndarray | None] = []
        self.sent_mit_torques: list[np.ndarray | None] = []
        self.sent_mit_kp: list[np.ndarray | None] = []
        self.sent_mit_kd: list[np.ndarray | None] = []
        self.enable_kwargs = None

    def mode_pos_vel(self) -> bool:
        self.mode_calls.append("pv")
        return True

    def mode_mit(self) -> bool:
        self.mode_calls.append("mit")
        return True

    def enable(self, **kwargs) -> None:
        self.enable_kwargs = kwargs
        self.enabled = True

    def send_pos_vel(self, target, *, vlim=None, strict=True) -> None:
        if self.send_error is not None:
            raise self.send_error
        self.sent_pos_vel.append(np.asarray(target, dtype=np.float64).copy())
        self.sent_pos_vel_limits.append(
            None if vlim is None else np.asarray(vlim, dtype=np.float64).copy()
        )

    def send_mit(self, target, *, strict=True, **kwargs) -> None:
        if self.send_error is not None:
            raise self.send_error
        self.sent_mit.append(np.asarray(target, dtype=np.float64).copy())
        velocity = kwargs.get("vel")
        self.sent_mit_velocities.append(
            None if velocity is None else np.asarray(velocity, dtype=np.float64).copy()
        )
        torque = kwargs.get("tau")
        self.sent_mit_torques.append(
            None if torque is None else np.asarray(torque, dtype=np.float64).copy()
        )
        kp = kwargs.get("kp")
        self.sent_mit_kp.append(
            None if kp is None else np.asarray(kp, dtype=np.float64).copy()
        )
        kd = kwargs.get("kd")
        self.sent_mit_kd.append(
            None if kd is None else np.asarray(kd, dtype=np.float64).copy()
        )


class FakeRobot:
    def __init__(self, position: float = 0.42) -> None:
        self.arm = FakeGroup()
        self.gripper = FakeGroup()
        self.estop_calls = 0
        self.disconnect_calls = 0
        self.position = position
        self.last_state_joint_names: list[str] | None = None
        self.clear_error_joint_names: list[str] | None = None
        self.estop_error: Exception | None = None
        self.state_error: Exception | None = None
        self.status_codes = {"joint1": 1, "gripper": 1}
        self.configured_can_timeouts: dict[str, int] = {}
        self._motor_map = {
            name: type(
                "FakeMotor",
                (),
                {
                    "set_can_timeout_ms": lambda _motor, timeout, motor_name=name: (
                        self.configured_can_timeouts.__setitem__(motor_name, timeout)
                    )
                },
            )()
            for name in ("joint1", "gripper")
        }

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        self.disconnect_calls += 1
        self.estop()

    def estop(self) -> None:
        self.estop_calls += 1
        if self.estop_error is not None:
            raise self.estop_error
        self.arm.enabled = False

    def clear_errors(self, *, joint_names=None):
        self.clear_error_joint_names = None if joint_names is None else list(joint_names)
        self.arm.enabled = False
        self.gripper.enabled = False
        return tuple(self.clear_error_joint_names or ())

    def get_state(
        self,
        *,
        request_feedback=True,
        require_complete=False,
        joint_names=None,
    ):
        if self.state_error is not None:
            raise self.state_error
        self.last_state_joint_names = None if joint_names is None else list(joint_names)
        count = 1 if joint_names is None else len(joint_names)
        values = np.full(count, self.position)
        return values, values.copy(), values.copy()

    def get_status_codes(self, *, joint_names=None):
        names = ["joint1"] if joint_names is None else list(joint_names)
        return {name: self.status_codes[name] for name in names}


def make_arm(
    *,
    timeout=0.03,
    grace=0.03,
    watchdog_action="safe_hold",
) -> tuple[ArxDCanArm, FakeRobot]:
    config = ArxDCanConfig(
        arm_joints=(JOINT,),
        watchdog_enabled=True,
        command_timeout_s=timeout,
        enable_grace_s=grace,
        watchdog_poll_s=0.005,
        watchdog_action=watchdog_action,
        safe_hold_hz=100.0,
        feedback_fault_threshold=3,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    return arm, robot


def test_mit_enable_passes_exact_initial_hold_to_joint_group() -> None:
    config = ArxDCanConfig(
        arm_joints=(JOINT,),
        arm_control_mode="mit",
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure("mit")

    assert robot.configured_can_timeouts == {"joint1": 500}
    try:
        arm.enable(
            initial_positions=[-0.347],
            initial_velocities=[0.0],
            initial_torques=[0.0],
            mit_kp=[120.0],
            mit_kd=[8.0],
        )

        assert robot.arm.enable_kwargs is not None
        np.testing.assert_allclose(
            robot.arm.enable_kwargs["mit_position"], [-0.347]
        )
        np.testing.assert_allclose(robot.arm.enable_kwargs["mit_velocity"], [0.0])
        np.testing.assert_allclose(robot.arm.enable_kwargs["mit_tau"], [0.0])
        np.testing.assert_allclose(robot.arm.enable_kwargs["mit_kp"], [120.0])
        np.testing.assert_allclose(robot.arm.enable_kwargs["mit_kd"], [8.0])
    finally:
        arm.close()


def wait_for_fault(arm: ArxDCanArm, timeout: float = 0.5) -> None:
    deadline = time.monotonic() + timeout
    while not arm.faulted and time.monotonic() < deadline:
        time.sleep(0.005)
    assert arm.faulted


def test_watchdog_holds_last_successful_command_without_disabling() -> None:
    arm, robot = make_arm()
    try:
        arm.send_joint_positions([0.0])
        wait_for_fault(arm)
        deadline = time.monotonic() + 0.2
        while len(robot.arm.sent_pos_vel) < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert arm.enabled
        assert arm.safe_holding
        assert "watchdog" in (arm.fault_reason or "")
        assert robot.estop_calls == 0
        np.testing.assert_allclose(robot.arm.sent_pos_vel[-1], [0.0])
    finally:
        arm.close()


def test_mit_watchdog_replays_complete_command_without_yaml_fallback() -> None:
    config = ArxDCanConfig(
        arm_control_mode="mit",
        arm_joints=(JOINT,),
        watchdog_enabled=True,
        command_timeout_s=0.03,
        enable_grace_s=0.03,
        watchdog_poll_s=0.005,
        watchdog_action="safe_hold",
        safe_hold_hz=100.0,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        arm.send_joint_positions(
            [0.25],
            velocities=[0.4],
            torques=[0.6],
            mit_kp=[3.0],
            mit_kd=[0.2],
        )
        wait_for_fault(arm)
        deadline = time.monotonic() + 0.2
        while len(robot.arm.sent_mit) < 2 and time.monotonic() < deadline:
            time.sleep(0.005)

        assert len(robot.arm.sent_mit) >= 2
        np.testing.assert_allclose(robot.arm.sent_mit[-1], [0.25])
        np.testing.assert_allclose(robot.arm.sent_mit_velocities[-1], [0.4])
        np.testing.assert_allclose(robot.arm.sent_mit_kp[-1], [3.0])
        np.testing.assert_allclose(robot.arm.sent_mit_kd[-1], [0.2])
        np.testing.assert_allclose(robot.arm.sent_mit_torques[-1], [0.6])
        assert arm._last_mit_command is not None
        assert arm._last_mit_command.kp == (3.0,)
        assert arm._last_mit_command.kd == (0.2,)
    finally:
        arm.close()


def test_watchdog_can_be_configured_to_disable() -> None:
    arm, robot = make_arm(watchdog_action="disable")
    try:
        arm.send_joint_positions([0.0])
        wait_for_fault(arm)
        assert not arm.enabled
        assert not arm.safe_holding
        assert robot.estop_calls >= 1
    finally:
        arm.close()


def test_send_failure_holds_last_command_until_feedback_recovers() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    try:
        arm.send_joint_positions([0.2])
        robot.arm.send_error = RuntimeError("simulated bus failure")
        arm.send_joint_positions([0.4])
        assert arm.faulted
        assert arm.enabled
        assert arm.safe_holding
        assert robot.estop_calls == 0
        arm.send_joint_positions([0.6])

        robot.arm.send_error = None
        arm.read_state()
        assert not arm.faulted
        assert not arm.safe_holding
        assert arm.enabled
        arm.send_joint_positions([0.5])
        np.testing.assert_allclose(robot.arm.sent_pos_vel[-1], [0.5])
    finally:
        arm.close()


def test_send_transport_error_is_exposed_through_communication_health() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    try:
        arm.send_joint_positions([0.2])
        error = TransportError(
            "serial write failed",
            operation="send_pos_vel",
            transport="dm-serial",
            channel="/dev/ttyACM0",
            motor_names=("joint1",),
            retryable=True,
        )
        robot.arm.send_error = error

        arm.send_joint_positions([0.4])

        assert arm.safe_holding
        health = arm.communication_health
        assert health.last_error is error
        assert not health.using_fallback_state
        assert not health.healthy
    finally:
        arm.close()


def test_consecutive_feedback_failures_hold_without_disabling_and_auto_recover() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    try:
        arm.send_joint_positions([0.3])
        last_state = arm.read_state()
        robot.state_error = FeedbackTimeoutError(
            "missing motor IDs: 1",
            operation="request_feedback",
            motor_names=("joint1",),
            retryable=True,
        )
        for _ in range(3):
            with pytest.raises(CommunicationError):
                arm.read_state()

        assert arm.read_cached_state() is last_state

        assert arm.faulted
        assert arm.safe_holding
        assert arm.enabled
        assert robot.estop_calls == 0

        robot.state_error = None
        arm.read_state()
        assert not arm.faulted
        assert not arm.safe_holding
        assert arm.enabled
    finally:
        arm.close()


def test_failed_fresh_read_is_visible_in_communication_health() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    try:
        arm.send_joint_positions([0.3])
        last_state = arm.read_state()
        assert arm.communication_health.healthy

        error = FeedbackTimeoutError(
            "missing motor IDs: 1",
            operation="request_feedback",
            motor_names=("joint1",),
            retryable=True,
        )
        robot.state_error = error

        with pytest.raises(CommunicationError):
            arm.read_state()
        assert arm.read_cached_state() is last_state
        health = arm.communication_health
        assert health.consecutive_feedback_failures == 1
        assert health.has_fresh_feedback
        assert not health.using_fallback_state
        assert health.last_error is error
        assert health.last_fresh_feedback_age_s is not None
        assert not health.healthy

        robot.state_error = None
        arm.read_state()
        recovered = arm.communication_health
        assert recovered.healthy
        assert recovered.last_error is None
        assert recovered.consecutive_feedback_failures == 0
    finally:
        arm.close()


def test_read_state_always_raises_instead_of_returning_fallback() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    try:
        arm.send_joint_positions([0.3])
        arm.read_state()
        robot.state_error = FeedbackTimeoutError(
            "feedback timed out",
            operation="request_feedback",
            motor_names=("joint1",),
            retryable=True,
        )

        with pytest.raises(CommunicationError, match="feedback timed out"):
            arm.read_state()

        assert arm.communication_health.consecutive_feedback_failures == 1
        assert not arm.communication_health.using_fallback_state
        assert not arm.safe_holding
    finally:
        arm.close()


def test_public_state_api_separates_fresh_and_cached_reads() -> None:
    arm, _ = make_arm(timeout=1.0, grace=1.0)
    try:
        with pytest.raises(RuntimeError, match="no cached state"):
            arm.read_cached_state()
        with pytest.raises(TypeError):
            arm.read_state(strict_feedback=True)
        with pytest.raises(TypeError):
            arm.read_state(request_feedback=False)

        fresh = arm.read_state()
        assert arm.read_cached_state() is fresh
    finally:
        arm.close()


def test_high_level_motion_helpers_own_the_refresh_loop(monkeypatch) -> None:
    config = ArxDCanConfig(
        arm_joints=(JOINT,),
        gripper=GRIPPER,
        watchdog_enabled=False,
        control_hz=100.0,
    )
    arm = ArxDCanArm(config=config, enable_gripper=True)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()

    clock = {"now": 0.0}
    monkeypatch.setattr(
        "arx_d_can.sdk.arm.time.monotonic",
        lambda: clock["now"],
    )
    monkeypatch.setattr(
        "arx_d_can.sdk.arm.time.sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )
    try:
        arm.hold_joint_positions([0.2], seconds=0.025, hz=100.0)
        assert len(robot.arm.sent_pos_vel) == 3

        clock["now"] = 0.0
        robot.arm.sent_pos_vel.clear()
        arm.move_joint_positions([0.0], seconds=0.02, hz=100.0)
        assert len(robot.arm.sent_pos_vel) == 3

        clock["now"] = 0.0
        arm.move_gripper(1000, seconds=0.025, hz=100.0)
        assert len(robot.gripper.sent_mit) == 3
    finally:
        arm.close()


def test_explicit_motor_fault_is_not_classified_as_communication_error() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    try:
        robot.state_error = MotorFaultError(
            "joint1 over-temperature",
            status_codes={"joint1": 8},
        )

        with pytest.raises(MotorFaultError):
            arm.read_state()

        assert arm.faulted
        assert not arm.safe_holding
        assert not arm.enabled
        assert robot.estop_calls == 1
        assert arm.communication_health.last_error is None
    finally:
        arm.close()


def test_background_feedback_refresh_does_not_hold_command_io_lock() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    feedback_entered = threading.Event()
    feedback_release = threading.Event()
    command_finished = threading.Event()
    original_get_state = robot.get_state

    def blocking_get_state(**kwargs):
        if kwargs.get("request_feedback", True):
            feedback_entered.set()
            assert feedback_release.wait(0.5)
        return original_get_state(**kwargs)

    robot.get_state = blocking_get_state
    feedback_thread = threading.Thread(target=arm.refresh_feedback_background)
    command_thread = threading.Thread(
        target=lambda: (
            arm.send_joint_positions([0.25]),
            command_finished.set(),
        )
    )
    try:
        feedback_thread.start()
        assert feedback_entered.wait(0.2)
        command_thread.start()
        assert command_finished.wait(0.2)
    finally:
        feedback_release.set()
        feedback_thread.join()
        command_thread.join()
        arm.close()


def test_cached_reads_do_not_clear_background_feedback_failure_count() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    arm.send_joint_positions([0.3])
    arm.read_state()
    original_get_state = robot.get_state

    def fail_only_fresh_feedback(**kwargs):
        if kwargs.get("request_feedback", True):
            raise FeedbackTimeoutError(
                "missing motor IDs: 15",
                operation="request_feedback",
                motor_names=("joint1",),
                retryable=True,
            )
        return original_get_state(**kwargs)

    robot.get_state = fail_only_fresh_feedback
    try:
        for _ in range(2):
            arm.refresh_feedback_background()
            arm.read_cached_state()
            assert not arm.faulted

        arm.refresh_feedback_background()

        assert arm.faulted
        assert arm.safe_holding
        assert "3 consecutive" in (arm.fault_reason or "")
    finally:
        robot.get_state = original_get_state
        arm.close()


def test_feedback_recovery_stays_in_hold_until_every_motor_is_enabled() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    try:
        arm.send_joint_positions([0.3])
        arm.read_state()
        robot.state_error = FeedbackTimeoutError(
            "missing motor IDs: 1",
            operation="request_feedback",
            motor_names=("joint1",),
            retryable=True,
        )
        for _ in range(3):
            with pytest.raises(CommunicationError):
                arm.read_state()
        assert arm.safe_holding

        robot.state_error = None
        robot.status_codes["joint1"] = 0
        arm.read_state()
        assert arm.faulted
        assert arm.safe_holding
        assert arm.enabled
        assert "unexpectedly disabled" in (arm.fault_reason or "")
        assert robot.estop_calls == 0

        robot.status_codes["joint1"] = 1
        arm.read_state()
        assert not arm.faulted
        assert not arm.safe_holding
        assert arm.enabled
    finally:
        arm.close()


def test_disable_failure_remains_enabled_and_records_reason() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    robot.estop_error = RuntimeError("disabled feedback missing")
    try:
        with pytest.raises(RuntimeError, match="disabled feedback missing"):
            arm.disable()
        assert arm.enabled
        assert arm.faulted
        assert arm.fault_reason == "disable failed: disabled feedback missing"
    finally:
        robot.estop_error = None
        arm.close()


def test_close_failure_does_not_claim_confirmed_disable() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    robot.estop_error = RuntimeError("disabled feedback missing")

    with pytest.raises(RuntimeError, match="disabled feedback missing"):
        arm.close()

    assert not arm.connected
    assert arm.enabled
    assert arm.faulted
    assert "close failed" in (arm.fault_reason or "")


@pytest.mark.parametrize("mode", ["pv", "mit"])
def test_joint_positions_use_configured_control_mode(mode: str) -> None:
    config = ArxDCanConfig(
        arm_control_mode=mode,
        arm_joints=(JOINT,),
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        velocities = [0.3] if mode == "mit" else None
        velocity_limits = [0.2] if mode == "pv" else None
        torques = [0.1] if mode == "mit" else None
        arm.send_joint_positions(
            [0.25],
            velocities=velocities,
            velocity_limits=velocity_limits,
            torques=torques,
        )

        assert robot.arm.mode_calls == [mode]
        if mode == "pv":
            np.testing.assert_allclose(robot.arm.sent_pos_vel[-1], [0.25])
            np.testing.assert_allclose(robot.arm.sent_pos_vel_limits[-1], [0.2])
            assert robot.arm.sent_mit == []
        else:
            np.testing.assert_allclose(robot.arm.sent_mit[-1], [0.25])
            np.testing.assert_allclose(robot.arm.sent_mit_velocities[-1], [0.3])
            np.testing.assert_allclose(robot.arm.sent_mit_torques[-1], [0.1])
            assert robot.arm.sent_pos_vel == []
    finally:
        arm.close()


def test_control_mode_can_only_change_while_disabled() -> None:
    config = ArxDCanConfig(
        arm_control_mode="pv",
        arm_joints=(JOINT,),
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.enable()
    try:
        with pytest.raises(RuntimeError, match=r"disable\(\) first"):
            arm.configure("mit")
        assert robot.arm.mode_calls == ["pv"]

        arm.disable()
        arm.configure("mit")
        arm.enable()
        assert robot.arm.mode_calls == ["pv", "mit"]
    finally:
        arm.close()


def test_pv_mode_rejects_mit_torques() -> None:
    config = ArxDCanConfig(
        arm_control_mode="pv",
        arm_joints=(JOINT,),
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        with pytest.raises(ValueError, match="only supported in MIT mode"):
            arm.send_joint_positions([0.25], torques=[0.1])
    finally:
        arm.close()


def test_mit_packet_can_override_kp_and_kd_without_changing_defaults() -> None:
    config = ArxDCanConfig(
        arm_control_mode="mit",
        arm_joints=(JOINT,),
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        arm.send_joint_positions(
            [0.25],
            torques=[0.1],
            mit_kp=[3.0],
            mit_kd=[0.2],
        )
        arm.send_joint_positions([0.3], torques=[0.2])

        np.testing.assert_allclose(robot.arm.sent_mit_kp[0], [3.0])
        np.testing.assert_allclose(robot.arm.sent_mit_kd[0], [0.2])
        np.testing.assert_allclose(robot.arm.sent_mit_kp[1], [JOINT.mit_kp])
        np.testing.assert_allclose(robot.arm.sent_mit_kd[1], [JOINT.mit_kd])
    finally:
        arm.close()


def test_scalar_zero_mit_gains_apply_to_every_joint() -> None:
    config = ArxDCanConfig(
        arm_control_mode="mit",
        arm_joints=(JOINT,),
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        arm.send_joint_positions(
            [0.0],
            torques=[0.4],
            mit_kp=0,
            mit_kd=0,
        )

        assert robot.arm.mode_calls == ["mit"]
        np.testing.assert_allclose(robot.arm.sent_mit[-1], [0.0])
        np.testing.assert_allclose(robot.arm.sent_mit_velocities[-1], [0.0])
        np.testing.assert_allclose(robot.arm.sent_mit_kp[-1], [0.0])
        np.testing.assert_allclose(robot.arm.sent_mit_kd[-1], [0.0])
        np.testing.assert_allclose(robot.arm.sent_mit_torques[-1], [0.4])
    finally:
        arm.close()


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"mit_kp": []}, "expected 1 MIT Kp"),
        ({"mit_kd": [float("nan")]}, "MIT Kd values must be finite"),
        ({"mit_kp": [-0.1]}, "MIT Kp values must be finite"),
    ],
)
def test_mit_gain_overrides_are_validated(kwargs: dict, message: str) -> None:
    config = ArxDCanConfig(
        arm_control_mode="mit",
        arm_joints=(JOINT,),
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        with pytest.raises(ValueError, match=message):
            arm.send_joint_positions([0.25], **kwargs)
    finally:
        arm.close()


@pytest.mark.parametrize(
    ("mode", "kwargs", "message"),
    [
        ("pv", {"velocities": [0.1]}, "only supported in MIT mode"),
        ("mit", {"velocity_limits": [0.1]}, "only supported in PV mode"),
        ("pv", {"mit_kp": [0.0]}, "only supported in MIT mode"),
        ("pv", {"mit_kd": [0.0]}, "only supported in MIT mode"),
    ],
)
def test_control_modes_reject_other_mode_velocity_parameter(
    mode: str,
    kwargs: dict,
    message: str,
) -> None:
    config = ArxDCanConfig(
        arm_control_mode=mode,
        arm_joints=(JOINT,),
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        with pytest.raises(ValueError, match=message):
            arm.send_joint_positions([0.25], **kwargs)
    finally:
        arm.close()


@pytest.mark.parametrize(
    ("requested", "expected"),
    [
        (-0.1, 0.0),
        (1.32, 1.32),
        (2.64, 2.64),
        (3.0, 2.64),
    ],
)
def test_gripper_motor_value_is_clamped_to_mechanical_range(
    requested: float,
    expected: float,
) -> None:
    config = ArxDCanConfig(
        arm_joints=(JOINT,),
        gripper=GRIPPER,
        gripper_closed_value=0.0,
        gripper_open_value=2.64,
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config, enable_gripper=True)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        arm.set_gripper_motor_value(requested)

        np.testing.assert_allclose(robot.gripper.sent_mit[-1], [expected])
    finally:
        arm.close()


@pytest.mark.parametrize(
    ("requested", "expected"),
    [
        (-1.0, 0.0),
        (0.0, 0.0),
        (500.0, 1.32),
        (1000.0, 2.64),
        (1001.0, 2.64),
    ],
)
def test_gripper_uses_simple_zero_to_one_thousand_scale(
    requested: float,
    expected: float,
) -> None:
    config = ArxDCanConfig(
        arm_joints=(JOINT,),
        gripper=GRIPPER,
        gripper_closed_value=0.0,
        gripper_open_value=2.64,
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config, enable_gripper=True)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        arm.set_gripper(requested)

        np.testing.assert_allclose(robot.gripper.sent_mit[-1], [expected])
    finally:
        arm.close()


def test_open_and_close_gripper_use_mapped_endpoints() -> None:
    config = ArxDCanConfig(
        arm_joints=(JOINT,),
        gripper=GRIPPER,
        gripper_closed_value=0.0,
        gripper_open_value=2.64,
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config, enable_gripper=True)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    arm.configure()
    arm.enable()
    try:
        arm.open_gripper()
        arm.close_gripper()

        np.testing.assert_allclose(robot.gripper.sent_mit[-2], [2.64])
        np.testing.assert_allclose(robot.gripper.sent_mit[-1], [0.0])
        assert robot.gripper.mode_calls == ["mit"]
    finally:
        arm.close()


def test_enable_reconfigures_automatically_after_clear_fault() -> None:
    arm, robot = make_arm()
    try:
        arm.send_joint_positions([0.0])
        wait_for_fault(arm)
        estop_calls = robot.estop_calls
        arm.clear_fault()
        assert not arm.faulted
        assert not arm.safe_holding
        assert arm.enabled
        assert robot.estop_calls == estop_calls
        arm.enable()
        assert arm.enabled
        assert robot.arm.mode_calls == ["pv"]
    finally:
        arm.close()


def test_clear_motor_faults_leaves_arm_disabled_until_enable() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    try:
        arm._faulted = True
        arm._fault_reason = "motor fault status=8"

        completed = arm.clear_motor_faults()

        assert completed == ("joint1",)
        assert robot.clear_error_joint_names == ["joint1"]
        assert not arm.enabled
        assert not arm.faulted
        assert arm.fault_reason is None
        arm.enable()
        assert arm.enabled
        assert robot.arm.mode_calls == ["pv", "pv"]
    finally:
        arm.close()


def test_safe_hold_send_failure_keeps_retrying_without_disabling() -> None:
    arm, robot = make_arm()
    try:
        arm.send_joint_positions([0.0])
        robot.arm.send_error = RuntimeError("hold bus failure")
        wait_for_fault(arm)
        assert arm.faulted
        assert arm.safe_holding
        assert arm.enabled
        assert robot.estop_calls == 0

        robot.arm.send_error = None
        arm.read_state()
        assert not arm.faulted
        assert not arm.safe_holding
    finally:
        arm.close()


@pytest.mark.parametrize(
    ("enable_gripper", "expected_names", "expects_gripper_state"),
    [
        (False, ["joint1"], False),
        (True, ["joint1", "gripper"], True),
        (None, ["joint1", "gripper"], True),
    ],
)
def test_read_state_requires_only_active_actuator_feedback(
    enable_gripper,
    expected_names,
    expects_gripper_state,
) -> None:
    config = ArxDCanConfig(
        arm_joints=(JOINT,),
        gripper=GRIPPER,
        watchdog_enabled=False,
    )
    arm = ArxDCanArm(config=config, enable_gripper=enable_gripper)
    robot = FakeRobot()
    arm.robot = robot
    arm.connect()
    try:
        state = arm.read_state()

        assert robot.last_state_joint_names == expected_names
        assert (state.gripper is not None) is expects_gripper_state
        if state.gripper is not None:
            assert state.gripper.opening == pytest.approx(1000.0 * 0.42 / 2.64)
            assert state.gripper.motor_position == pytest.approx(0.42)
            assert state.gripper.position == pytest.approx(0.42)
            assert state.gripper.motor_velocity == pytest.approx(0.42)
            assert state.gripper.velocity == pytest.approx(0.42)
    finally:
        arm.close()


def test_explicit_gripper_requires_model_configuration() -> None:
    config = ArxDCanConfig(
        arm_joints=(JOINT,),
        gripper=None,
        watchdog_enabled=False,
    )

    with pytest.raises(ValueError, match="does not configure a gripper"):
        ArxDCanArm(config=config, enable_gripper=True)
