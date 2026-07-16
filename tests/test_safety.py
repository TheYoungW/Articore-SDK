import time

import numpy as np
import pytest

from arx_d_can import ArxDCanArm, ArxDCanConfig, JointMotorConfig


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
        self.sent_mit: list[np.ndarray] = []
        self.sent_mit_torques: list[np.ndarray | None] = []

    def mode_pos_vel(self) -> bool:
        self.mode_calls.append("pv")
        return True

    def mode_mit(self) -> bool:
        self.mode_calls.append("mit")
        return True

    def enable(self) -> None:
        self.enabled = True

    def send_pos_vel(self, target, *, strict=True) -> None:
        if self.send_error is not None:
            raise self.send_error
        self.sent_pos_vel.append(np.asarray(target, dtype=np.float64).copy())

    def send_mit(self, target, *, strict=True, **kwargs) -> None:
        if self.send_error is not None:
            raise self.send_error
        self.sent_mit.append(np.asarray(target, dtype=np.float64).copy())
        torque = kwargs.get("tau")
        self.sent_mit_torques.append(
            None if torque is None else np.asarray(torque, dtype=np.float64).copy()
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

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        self.disconnect_calls += 1
        self.estop()

    def estop(self) -> None:
        self.estop_calls += 1
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
        self.last_state_joint_names = None if joint_names is None else list(joint_names)
        count = 1 if joint_names is None else len(joint_names)
        values = np.full(count, self.position)
        return values, values.copy(), values.copy()


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


def wait_for_fault(arm: ArxDCanArm, timeout: float = 0.5) -> None:
    deadline = time.monotonic() + timeout
    while not arm.faulted and time.monotonic() < deadline:
        time.sleep(0.005)
    assert arm.faulted


def test_watchdog_holds_actual_position_and_latches_fault() -> None:
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
        np.testing.assert_allclose(robot.arm.sent_pos_vel[-1], [robot.position])
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


def test_send_failure_disables_and_latches_fault() -> None:
    arm, robot = make_arm(timeout=1.0, grace=1.0)
    robot.arm.send_error = RuntimeError("simulated bus failure")
    try:
        with pytest.raises(RuntimeError, match="simulated bus failure"):
            arm.send_joint_positions([0.0])
        assert arm.faulted
        assert not arm.enabled
        assert robot.estop_calls >= 1
    finally:
        arm.close()


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
        torques = [0.1] if mode == "mit" else None
        arm.send_joint_positions([0.25], torques=torques)

        assert robot.arm.mode_calls == [mode]
        if mode == "pv":
            np.testing.assert_allclose(robot.arm.sent_pos_vel[-1], [0.25])
            assert robot.arm.sent_mit == []
        else:
            np.testing.assert_allclose(robot.arm.sent_mit[-1], [0.25])
            np.testing.assert_allclose(robot.arm.sent_mit_torques[-1], [0.1])
            assert robot.arm.sent_pos_vel == []
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


def test_clear_fault_requires_explicit_reconfigure_and_enable() -> None:
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
        with pytest.raises(RuntimeError, match="configured before enable"):
            arm.enable()
        arm.configure()
        arm.enable()
        assert arm.enabled
    finally:
        arm.close()


def test_clear_motor_faults_clears_hardware_and_leaves_arm_disabled() -> None:
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
        with pytest.raises(RuntimeError, match="configured before enable"):
            arm.enable()
    finally:
        arm.close()


def test_safe_hold_send_failure_promotes_to_hard_fault() -> None:
    arm, robot = make_arm()
    try:
        arm.send_joint_positions([0.0])
        robot.arm.send_error = RuntimeError("hold bus failure")
        wait_for_fault(arm)
        deadline = time.monotonic() + 0.2
        while arm.safe_holding and time.monotonic() < deadline:
            time.sleep(0.005)
        assert arm.faulted
        assert not arm.safe_holding
        assert not arm.enabled
        assert "safe hold command failed" in (arm.fault_reason or "")
        assert robot.estop_calls >= 1
    finally:
        arm.close()


@pytest.mark.parametrize(
    ("enable_gripper", "expected_names", "expects_gripper_state"),
    [
        (False, ["joint1"], False),
        (True, ["joint1", "gripper"], True),
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
        state = arm.read_state(request_feedback=True)

        assert robot.last_state_joint_names == expected_names
        assert (state.gripper is not None) is expects_gripper_state
    finally:
        arm.close()
