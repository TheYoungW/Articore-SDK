from types import SimpleNamespace

import pytest

from arx_d_can.actuator import arx_d_can as actuator_module
from arx_d_can.actuator.arx_d_can import ArxDCan, JointCfg, JointGroup
from arx_d_can.sdk import ArxDCanArm
from arx_d_can.service_tools import zero_current_position as zero_tool


class FakeController:
    def __init__(self):
        self.added_motors = []

    def add_damiao_motor(self, motor_id, feedback_id, model):
        self.added_motors.append((motor_id, feedback_id, model))
        return object()


def make_uninitialized_arm(joint: JointCfg) -> ArxDCan:
    arm = ArxDCan.__new__(ArxDCan)
    arm._all_joints = [joint]
    arm._ctrl_map = {}
    arm._motor_map = {}
    arm._fake_controller = FakeController()
    arm._make_controller = lambda: arm._fake_controller
    return arm


def test_setup_motors_registers_damiao_motor_without_motor_brand_config():
    arm = make_uninitialized_arm(
        JointCfg(
            name="joint1",
            motor_id=1,
            feedback_id=0x11,
            model="4340P",
        )
    )

    arm._setup_motors()

    assert list(arm._motor_map) == ["joint1"]
    assert arm._fake_controller.added_motors == [(1, 0x11, "4340P")]


@pytest.mark.parametrize(
    ("enable_gripper", "expected_names"),
    [
        (False, ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")),
        (
            True,
            ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"),
        ),
    ],
)
def test_high_level_arm_registers_only_active_actuators(
    enable_gripper,
    expected_names,
):
    arm = ArxDCanArm(enable_gripper=enable_gripper)

    assert tuple(arm.robot.joint_names) == expected_names
    assert arm.robot.has_gripper is enable_gripper


class FakeZeroMotor:
    def __init__(
        self,
        *,
        position=0.4,
        feedback=True,
        status_code=0,
        clear_error=None,
        velocity=0.0,
        zero_feedback=None,
    ):
        self.position = position
        self.velocity = velocity
        self.feedback = feedback
        self.status_code = status_code
        self.clear_error_exception = clear_error
        self.zero_feedback = list(zero_feedback or ())
        self.zero_feedback_index = 0
        self.fresh_requests = 0
        self.clear_error_calls = 0
        self.zero_writes = 0

    def disable(self):
        pass

    def request_feedback(self):
        if not self.feedback:
            raise RuntimeError("no feedback")

    def request_fresh_state(self, timeout_ms=50):
        del timeout_ms
        self.fresh_requests += 1
        self.request_feedback()
        if self.zero_writes and self.zero_feedback:
            index = min(self.zero_feedback_index, len(self.zero_feedback) - 1)
            self.zero_feedback_index += 1
            position, velocity, status_code = self.zero_feedback[index]
            return SimpleNamespace(
                pos=position,
                vel=velocity,
                torq=0.0,
                status_code=status_code,
            )
        return self.get_state()

    def get_state(self):
        if not self.feedback:
            return None
        return SimpleNamespace(
            pos=self.position,
            vel=self.velocity,
            torq=0.0,
            status_code=self.status_code,
        )

    def set_zero_position(self):
        self.zero_writes += 1
        self.position = 0.0

    def clear_error(self):
        self.clear_error_calls += 1
        if self.clear_error_exception is not None:
            raise self.clear_error_exception
        self.status_code = 0


class FakePollController:
    def request_feedback_all(self, timeout_ms=50):
        del timeout_ms
        pass


def make_zero_arm(*motors):
    arm = ArxDCan.__new__(ArxDCan)
    arm._all_joints = [
        JointCfg(
            name=f"joint{index}",
            motor_id=index,
            feedback_id=0x10 + index,
            model="4340P",
        )
        for index in range(1, len(motors) + 1)
    ]
    arm._motor_map = {
        joint.name: motor for joint, motor in zip(arm._all_joints, motors)
    }
    arm._ctrl_map = {"main": FakePollController()}
    return arm


def test_zero_preflight_prevents_partial_writes(monkeypatch):
    monkeypatch.setattr(actuator_module.time, "sleep", lambda _seconds: None)
    first = FakeZeroMotor()
    second = FakeZeroMotor(feedback=False)
    arm = make_zero_arm(first, second)

    with pytest.raises(RuntimeError, match="joint2: healthy feedback unavailable"):
        arm.set_zero(poll_max=2, poll_interval=0.0)

    assert first.zero_writes == 0
    assert second.zero_writes == 0


def test_zero_writes_and_verifies_selected_motor(monkeypatch):
    monkeypatch.setattr(actuator_module.time, "sleep", lambda _seconds: None)
    first = FakeZeroMotor(position=0.4)
    second = FakeZeroMotor(position=-0.2)
    arm = make_zero_arm(first, second)

    completed = arm.set_zero(
        joint_names=["joint1"],
        poll_max=2,
        poll_interval=0.0,
    )

    assert completed == ("joint1",)
    assert first.zero_writes == 1
    assert first.fresh_requests == 4  # one preflight plus three verification frames
    assert second.zero_writes == 0


def test_zero_rejects_any_nonzero_consecutive_feedback_sample(monkeypatch):
    monkeypatch.setattr(actuator_module.time, "sleep", lambda _seconds: None)
    motor = FakeZeroMotor(
        position=0.4,
        zero_feedback=[
            (0.0, 0.0, 0),
            (0.03, 0.0, 0),
            (0.0, 0.0, 0),
        ],
    )
    arm = make_zero_arm(motor)

    with pytest.raises(RuntimeError, match=r"fresh sample 2/3.*position=\+0.030000"):
        arm.set_zero(poll_max=2, poll_interval=0.0)


def test_zero_rejects_feedback_velocity_after_write(monkeypatch):
    monkeypatch.setattr(actuator_module.time, "sleep", lambda _seconds: None)
    motor = FakeZeroMotor(
        position=-0.4,
        zero_feedback=[(0.0, 0.06, 0)],
    )
    arm = make_zero_arm(motor)

    with pytest.raises(RuntimeError, match=r"velocity=\+0.060000"):
        arm.set_zero(poll_max=2, poll_interval=0.0)


def test_zero_rejects_any_faulted_verification_frame(monkeypatch):
    monkeypatch.setattr(actuator_module.time, "sleep", lambda _seconds: None)
    motor = FakeZeroMotor(
        position=0.4,
        zero_feedback=[
            (0.0, 0.0, 0),
            (0.0, 0.0, 8),
            (0.0, 0.0, 0),
        ],
    )
    arm = make_zero_arm(motor)

    with pytest.raises(RuntimeError, match=r"fresh sample 2/3: motor status=8"):
        arm.set_zero(poll_max=2, poll_interval=0.0)


def test_clear_errors_clears_and_verifies_every_selected_motor(monkeypatch):
    monkeypatch.setattr(actuator_module.time, "sleep", lambda _seconds: None)
    first = FakeZeroMotor(status_code=8)
    second = FakeZeroMotor(status_code=12)
    arm = make_zero_arm(first, second)

    completed = arm.clear_errors(poll_max=2, poll_interval=0.0)

    assert completed == ("joint1", "joint2")
    assert first.clear_error_calls == 1
    assert second.clear_error_calls == 1
    assert first.status_code == 0
    assert second.status_code == 0


def test_clear_errors_attempts_remaining_motors_after_one_failure(monkeypatch):
    monkeypatch.setattr(actuator_module.time, "sleep", lambda _seconds: None)
    first = FakeZeroMotor(
        status_code=8,
        clear_error=RuntimeError("simulated clear failure"),
    )
    second = FakeZeroMotor(status_code=12)
    arm = make_zero_arm(first, second)

    with pytest.raises(
        RuntimeError,
        match=r"cleared=\['joint2'\].*joint1: simulated clear failure",
    ):
        arm.clear_errors(poll_max=2, poll_interval=0.0)

    assert first.clear_error_calls == 1
    assert second.clear_error_calls == 1


@pytest.mark.parametrize("status_code", [0, 1])
def test_global_state_accepts_disabled_and_enabled_status(status_code):
    arm = make_zero_arm(FakeZeroMotor(status_code=status_code))

    positions, velocities, torques = arm.get_state(request_feedback=False)

    assert positions.tolist() == [0.4]
    assert velocities.tolist() == [0.0]
    assert torques.tolist() == [0.0]


def test_global_state_rejects_damiao_fault_status():
    arm = make_zero_arm(FakeZeroMotor(status_code=8))

    with pytest.raises(RuntimeError, match="joint1: motor fault status=8"):
        arm.get_state(request_feedback=False)


def test_global_state_can_require_only_selected_joint_feedback():
    arm = make_zero_arm(
        FakeZeroMotor(position=0.4),
        FakeZeroMotor(feedback=False),
    )

    positions, velocities, torques = arm.get_state(
        request_feedback=False,
        require_complete=True,
        joint_names=["joint1"],
    )

    assert positions.tolist() == [0.4]
    assert velocities.tolist() == [0.0]
    assert torques.tolist() == [0.0]


def test_global_state_rejects_unknown_selected_joint():
    arm = make_zero_arm(FakeZeroMotor())

    with pytest.raises(ValueError, match="unknown joints: gripper"):
        arm.get_state(request_feedback=False, joint_names=["gripper"])


@pytest.mark.parametrize("status_code", [0, 1])
def test_joint_group_state_accepts_disabled_and_enabled_status(status_code):
    arm = make_zero_arm(FakeZeroMotor(status_code=status_code))
    group = JointGroup(
        "arm",
        ["joint1"],
        arm._all_joints,
        arm._motor_map,
        arm._ctrl_map,
    )

    positions, velocities, torques = group.read_state(request_feedback=False)

    assert positions.tolist() == [0.4]
    assert velocities.tolist() == [0.0]
    assert torques.tolist() == [0.0]


def test_joint_group_state_rejects_damiao_fault_status():
    arm = make_zero_arm(FakeZeroMotor(status_code=8))
    group = JointGroup(
        "arm",
        ["joint1"],
        arm._all_joints,
        arm._motor_map,
        arm._ctrl_map,
    )

    with pytest.raises(RuntimeError, match="arm/joint1: motor fault status=8"):
        group.read_state(request_feedback=False)


@pytest.mark.parametrize("include_gripper", [False, True])
def test_zero_tool_requests_feedback_for_selected_actuators(
    monkeypatch,
    include_gripper,
):
    captured = {}

    class FakeArm:
        def connect(self):
            pass

        def set_zero(
            self,
            *,
            joint_names,
            verify_tolerance,
            verify_velocity,
            verify_samples,
        ):
            captured["joint_names"] = list(joint_names)
            captured["verify_tolerance"] = verify_tolerance
            captured["verify_velocity"] = verify_velocity
            captured["verify_samples"] = verify_samples
            return tuple(joint_names)

        def read_state(self, *, request_feedback=True):
            del request_feedback
            return state

        def close(self):
            pass

    fake_arm = FakeArm()

    def fake_make_arm(args, *, enable_gripper=False):
        captured["enable_gripper"] = enable_gripper
        return fake_arm

    state = SimpleNamespace(
        arm=SimpleNamespace(names=("joint1",), positions=(0.0,)),
        gripper=(
            SimpleNamespace(name="gripper", position=0.0)
            if include_gripper
            else None
        ),
    )
    monkeypatch.setattr(zero_tool, "make_arm", fake_make_arm)
    monkeypatch.setattr(zero_tool, "require_stationary", lambda *_args, **_kwargs: state)
    args = SimpleNamespace(
        include_gripper=include_gripper,
        stationary_seconds=1.0,
        stationary_hz=20.0,
        max_velocity=0.05,
        max_movement=0.01,
        verify_tolerance=0.02,
    )

    zero_tool.main(args)

    assert captured["enable_gripper"] is include_gripper
    assert captured["joint_names"] == (
        ["joint1", "gripper"] if include_gripper else ["joint1"]
    )
    assert captured["verify_velocity"] == 0.05
    assert captured["verify_samples"] == 3
