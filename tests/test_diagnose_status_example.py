from contextlib import nullcontext
from types import SimpleNamespace

import pytest

from arx_d_can.actuator.arx_d_can import JointCfg
from arx_d_can.sdk import diagnostics as example


class FakeMotor:
    def __init__(self, *, status_code, rotor_temperature=30.0):
        self.status_code = status_code
        self.rotor_temperature = rotor_temperature

    def get_state(self):
        return SimpleNamespace(
            status_code=self.status_code,
            pos=0.1,
            vel=0.2,
            torq=0.3,
            t_mos=31.0,
            t_rotor=self.rotor_temperature,
        )

def joint(name, motor_id):
    return JointCfg(
        name=name,
        motor_id=motor_id,
        feedback_id=0x10 + motor_id,
        model="4310",
        velocity_range=10.0,
    )


def test_read_diagnostics_reports_fault_and_configured_control_mode():
    joints = (joint("joint4", 4), joint("gripper", 7))
    motors = {
        "joint4": FakeMotor(status_code=0xC),
        "gripper": FakeMotor(status_code=0x0),
    }
    arm = SimpleNamespace(
        connected=True,
        _mode="posvel",
        _io_lock=nullcontext(),
        _active_joint_names=lambda: ["joint4", "gripper"],
        robot=SimpleNamespace(_all_joints=joints, _motor_map=motors),
    )
    diagnostics = example.read_motor_diagnostics(arm, timeout_ms=100)

    assert diagnostics[0].status_code == 0xC
    assert diagnostics[0].control_mode == 2
    assert example.status_name(diagnostics[0].status_code) == "COIL_OVER_TEMPERATURE"
    assert example.mode_name(diagnostics[0].control_mode) == "POS_VEL"
    assert example.mode_name(3) == "UNSUPPORTED"
    assert diagnostics[1].status_code == 0x0
    assert diagnostics[1].control_mode == 2
    assert diagnostics[0].velocity == pytest.approx(0.2 / 3.0)


def test_summary_warns_for_fault_and_abnormal_temperature(capsys):
    diagnostics = [
        example.MotorDiagnostic(
            name="joint4",
            motor_id=4,
            feedback_id=0x14,
            status_code=0xC,
            control_mode=2,
            mos_temperature=37.0,
            rotor_temperature=34.0,
        ),
        example.MotorDiagnostic(
            name="joint5",
            motor_id=5,
            feedback_id=0x15,
            status_code=0x0,
            control_mode=2,
            mos_temperature=30.0,
            rotor_temperature=194.0,
        ),
    ]

    example.print_diagnostic_summary(diagnostics, temperature_warning=80.0)

    output = capsys.readouterr().out
    assert "joint4=0xC(COIL_OVER_TEMPERATURE)" in output
    assert "joint5(mos=30C,rotor=194C)" in output
    assert "不要直接使能" in output
