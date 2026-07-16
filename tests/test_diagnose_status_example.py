from types import SimpleNamespace

from arx_d_can.actuator import JointCfg
from arx_d_can.examples import example_09_diagnose_status as example


class FakeController:
    def __init__(self):
        self.timeouts = []

    def request_feedback_all(self, timeout_ms):
        self.timeouts.append(timeout_ms)


class FakeMotor:
    def __init__(self, *, status_code, mode, rotor_temperature=30.0):
        self.status_code = status_code
        self.mode = mode
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

    def get_register_u32(self, register, timeout_ms):
        assert register == example.CTRL_MODE_REGISTER
        assert timeout_ms == 100
        return self.mode


def joint(name, motor_id):
    return JointCfg(
        name=name,
        motor_id=motor_id,
        feedback_id=0x10 + motor_id,
        model="4310",
    )


def test_read_diagnostics_reports_fault_and_actual_control_mode():
    controller = FakeController()
    diagnostics = example.read_diagnostics(
        controller,
        [
            (joint("joint4", 4), FakeMotor(status_code=0xC, mode=2)),
            (joint("gripper", 7), FakeMotor(status_code=0x0, mode=1)),
        ],
        timeout_ms=100,
    )

    assert controller.timeouts == [100]
    assert diagnostics[0].status_code == 0xC
    assert diagnostics[0].control_mode == 2
    assert example.status_name(diagnostics[0].status_code) == "COIL_OVER_TEMPERATURE"
    assert example.mode_name(diagnostics[0].control_mode) == "POS_VEL"
    assert diagnostics[1].status_code == 0x0
    assert diagnostics[1].control_mode == 1


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

    example.print_summary(diagnostics, temperature_warning=80.0)

    output = capsys.readouterr().out
    assert "joint4=0xC(COIL_OVER_TEMPERATURE)" in output
    assert "joint5(mos=30C,rotor=194C)" in output
    assert "do not enable" in output
