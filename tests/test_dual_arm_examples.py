from pathlib import Path

from arx_d_can.examples import example_13_set_zero_current_position as set_zero


def test_product_examples_are_flat_and_exclude_removed_runtime_demo() -> None:
    root = Path(__file__).resolve().parents[1] / "arx_d_can" / "examples"
    examples = sorted(path.name for path in root.glob("example_*.py"))

    assert not (root / "dual_arm").exists()
    assert examples == [
        "example_02_switch_control_mode.py",
        "example_03_enable_disable.py",
        "example_04_read_state.py",
        "example_05_clear_faults.py",
        "example_06_send_position_pv.py",
        "example_07_send_position_mit.py",
        "example_08_set_gripper_openings.py",
        "example_09_benchmark_read_rate.py",
        "example_11_return_zero.py",
        "example_12_diagnose_status.py",
        "example_13_set_zero_current_position.py",
        "example_15_gravity_compensation.py",
        "example_16_record_gravity_trajectory.py",
        "example_17_replay_trajectory.py",
    ]


def test_set_zero_requires_enter_confirmation(monkeypatch) -> None:
    calls: list[str] = []

    class FakeRobot:
        def connect(self) -> None:
            calls.append("connect")

        def set_zero(self) -> bool:
            calls.append("set_zero")
            return True

        def disconnect(self) -> None:
            calls.append("disconnect")

    def confirm(prompt: str) -> str:
        assert calls == ["connect"]
        assert "按回车继续" in prompt
        calls.append("confirm")
        return ""

    monkeypatch.setattr(set_zero, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr("builtins.input", confirm)

    set_zero.main()

    assert calls == ["connect", "confirm", "set_zero", "disconnect"]
