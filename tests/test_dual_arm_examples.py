from pathlib import Path

from arx_d_can.examples.maintenance import example_02_recover_to_zero as recover
from arx_d_can.examples.maintenance import (
    example_03_set_zero_current_position as set_zero,
)


def test_product_examples_are_grouped_into_three_purpose_directories() -> None:
    root = Path(__file__).resolve().parents[1] / "arx_d_can" / "examples"
    directories = sorted(
        path.name for path in root.iterdir()
        if path.is_dir() and not path.name.startswith("__")
    )

    assert directories == ["control", "diagnostics", "maintenance"]
    assert not list(root.glob("example_*.py"))
    assert sorted(path.name for path in (root / "control").glob("example_*.py")) == [
        "example_01_switch_control_mode.py",
        "example_02_enable_disable.py",
        "example_03_send_position_pv.py",
        "example_04_send_position_mit.py",
        "example_05_set_gripper_openings.py",
        "example_06_return_zero.py",
        "example_07_cartesian_circular.py",
        "example_07_cartesian_linear.py",
        "example_07_cartesian_ptp.py",
        "example_08_gravity_compensation.py",
        "example_09_record_gravity_trajectory.py",
        "example_10_replay_trajectory.py",
        "example_11_bimanual_follow.py",
        "example_12_tcp_offset.py",
    ]
    assert sorted(
        path.name for path in (root / "diagnostics").glob("example_*.py")
    ) == [
        "example_01_read_state.py",
        "example_02_benchmark_read_rate.py",
        "example_03_read_health.py",
        "example_04_read_pose.py",
    ]
    assert sorted(
        path.name for path in (root / "maintenance").glob("example_*.py")
    ) == [
        "example_01_clear_faults.py",
        "example_02_recover_to_zero.py",
        "example_03_set_zero_current_position.py",
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


def test_recover_example_confirms_then_recovers(monkeypatch) -> None:
    calls: list[str] = []

    class FakeRobot:
        def connect(self) -> None:
            calls.append("connect")

        def recover(self) -> None:
            calls.append("recover")

        def disconnect(self) -> None:
            calls.append("disconnect")

    def confirm(prompt: str) -> str:
        assert calls == ["connect"]
        assert "低速回到已标定零点" in prompt
        assert "按回车继续" in prompt
        calls.append("confirm")
        return ""

    monkeypatch.setattr(recover, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr("builtins.input", confirm)

    recover.main()

    assert calls == ["connect", "confirm", "recover", "disconnect"]


def test_recover_example_handles_connected_configuration_failure(
    monkeypatch,
) -> None:
    calls: list[str] = []

    class FakeRobot:
        connected = True

        def connect(self) -> None:
            calls.append("connect")
            raise recover.RuntimeTransactionError("configure failed", object())

        def recover(self) -> None:
            calls.append("recover")

        def disconnect(self) -> None:
            calls.append("disconnect")

    monkeypatch.setattr(recover, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr("builtins.input", lambda _prompt: "")

    recover.main()

    assert calls == ["connect", "recover", "disconnect"]
