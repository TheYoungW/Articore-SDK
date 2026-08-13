from __future__ import annotations

from types import SimpleNamespace

from arx_d_can.examples.single_arm import example_02_read_state as example


def test_reads_and_prints_state_inline(monkeypatch) -> None:
    state = SimpleNamespace(
        arm=SimpleNamespace(
            names=("joint1",),
            positions=(0.0,),
            velocities=(0.0,),
            torques=(0.0,),
        ),
        gripper=SimpleNamespace(opening=500.0),
    )
    close_calls: list[bool] = []
    arm = SimpleNamespace(
        connect=lambda: None,
        read_state=lambda: state,
        close=lambda: close_calls.append(True),
    )
    output: list[str] = []

    def capture_print(*args, **_kwargs) -> None:
        output.append(" ".join(str(value) for value in args))

    monkeypatch.setattr(example, "ArxDCanArm", lambda **_kwargs: arm)
    monkeypatch.setattr("builtins.print", capture_print)

    example.main(
        SimpleNamespace(
            arm_model=None,
            port="/dev/ttyACM0",
            baud=1_000_000,
            transport="dm-serial",
        )
    )

    assert any("关节角度 (deg)" in line for line in output)
    assert any("关节速度 (rad/s)" in line for line in output)
    assert any("夹爪开合度:        500 / 1000" in line for line in output)
    assert close_calls == [True]
