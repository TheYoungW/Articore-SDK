from __future__ import annotations

from types import SimpleNamespace

from arx_d_can.examples import example_02_read_state as example


def test_print_state_flushes_streaming_output(monkeypatch) -> None:
    state = SimpleNamespace(
        arm=SimpleNamespace(
            names=("joint1",),
            positions=(0.0,),
            velocities=(0.0,),
        ),
        gripper=None,
    )
    arm = SimpleNamespace(read_state=lambda: state)
    calls: list[dict[str, object]] = []

    def capture_print(*_args, **kwargs) -> None:
        calls.append(kwargs)

    monkeypatch.setattr("builtins.print", capture_print)

    example.print_state(arm, sample_index=1)

    assert len(calls) == 2
    assert all(call.get("flush") is True for call in calls)
