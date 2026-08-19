from __future__ import annotations

from types import SimpleNamespace

from arx_d_can.examples import example_03_enable_disable as example


def test_enable_disable_example_runs_interactive_sequence(monkeypatch) -> None:
    events: list[object] = []

    class FakeRobot:
        def __init__(self, **kwargs) -> None:
            events.append(("create", kwargs))

        def connect(self) -> None:
            events.append("connect")

        def enable(self) -> None:
            events.append("enable")

        def read_cached_state(self):
            events.append("read_cached_state")
            return SimpleNamespace(
                left=SimpleNamespace(positions=(1.0, 2.0)),
                right=SimpleNamespace(positions=(3.0, 4.0)),
            )

        def set_joint_pv(self, *, left, right) -> None:
            events.append(("set_joint_pv", left, right))

        def disable(self) -> None:
            events.append("disable")

        def disconnect(self) -> None:
            events.append("close")

    prompts: list[str] = []
    monkeypatch.setattr(example, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr(
        "builtins.input",
        lambda prompt: prompts.append(prompt) or "",
    )

    example.main()

    assert events == [
        ("create", {"control_mode": "pv"}),
        "connect",
        "enable",
        "read_cached_state",
        ("set_joint_pv", (1.0, 2.0), (3.0, 4.0)),
        "disable",
        "close",
    ]
    assert len(prompts) == 2


def test_enable_disable_example_closes_after_interruption(monkeypatch) -> None:
    events: list[str] = []

    class FakeRobot:
        def __init__(self, **_kwargs) -> None:
            pass

        def connect(self) -> None:
            events.append("connect")

        def disconnect(self) -> None:
            events.append("close")

    monkeypatch.setattr(example, "ArxDCanDualArm", FakeRobot)
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt: (_ for _ in ()).throw(KeyboardInterrupt),
    )

    example.main()

    assert events == ["connect", "close"]
