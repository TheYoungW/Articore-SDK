from __future__ import annotations

from arx_d_can.controllers import dual_gravity_compensation as module


def test_dual_gravity_start_run_and_shutdown(monkeypatch) -> None:
    calls: list[str] = []

    class FakeMode:
        def __init__(self, arm, **_kwargs) -> None:
            self.name = arm

        def start(self) -> None:
            calls.append(f"start:{self.name}")

        def step(self) -> None:
            calls.append(f"step:{self.name}")

        def stop(self) -> None:
            calls.append(f"stop:{self.name}")

    class FakeRobot:
        left = "left"
        right = "right"
        connected = False

        def connect(self) -> None:
            calls.append("connect")
            self.connected = True

        def close(self) -> None:
            calls.append("close")
            self.connected = False

    monkeypatch.setattr(module, "GravityCompensationMode", FakeMode)
    robot = FakeRobot()
    gravity = module.DualArmGravityCompensationMode(robot, hz=100.0)

    gravity.start()
    gravity.run(seconds=0.001)
    gravity.shutdown()

    assert calls[:3] == ["connect", "start:left", "start:right"]
    assert "step:left" in calls
    assert "step:right" in calls
    assert calls[-3:] == ["stop:left", "stop:right", "close"]
