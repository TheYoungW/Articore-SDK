from __future__ import annotations

from arx_d_can.examples.diagnostics import example_04_read_pose as example


def test_read_pose_example_reads_both_sides_without_enabling(monkeypatch, capsys) -> None:
    calls: list[tuple] = []

    class FakeRobot:
        def connect(self) -> None:
            calls.append(("connect",))

        def get_pose(self, side: str) -> list[float]:
            calls.append(("get_pose", side))
            offset = 0.0 if side == "left" else 1.0
            return [offset + index / 10 for index in range(6)]

        def disconnect(self) -> None:
            calls.append(("disconnect",))

    monkeypatch.setattr(example, "ArxDCanDualArm", FakeRobot)

    example.main()

    assert calls == [
        ("connect",),
        ("get_pose", "left"),
        ("get_pose", "right"),
        ("disconnect",),
    ]
    output = capsys.readouterr().out
    assert "[x, y, z, roll, pitch, yaw]" in output
    assert "left:" in output
    assert "right:" in output
