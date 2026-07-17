from arx_d_can.examples import example_10_set_zero_current_position as example


def test_zero_example_runs_with_default_arm_only_selection(monkeypatch):
    captured = {}
    monkeypatch.setattr(
        example,
        "zero_current_position",
        lambda args: captured.update(vars(args)),
    )

    example.main(["--port", "/dev/null"])

    assert captured["port"] == "/dev/null"
    assert captured["include_gripper"] is False


def test_zero_example_forwards_gripper_selection(monkeypatch):
    captured = {}
    monkeypatch.setattr(
        example,
        "zero_current_position",
        lambda args: captured.update(vars(args)),
    )

    example.main(
        [
            "--port",
            "/dev/ttyACM3",
            "--include-gripper",
        ]
    )

    assert captured["port"] == "/dev/ttyACM3"
    assert captured["include_gripper"] is True
