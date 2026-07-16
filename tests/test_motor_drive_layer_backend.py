from arx_d_can.driver import build_scan_command, parse_scan_ids


def test_build_scan_command_uses_motor_drive_layer_cli():
    command = build_scan_command(
        python_executable="/usr/bin/python3",
        port="/dev/ttyACM4",
        baud=921600,
        model="4340P",
        start_id=1,
        end_id=7,
        feedback_base="0x10",
        timeout_ms=30,
    )

    assert command[:4] == [
        "/usr/bin/python3",
        "-m",
        "motor_drive_layer.cli",
        "scan",
    ]
    assert command[command.index("--serial-port") + 1] == "/dev/ttyACM4"
    assert command[command.index("--end-id") + 1] == "7"


def test_parse_scan_ids_accepts_motor_drive_layer_hit_lines_only():
    output = """\
[..] id=0x1 feedback_id=0x11
[hit] id=0x2 feedback_id=0x12 model=4340P
[hit] id=0x0A feedback_id=0x1A model=4340P
scan complete
"""

    assert parse_scan_ids(output) == [2, 10]
