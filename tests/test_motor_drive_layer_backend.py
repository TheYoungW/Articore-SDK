import pytest

from arx_d_can import TransportError
from arx_d_can.driver import (
    CallError,
    build_scan_command,
    create_controller,
    parse_scan_ids,
    resolve_transport,
)
from arx_d_can.driver import motor_drive_layer_backend as backend


def build_command(*, port: str, transport: str) -> list[str]:
    return build_scan_command(
        python_executable="/usr/bin/python3",
        port=port,
        baud=921600,
        transport=transport,
        model="4340P",
        start_id=1,
        end_id=7,
        feedback_base="0x200",
        timeout_ms=30,
    )


def test_build_scan_command_uses_dm_serial_arguments():
    command = build_command(port="/dev/ttyACM4", transport="dm-serial")

    assert command[:4] == [
        "/usr/bin/python3",
        "-m",
        "motor_drive_layer.cli",
        "scan",
    ]
    assert command[command.index("--transport") + 1] == "dm-serial"
    assert command[command.index("--serial-port") + 1] == "/dev/ttyACM4"
    assert command[command.index("--serial-baud") + 1] == "921600"
    assert "--channel" not in command


def test_build_scan_command_uses_dm_device_arguments():
    command = build_command(port="1", transport="dm-device")

    assert command[command.index("--transport") + 1] == "dm-device"
    assert command[command.index("--dm-device-type") + 1] == "usb2canfd-dual"
    assert command[command.index("--dm-channel") + 1] == "1"
    assert command[command.index("--dm-bitrate") + 1] == "921600"
    assert command[command.index("--dm-data-bitrate") + 1] == "5000000"
    assert "--channel" not in command


@pytest.mark.parametrize("transport", ["socketcan", "socketcanfd"])
def test_build_scan_command_uses_socketcan_channel(transport: str):
    command = build_command(port="can0", transport=transport)

    assert command[command.index("--transport") + 1] == transport
    assert command[command.index("--channel") + 1] == "can0"
    assert "--serial-port" not in command
    if transport == "socketcanfd":
        assert command[command.index("--socketcanfd-brs") + 1] == "on"
    else:
        assert "--socketcanfd-brs" not in command


def test_auto_transport_preserves_legacy_channel_inference():
    assert resolve_transport("auto", "/dev/ttyACM0") == "dm-serial"
    assert resolve_transport(None, "can0") == "socketcan"
    with pytest.raises(ValueError, match="unsupported transport"):
        resolve_transport("bluetooth", "can0")


def test_create_controller_selects_matching_motor_drive_layer_constructor(monkeypatch):
    calls = []

    class FakeController:
        def __init__(self, channel):
            calls.append(("socketcan", channel))

        @classmethod
        def from_dm_serial(cls, channel, baud):
            calls.append(("dm-serial", channel, baud))
            return object()

        @classmethod
        def from_dm_device(cls, **kwargs):
            calls.append(("dm-device", kwargs))
            return object()

        @classmethod
        def from_socketcanfd(cls, channel, enable_brs=True):
            calls.append(("socketcanfd", channel, enable_brs))
            return object()

    monkeypatch.setattr(backend, "Controller", FakeController)

    create_controller(transport="dm-serial", channel="/dev/ttyACM0", baud=500000)
    create_controller(transport="dm-device", channel="1", baud=1000000)
    create_controller(transport="socketcan", channel="can0")
    create_controller(transport="socketcanfd", channel="can1")

    assert calls == [
        ("dm-serial", "/dev/ttyACM0", 500000),
        (
            "dm-device",
            {
                "device": "usb2canfd-dual",
                "channel": "1",
                "bitrate": 1000000,
                "data_bitrate": 5000000,
            },
        ),
        ("socketcan", "can0"),
        ("socketcanfd", "can1", True),
    ]


def test_create_controller_wraps_vendor_call_error(monkeypatch):
    class FailingController:
        def __init__(self, channel):
            raise CallError(f"cannot open {channel}")

    monkeypatch.setattr(backend, "Controller", FailingController)

    with pytest.raises(TransportError) as caught:
        create_controller(transport="socketcan", channel="can9")

    assert caught.value.operation == "open"
    assert caught.value.transport == "socketcan"
    assert caught.value.channel == "can9"
    assert caught.value.retryable
    assert isinstance(caught.value.__cause__, CallError)


def state(
    arbitration_id: int,
    *,
    status_code: int = 0,
    pos: float = 1.0695,
    vel: float = 0.0,
    torq: float = 0.0,
    t_mos: float = 30.0,
    t_rotor: float = 31.0,
) -> str:
    return (
        "MotorState(can_id=6, arbitration_id="
        f"{arbitration_id}, status_code={status_code}, pos={pos}, vel={vel}, "
        f"torq={torq}, t_mos={t_mos}, t_rotor={t_rotor})"
    )


def test_parse_scan_ids_accepts_only_validated_hit_lines():
    output = "\n".join(
        [
            "[.. ] id=0x1 no reply: timeout",
            f"[hit] id=0x6 feedback_id=0x206 state={state(0x206)}",
            # 仲裁 ID 错误：这是已报告的 ID 15 误检情况。
            f"[hit] id=0xF feedback_id=0x20f state={state(0x206)}",
            f"[hit] id=0x7 feedback_id=0x207 state={state(0x207, t_mos=255)}",
            f"[hit] id=0x8 feedback_id=0x208 state={state(0x208, pos=float('nan'))}",
            # 位置、速度和力矩均处于该型号的编码极限。
            f"[hit] id=0x9 feedback_id=0x209 state={state(0x209, pos=12.5, vel=10, torq=28)}",
            "[hit] id=0xA feedback_id=0x20A model=4340P",
        ]
    )

    assert parse_scan_ids(output, model="4340P") == [6]


def test_parse_scan_ids_accepts_dm_device_arbitration_id_mapping():
    output = (
        f"[hit] id=0x6 feedback_id=0x16 "
        f"state={state(0x206)}"
    )

    assert parse_scan_ids(output, model="4340P") == [6]
