"""motor-drive-layer integration kept behind the ARX driver boundary."""
from __future__ import annotations

import re
from math import isfinite

from motor_drive_layer import CallError, Controller, Mode

from ..errors import TransportError


SUPPORTED_TRANSPORTS = ("auto", "dm-serial", "socketcan", "socketcanfd")

_DAMIAO_MODEL_LIMITS: dict[str, tuple[float, float, float]] = {
    "3507": (12.566, 50.0, 5.0),
    "4310": (12.5, 30.0, 10.0),
    "4310P": (12.5, 50.0, 10.0),
    "4340": (12.5, 10.0, 28.0),
    "4340P": (12.5, 10.0, 28.0),
    "4340_v20": (12.5, 20.0, 28.0),
    "6006": (12.5, 45.0, 20.0),
    "8006": (12.5, 45.0, 40.0),
    "8009": (12.5, 45.0, 54.0),
    "10010L": (12.5, 25.0, 200.0),
    "10010": (12.5, 20.0, 200.0),
    "H3510": (12.5, 280.0, 1.0),
    "G6215": (12.5, 45.0, 10.0),
    "H6220": (12.5, 45.0, 10.0),
    "JH11": (12.5, 10.0, 12.0),
    "6248P": (12.566, 20.0, 120.0),
}


def resolve_transport(transport: str | None, channel: str) -> str:
    """Resolve an explicit transport or infer one for legacy configurations."""
    normalized = str(transport or "auto").strip().lower().replace("_", "-")
    if normalized not in SUPPORTED_TRANSPORTS:
        raise ValueError(
            f"unsupported transport {transport!r}; expected one of "
            + ", ".join(SUPPORTED_TRANSPORTS)
        )
    if normalized == "auto":
        return "dm-serial" if channel.startswith("/dev/tty") else "socketcan"
    return normalized


def create_controller(
    *,
    transport: str | None,
    channel: str,
    baud: int = 1_000_000,
) -> Controller:
    """Open a motor-drive-layer controller for one supported transport."""
    resolved = resolve_transport(transport, channel)
    try:
        if resolved == "dm-serial":
            return Controller.from_dm_serial(channel, baud)
        if resolved == "socketcan":
            return Controller(channel)
        if resolved == "socketcanfd":
            return Controller.from_socketcanfd(channel)
    except CallError as exc:
        raise TransportError(
            f"failed to open {resolved} transport on {channel}: {exc}",
            operation="open",
            transport=resolved,
            channel=channel,
            retryable=True,
        ) from exc
    raise AssertionError(f"unhandled transport: {resolved}")


def build_scan_command(
    *,
    python_executable: str,
    port: str,
    baud: int,
    transport: str = "auto",
    model: str,
    start_id: int,
    end_id: int,
    feedback_base: str,
    timeout_ms: int,
) -> list[str]:
    """Build the motor-drive-layer CLI command used for read-only ID scans."""
    resolved = resolve_transport(transport, port)
    command = [
        python_executable,
        "-m",
        "motor_drive_layer.cli",
        "scan",
        "--vendor",
        "damiao",
        "--transport",
        resolved,
    ]
    if resolved == "dm-serial":
        command.extend(["--serial-port", port, "--serial-baud", str(baud)])
    else:
        command.extend(["--channel", port])
    command.extend([
        "--model",
        model,
        "--start-id",
        str(start_id),
        "--end-id",
        str(end_id),
        "--feedback-base",
        feedback_base,
        "--timeout-ms",
        str(timeout_ms),
    ])
    return command


def _state_field(state: str, name: str) -> str | None:
    match = re.search(rf"(?:^|,\s*){re.escape(name)}=([^,\)]+)", state)
    return None if match is None else match.group(1).strip()


def _valid_scan_hit(line: str, *, model: str | None) -> int | None:
    match = re.match(
        r"^\[hit\]\s+id=(0x[0-9A-Fa-f]+|\d+)\s+"
        r"feedback_id=(0x[0-9A-Fa-f]+|\d+)\s+"
        r"state=MotorState\((.*)\)\s*$",
        line,
    )
    if match is None:
        return None
    motor_id = int(match.group(1), 0)
    expected_feedback_id = int(match.group(2), 0)
    state = match.group(3)
    arbitration_text = _state_field(state, "arbitration_id")
    status_text = _state_field(state, "status_code")
    numeric_names = ("pos", "vel", "torq", "t_mos", "t_rotor")
    numeric_text = [_state_field(state, name) for name in numeric_names]
    if arbitration_text is None or status_text is None or any(
        value is None for value in numeric_text
    ):
        return None
    try:
        arbitration_id = int(arbitration_text, 0)
        status_code = int(status_text, 0)
        pos, vel, torq, t_mos, t_rotor = (
            float(value) for value in numeric_text if value is not None
        )
    except ValueError:
        return None
    if arbitration_id != expected_feedback_id or not 0 <= status_code <= 0xE:
        return None
    values = (pos, vel, torq, t_mos, t_rotor)
    if not all(isfinite(value) for value in values):
        return None
    if not (-40.0 <= t_mos < 250.0 and -40.0 <= t_rotor < 250.0):
        return None
    limits = _DAMIAO_MODEL_LIMITS.get(str(model))
    if limits is not None and all(
        abs(value) >= limit * 0.999
        for value, limit in zip((pos, vel, torq), limits)
    ):
        return None
    return motor_id


def parse_scan_ids(output: str, *, model: str | None = None) -> list[int]:
    """Extract only validated motor IDs from motor-drive-layer scan output."""
    return [
        motor_id
        for line in output.splitlines()
        if (motor_id := _valid_scan_hit(line, model=model)) is not None
    ]
