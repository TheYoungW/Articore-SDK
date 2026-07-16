"""motor-drive-layer integration kept behind the ARX driver boundary."""
from __future__ import annotations

import re

from motor_drive_layer import CallError, Controller, Mode


def build_scan_command(
    *,
    python_executable: str,
    port: str,
    baud: int,
    model: str,
    start_id: int,
    end_id: int,
    feedback_base: str,
    timeout_ms: int,
) -> list[str]:
    """Build the motor-drive-layer CLI command used for read-only ID scans."""
    return [
        python_executable,
        "-m",
        "motor_drive_layer.cli",
        "scan",
        "--vendor",
        "damiao",
        "--transport",
        "dm-serial",
        "--serial-port",
        port,
        "--serial-baud",
        str(baud),
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
    ]


def parse_scan_ids(output: str) -> list[int]:
    """Extract motor IDs from motor-drive-layer scan output."""
    return [
        int(match.group(1), 16)
        for match in re.finditer(
            r"^\[hit\]\s+id=0x([0-9A-Fa-f]+)\b",
            output,
            flags=re.MULTILINE,
        )
    ]
