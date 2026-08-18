from __future__ import annotations

import os
from importlib.metadata import PackageNotFoundError, distribution
from pathlib import Path


NATIVE_DISTRIBUTION = "motor-drive-layer"
NATIVE_PAYLOAD_PACKAGE = "motor_drive_layer_native"


def distribution_payload_candidates(*parts: str) -> list[Path]:
    """Locate native payload files without importing a motor Python package."""
    try:
        dist = distribution(NATIVE_DISTRIBUTION)
    except PackageNotFoundError:
        return []

    suffix = (NATIVE_PAYLOAD_PACKAGE, *parts)
    matches: list[Path] = []
    for entry in dist.files or ():
        entry_parts = tuple(entry.parts)
        if len(entry_parts) >= len(suffix) and entry_parts[-len(suffix) :] == suffix:
            matches.append(Path(dist.locate_file(entry)).resolve())
    return matches


def source_checkout_root() -> Path | None:
    override = os.getenv("MOTOR_DRIVE_LAYER_SOURCE")
    if override:
        return Path(override).expanduser().resolve()

    current = Path.cwd().resolve()
    for candidate in (current, *current.parents):
        if (candidate / "cpp_damiao").is_dir() and (
            candidate / "articore_runtime"
        ).is_dir():
            return candidate
    return None
