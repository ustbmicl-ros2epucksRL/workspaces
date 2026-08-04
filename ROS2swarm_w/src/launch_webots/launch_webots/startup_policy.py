"""Small launch policies that are enabled only by the platform wrapper."""

from __future__ import annotations

import math
import os


def controller_start_delay_seconds(robot_index: int) -> float:
    """Return the opt-in per-robot controller stagger used by platform Runs."""
    if robot_index < 0:
        raise ValueError("robot_index must be non-negative")
    mode = os.environ.get("MASAR_PLATFORM_MODE", "").strip().lower()
    if mode not in {"1", "true", "yes", "on"}:
        return 0.0
    raw_delay = os.environ.get("MASAR_PLATFORM_CONTROLLER_START_DELAY_SECONDS", "1.0")
    try:
        delay = float(raw_delay)
    except ValueError as exc:
        raise ValueError("MASAR_PLATFORM_CONTROLLER_START_DELAY_SECONDS must be numeric") from exc
    if not math.isfinite(delay) or delay < 0:
        raise ValueError("MASAR_PLATFORM_CONTROLLER_START_DELAY_SECONDS must be finite and non-negative")
    return robot_index * delay
