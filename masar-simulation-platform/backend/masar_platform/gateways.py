"""Preflight checks for the read-only browser ROS data gateways."""

from __future__ import annotations

from collections.abc import Callable
import socket

from .contracts import ScenarioManifest
from .profiles import get_topic_profile


def check_web_gateway_prerequisites(
    manifest: ScenarioManifest,
    *,
    ros1_port: int = 18_111,
    ros2_port: int = 18_112,
    probe: Callable[[int], bool] | None = None,
) -> tuple[bool, str]:
    """Require the rosbridge graphs used by the scenario's data profile."""

    profile = get_topic_profile(manifest.data_profile)
    required: list[tuple[str, int]] = []
    if any(any("/msg/" not in item for item in topic.message_types) for topic in profile.topics):
        required.append(("ros1", ros1_port))
    if any(any("/msg/" in item for item in topic.message_types) for topic in profile.topics):
        required.append(("ros2", ros2_port))
    check = probe or _tcp_ready
    unavailable = [graph for graph, port in required if not check(port)]
    if unavailable:
        return False, "rosbridge unavailable: " + ", ".join(unavailable)
    return True, "rosbridge ready: " + ", ".join(graph for graph, _port in required)


def _tcp_ready(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1.0):
            return True
    except OSError:
        return False


__all__ = ["check_web_gateway_prerequisites"]
