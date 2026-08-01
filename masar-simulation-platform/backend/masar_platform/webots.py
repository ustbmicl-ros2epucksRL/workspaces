"""Strict Webots stream metadata shared by orchestration and the web UI."""

from __future__ import annotations

from dataclasses import dataclass


_LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})


@dataclass(frozen=True, slots=True)
class WebotsStreamConfig:
    """Describe the private Webots endpoint and its same-origin browser path."""

    bind_host: str = "127.0.0.1"
    port: int = 18_141
    public_path: str = "/webots/"

    def __post_init__(self) -> None:
        if self.bind_host not in _LOOPBACK_HOSTS:
            raise ValueError("Webots stream must bind to a loopback host")
        if isinstance(self.port, bool) or not isinstance(self.port, int) or not 1 <= self.port <= 65_535:
            raise ValueError("Webots stream port must be between 1 and 65535")
        if (
            not self.public_path.startswith("/")
            or not self.public_path.endswith("/")
            or ".." in self.public_path
            or "?" in self.public_path
            or "#" in self.public_path
        ):
            raise ValueError("public_path must be an absolute, traversal-free path ending in /")

    @property
    def readiness_target(self) -> str:
        return f"{self.bind_host}:{self.port}"

    def launch_environment(self) -> dict[str, str]:
        """Return the explicit opt-in consumed by ``start_robosar_webots.sh``."""

        return {"WEBOTS_STREAM": "true", "WEBOTS_PORT": str(self.port)}


__all__ = ["WebotsStreamConfig"]
