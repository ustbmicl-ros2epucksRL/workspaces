"""Configuration for the MASAR simulation platform control plane.

The module only parses values and constructs immutable objects.  It does not
create directories, open sockets, or otherwise change process state at import
time.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping


_LOOPBACK = "127.0.0.1"
_PLATFORM_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_SECRET_PATH = Path("/run/masar-platform/agent.secret")
_DEFAULT_SCENARIO_DIR = _PLATFORM_ROOT / "scenarios"


def _default_data_dir() -> Path:
    """Use the checkout's runs directory, or a writable user path when installed."""

    if (_PLATFORM_ROOT / "pyproject.toml").is_file():
        return _PLATFORM_ROOT / "runs"
    state_home = os.environ.get("XDG_STATE_HOME")
    base = Path(state_home).expanduser() if state_home and state_home.strip() else Path.home() / ".local" / "state"
    return base / "masar-platform" / "runs"


_DEFAULT_DATA_DIR = _default_data_dir()


def _env(environ: Mapping[str, str], name: str, default: str) -> str:
    value = environ.get(f"MASAR_PLATFORM_{name}")
    return default if value is None else value


def _positive_int(name: str, value: str | int, *, port: bool = False) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be an integer") from exc
    upper_bound = 65_535 if port else None
    if parsed <= 0 or (upper_bound is not None and parsed > upper_bound):
        suffix = " between 1 and 65535" if port else " greater than zero"
        raise ValueError(f"{name} must be{suffix}")
    return parsed


def _boolean(name: str, value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be a boolean")


def _host(name: str, value: str) -> str:
    host = str(value).strip()
    if not host:
        raise ValueError(f"{name} must not be empty")
    return host


def _path(name: str, value: str | Path) -> Path:
    raw_value = str(value)
    if not raw_value.strip():
        raise ValueError(f"{name} must not be empty")
    return Path(raw_value).expanduser()


@dataclass(frozen=True, slots=True)
class Settings:
    """Immutable runtime settings with loopback-only defaults."""

    external_host: str = _LOOPBACK
    external_port: int = 18_090
    api_host: str = _LOOPBACK
    api_port: int = 18_100
    ros1_agent_host: str = _LOOPBACK
    ros1_agent_port: int = 18_101
    ros2_agent_host: str = _LOOPBACK
    ros2_agent_port: int = 18_102
    ros1_rosbridge_port: int = 18_111
    ros2_rosbridge_port: int = 18_112
    ros1_video_port: int = 18_121
    ros2_video_port: int = 18_122
    ros1_novnc_port: int = 18_131
    ros2_novnc_port: int = 18_132
    webots_stream_port: int = 18_141
    data_dir: Path = field(default_factory=lambda: _DEFAULT_DATA_DIR)
    secret_path: Path = _DEFAULT_SECRET_PATH
    scenario_dir: Path = _DEFAULT_SCENARIO_DIR
    visual_stack_ready: bool = False
    event_retention_days: int = 30
    event_retention_count: int = 10_000

    def __post_init__(self) -> None:
        for field_name in (
            "external_port",
            "api_port",
            "ros1_agent_port",
            "ros2_agent_port",
            "ros1_rosbridge_port",
            "ros2_rosbridge_port",
            "ros1_video_port",
            "ros2_video_port",
            "ros1_novnc_port",
            "ros2_novnc_port",
            "webots_stream_port",
        ):
            object.__setattr__(
                self,
                field_name,
                _positive_int(field_name, getattr(self, field_name), port=True),
            )
        for field_name in ("event_retention_days", "event_retention_count"):
            object.__setattr__(
                self,
                field_name,
                _positive_int(field_name, getattr(self, field_name)),
            )
        for field_name in (
            "external_host",
            "api_host",
            "ros1_agent_host",
            "ros2_agent_host",
        ):
            object.__setattr__(self, field_name, _host(field_name, getattr(self, field_name)))
        object.__setattr__(self, "data_dir", _path("data_dir", self.data_dir))
        object.__setattr__(self, "secret_path", _path("secret_path", self.secret_path))
        object.__setattr__(self, "scenario_dir", _path("scenario_dir", self.scenario_dir))
        object.__setattr__(self, "visual_stack_ready", _boolean("visual_stack_ready", self.visual_stack_ready))

    @classmethod
    def from_env(cls, environ: Mapping[str, str] | None = None) -> "Settings":
        """Build settings from ``MASAR_PLATFORM_*`` environment variables."""

        values = os.environ if environ is None else environ
        return cls(
            external_host=_env(values, "EXTERNAL_HOST", _LOOPBACK),
            external_port=_positive_int("external_port", _env(values, "EXTERNAL_PORT", "18090"), port=True),
            api_host=_env(values, "API_HOST", _LOOPBACK),
            api_port=_positive_int("api_port", _env(values, "API_PORT", "18100"), port=True),
            ros1_agent_host=_env(values, "ROS1_AGENT_HOST", _LOOPBACK),
            ros1_agent_port=_positive_int("ros1_agent_port", _env(values, "ROS1_AGENT_PORT", "18101"), port=True),
            ros2_agent_host=_env(values, "ROS2_AGENT_HOST", _LOOPBACK),
            ros2_agent_port=_positive_int("ros2_agent_port", _env(values, "ROS2_AGENT_PORT", "18102"), port=True),
            ros1_rosbridge_port=_positive_int(
                "ros1_rosbridge_port", _env(values, "ROS1_ROSBRIDGE_PORT", "18111"), port=True
            ),
            ros2_rosbridge_port=_positive_int(
                "ros2_rosbridge_port", _env(values, "ROS2_ROSBRIDGE_PORT", "18112"), port=True
            ),
            ros1_video_port=_positive_int("ros1_video_port", _env(values, "ROS1_VIDEO_PORT", "18121"), port=True),
            ros2_video_port=_positive_int("ros2_video_port", _env(values, "ROS2_VIDEO_PORT", "18122"), port=True),
            ros1_novnc_port=_positive_int("ros1_novnc_port", _env(values, "ROS1_NOVNC_PORT", "18131"), port=True),
            ros2_novnc_port=_positive_int("ros2_novnc_port", _env(values, "ROS2_NOVNC_PORT", "18132"), port=True),
            webots_stream_port=_positive_int(
                "webots_stream_port", _env(values, "WEBOTS_STREAM_PORT", "18141"), port=True
            ),
            data_dir=_path("data_dir", _env(values, "DATA_DIR", str(_DEFAULT_DATA_DIR))),
            secret_path=_path("secret_path", _env(values, "SECRET_PATH", str(_DEFAULT_SECRET_PATH))),
            scenario_dir=_path("scenario_dir", _env(values, "SCENARIO_DIR", str(_DEFAULT_SCENARIO_DIR))),
            visual_stack_ready=_boolean("visual_stack_ready", _env(values, "VISUAL_STACK_READY", "false")),
            event_retention_days=_positive_int(
                "event_retention_days", _env(values, "EVENT_RETENTION_DAYS", "30")
            ),
            event_retention_count=_positive_int(
                "event_retention_count", _env(values, "EVENT_RETENTION_COUNT", "10000")
            ),
        )


def get_settings(environ: Mapping[str, str] | None = None) -> Settings:
    """Return a fresh immutable settings snapshot.

    Passing an explicit mapping makes configuration deterministic in tests and
    allows callers to avoid mutating process-wide environment state.
    """

    return Settings.from_env(environ)


settings = get_settings()


__all__ = ["Settings", "get_settings", "settings"]
