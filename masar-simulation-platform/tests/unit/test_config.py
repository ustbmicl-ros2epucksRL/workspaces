from pathlib import Path

import pytest

from masar_platform.config import Settings, get_settings


def _clear_platform_env(monkeypatch: pytest.MonkeyPatch) -> None:
    for name in (
        "EXTERNAL_HOST",
        "EXTERNAL_PORT",
        "API_HOST",
        "API_PORT",
        "ROS1_AGENT_HOST",
        "ROS1_AGENT_PORT",
        "ROS2_AGENT_HOST",
        "ROS2_AGENT_PORT",
        "ROS1_ROSBRIDGE_PORT",
        "ROS2_ROSBRIDGE_PORT",
        "ROS1_VIDEO_PORT",
        "ROS2_VIDEO_PORT",
        "ROS1_NOVNC_PORT",
        "ROS2_NOVNC_PORT",
        "WEBOTS_STREAM_PORT",
        "DATA_DIR",
        "SECRET_PATH",
        "EVENT_RETENTION_DAYS",
        "EVENT_RETENTION_COUNT",
    ):
        monkeypatch.delenv(f"MASAR_PLATFORM_{name}", raising=False)


def test_default_loopback_ports_and_paths(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_platform_env(monkeypatch)

    settings = get_settings()
    expected_ports = {
        "external_port": 18090,
        "api_port": 18100,
        "ros1_agent_port": 18101,
        "ros2_agent_port": 18102,
        "ros1_rosbridge_port": 18111,
        "ros2_rosbridge_port": 18112,
        "ros1_video_port": 18121,
        "ros2_video_port": 18122,
        "ros1_novnc_port": 18131,
        "ros2_novnc_port": 18132,
        "webots_stream_port": 18141,
    }

    assert settings.external_host == "127.0.0.1"
    assert settings.api_host == "127.0.0.1"
    assert settings.ros1_agent_host == "127.0.0.1"
    assert settings.ros2_agent_host == "127.0.0.1"
    for field, value in expected_ports.items():
        assert getattr(settings, field) == value

    project_root = Path(__file__).resolve().parents[2]
    assert Path(settings.data_dir).resolve() == project_root / "runs"
    assert settings.secret_path == Path("/run/masar-platform/agent.secret")
    assert settings.event_retention_days == 30
    assert settings.event_retention_count == 10_000


def test_environment_values_override_ports_and_paths(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    _clear_platform_env(monkeypatch)
    monkeypatch.setenv("MASAR_PLATFORM_EXTERNAL_HOST", "0.0.0.0")
    monkeypatch.setenv("MASAR_PLATFORM_EXTERNAL_PORT", "19090")
    monkeypatch.setenv("MASAR_PLATFORM_API_PORT", "19100")
    monkeypatch.setenv("MASAR_PLATFORM_ROS1_AGENT_PORT", "19101")
    monkeypatch.setenv("MASAR_PLATFORM_ROS2_AGENT_PORT", "19102")
    monkeypatch.setenv("MASAR_PLATFORM_ROS1_ROSBRIDGE_PORT", "19111")
    monkeypatch.setenv("MASAR_PLATFORM_ROS2_ROSBRIDGE_PORT", "19112")
    monkeypatch.setenv("MASAR_PLATFORM_ROS1_VIDEO_PORT", "19121")
    monkeypatch.setenv("MASAR_PLATFORM_ROS2_VIDEO_PORT", "19122")
    monkeypatch.setenv("MASAR_PLATFORM_ROS1_NOVNC_PORT", "19131")
    monkeypatch.setenv("MASAR_PLATFORM_ROS2_NOVNC_PORT", "19132")
    monkeypatch.setenv("MASAR_PLATFORM_WEBOTS_STREAM_PORT", "19141")
    monkeypatch.setenv("MASAR_PLATFORM_DATA_DIR", str(tmp_path / "runs"))
    monkeypatch.setenv("MASAR_PLATFORM_SECRET_PATH", str(tmp_path / "agent.secret"))
    monkeypatch.setenv("MASAR_PLATFORM_EVENT_RETENTION_DAYS", "7")
    monkeypatch.setenv("MASAR_PLATFORM_EVENT_RETENTION_COUNT", "123")

    settings = get_settings()

    assert settings.external_host == "0.0.0.0"
    assert settings.external_port == 19090
    assert settings.api_port == 19100
    assert settings.ros1_agent_port == 19101
    assert settings.ros2_agent_port == 19102
    assert settings.ros1_rosbridge_port == 19111
    assert settings.ros2_rosbridge_port == 19112
    assert settings.ros1_video_port == 19121
    assert settings.ros2_video_port == 19122
    assert settings.ros1_novnc_port == 19131
    assert settings.ros2_novnc_port == 19132
    assert settings.webots_stream_port == 19141
    assert settings.data_dir == tmp_path / "runs"
    assert settings.secret_path == tmp_path / "agent.secret"
    assert settings.event_retention_days == 7
    assert settings.event_retention_count == 123


def test_settings_is_frozen(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_platform_env(monkeypatch)
    settings = Settings()

    with pytest.raises((AttributeError, TypeError)):
        settings.external_port = 19090  # type: ignore[misc]


def test_invalid_positive_values_are_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_platform_env(monkeypatch)
    monkeypatch.setenv("MASAR_PLATFORM_API_PORT", "0")

    with pytest.raises(ValueError, match="api_port"):
        get_settings()
