import sys
from types import SimpleNamespace
from unittest.mock import Mock

from masar_platform.main import run


def test_run_uses_configured_api_binding(monkeypatch) -> None:
    uvicorn_run = Mock()
    monkeypatch.setitem(sys.modules, "uvicorn", SimpleNamespace(run=uvicorn_run))
    monkeypatch.setenv("MASAR_PLATFORM_API_HOST", "0.0.0.0")
    monkeypatch.setenv("MASAR_PLATFORM_API_PORT", "19100")

    run()

    uvicorn_run.assert_called_once_with(
        "masar_platform.main:app",
        host="0.0.0.0",
        port=19_100,
        factory=False,
    )
