"""ASGI entry point for the MASAR platform.

Importing this module never starts a server.  FastAPI and Uvicorn remain
runtime dependencies and are imported lazily so configuration/unit tests can
run in a minimal Python environment.
"""

from __future__ import annotations

from typing import Any

from .config import get_settings


class _MissingFastAPI:
    """Small import-safe placeholder when optional ASGI dependencies are absent."""

    def __call__(self, *_args: Any, **_kwargs: Any) -> None:
        raise RuntimeError("FastAPI is required to serve the platform API")


def create_app() -> Any:
    """Create the FastAPI application without binding a socket or starting Uvicorn."""

    try:
        from .api import create_api_app
    except ImportError:
        return _MissingFastAPI()
    return create_api_app()


app = create_app()


def run() -> None:
    """Run Uvicorn when explicitly called (never as an import side effect)."""

    try:
        import uvicorn
    except ImportError as exc:
        raise RuntimeError("Uvicorn is required to run the platform API") from exc

    runtime_settings = get_settings()
    uvicorn.run(
        "masar_platform.main:app",
        host=runtime_settings.api_host,
        port=runtime_settings.api_port,
        factory=False,
    )


__all__ = ["app", "create_app", "run"]
