from pathlib import Path

import pytest

from masar_platform.webots import WebotsStreamConfig


ROOT = Path(__file__).resolve().parents[2]


def test_platform_webots_endpoint_is_loopback_and_default_off_compatible() -> None:
    stream = WebotsStreamConfig(port=18_141)

    assert stream.bind_host == "127.0.0.1"
    assert stream.public_path == "/webots/"
    assert stream.launch_environment() == {
        "WEBOTS_STREAM": "true",
        "WEBOTS_PORT": "18141",
    }
    assert stream.readiness_target == "127.0.0.1:18141"


def test_platform_webots_endpoint_rejects_public_bind_and_invalid_path() -> None:
    with pytest.raises(ValueError, match="loopback"):
        WebotsStreamConfig(bind_host="0.0.0.0")
    with pytest.raises(ValueError, match="public_path"):
        WebotsStreamConfig(public_path="webots")


def test_caddy_webots_novnc_route_preserves_websocket_upgrade() -> None:
    caddyfile = (ROOT / "deploy/caddy/Caddyfile").read_text(encoding="utf-8")

    assert "handle_path /display/webots/*" in caddyfile
    assert "reverse_proxy 127.0.0.1:18133" in caddyfile
    assert "reverse_proxy 127.0.0.1:18141" not in caddyfile
    assert "header_up Upgrade {http.request.header.Upgrade}" in caddyfile
    assert "header_up Connection {http.request.header.Connection}" in caddyfile


def test_webots_guard_rejects_non_loopback_ingress_on_the_native_stream_port() -> None:
    guard = (ROOT / "deploy" / "webots-guard" / "webots-port-guard").read_text(
        encoding="utf-8"
    )
    dockerfile = (ROOT / "deploy" / "webots-guard" / "Dockerfile").read_text(
        encoding="utf-8"
    )

    assert "INPUT ! -i lo -p tcp" in guard
    assert '--dport "$port"' in guard
    assert "-j REJECT --reject-with tcp-reset" in guard
    assert "iptables" in guard and "ip6tables" in guard
    assert "--healthcheck" in guard
    assert "iptables=1.8.11-r1" in dockerfile
