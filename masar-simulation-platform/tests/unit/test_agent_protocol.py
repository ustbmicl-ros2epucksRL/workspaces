from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from agents.runtime_agent.protocol import (
    AuthenticationError,
    RequestAuthenticator,
    derive_run_key,
    error_response,
    sign_request,
    signed_headers,
    success_response,
)


MASTER_SECRET = b"m" * 32
RUN_ID = "run-01"
TARGET = "/v1/runs/run-01/probe"


@pytest.mark.parametrize(
    "missing_header",
    ("X-Platform-Run", "X-Platform-Sequence", "X-Platform-Signature"),
)
def test_authenticator_rejects_each_missing_header(missing_header: str) -> None:
    authenticator = RequestAuthenticator(MASTER_SECRET)
    headers = dict(signed_headers(MASTER_SECRET, RUN_ID, 1, "GET", TARGET, b""))
    headers.pop(missing_header)

    with pytest.raises(AuthenticationError) as caught:
        authenticator.authenticate("GET", TARGET, b"", headers)

    assert caught.value.code == "AUTH_MISSING_HEADER"


def test_authenticator_accepts_a_valid_request_once() -> None:
    authenticator = RequestAuthenticator(MASTER_SECRET)
    headers = signed_headers(MASTER_SECRET, RUN_ID, 7, "GET", TARGET, b"")

    context = authenticator.authenticate("GET", TARGET, b"", headers)

    assert context.run_id == RUN_ID
    assert context.sequence == 7


def test_authenticator_rejects_stale_and_reordered_sequences() -> None:
    authenticator = RequestAuthenticator(MASTER_SECRET)
    authenticator.authenticate(
        "GET",
        TARGET,
        b"",
        signed_headers(MASTER_SECRET, RUN_ID, 7, "GET", TARGET, b""),
    )

    with pytest.raises(AuthenticationError) as stale:
        authenticator.authenticate(
            "GET",
            TARGET,
            b"",
            signed_headers(MASTER_SECRET, RUN_ID, 7, "GET", TARGET, b""),
        )
    with pytest.raises(AuthenticationError) as reordered:
        authenticator.authenticate(
            "GET",
            TARGET,
            b"",
            signed_headers(MASTER_SECRET, RUN_ID, 6, "GET", TARGET, b""),
        )

    assert stale.value.code == "AUTH_STALE_SEQUENCE"
    assert reordered.value.code == "AUTH_REORDERED_SEQUENCE"


def test_signature_covers_method_target_sequence_and_body() -> None:
    body = b'{"process_id":"step-1"}'
    signature = sign_request(MASTER_SECRET, RUN_ID, 3, "POST", TARGET, body)
    assert len(signature) == 64
    assert signature != sign_request(MASTER_SECRET, RUN_ID, 3, "GET", TARGET, body)
    assert signature != sign_request(MASTER_SECRET, RUN_ID, 3, "POST", TARGET + "?x=1", body)
    assert signature != sign_request(MASTER_SECRET, RUN_ID, 4, "POST", TARGET, body)
    assert signature != sign_request(MASTER_SECRET, RUN_ID, 3, "POST", TARGET, body + b" ")


def test_invalid_signature_does_not_consume_the_sequence() -> None:
    authenticator = RequestAuthenticator(MASTER_SECRET)
    headers = dict(signed_headers(MASTER_SECRET, RUN_ID, 1, "GET", TARGET, b""))
    headers["X-Platform-Signature"] = "0" * 64

    with pytest.raises(AuthenticationError) as caught:
        authenticator.authenticate("GET", TARGET, b"", headers)

    assert caught.value.code == "AUTH_INVALID_SIGNATURE"
    accepted = authenticator.authenticate(
        "GET",
        TARGET,
        b"",
        signed_headers(MASTER_SECRET, RUN_ID, 1, "GET", TARGET, b""),
    )
    assert accepted.sequence == 1


def test_run_keys_are_derived_per_run() -> None:
    assert derive_run_key(MASTER_SECRET, "run-a") != derive_run_key(MASTER_SECRET, "run-b")
    assert len(derive_run_key(MASTER_SECRET, "run-a")) == 32


def test_response_schema_never_contains_the_master_secret() -> None:
    success = success_response({"status": "ok"}, request_id="request-1")
    failure = error_response("INVALID", "request rejected", {"field": "argv"}, request_id="request-2")

    assert json.loads(success) == {
        "ok": True,
        "data": {"status": "ok"},
        "request_id": "request-1",
    }
    assert json.loads(failure) == {
        "ok": False,
        "error": {"code": "INVALID", "message": "request rejected", "details": {"field": "argv"}},
        "request_id": "request-2",
    }
    assert MASTER_SECRET not in success
    assert MASTER_SECRET not in failure


def test_master_secret_file_requires_exactly_32_bytes_and_mode_0600(tmp_path: Path) -> None:
    from masar_platform.agents import SecretFileError, load_master_secret

    path = tmp_path / "agent.secret"
    path.write_bytes(MASTER_SECRET)
    path.chmod(0o600)
    assert load_master_secret(path) == MASTER_SECRET

    path.chmod(0o640)
    with pytest.raises(SecretFileError, match="0600"):
        load_master_secret(path)

    path.chmod(0o600)
    path.write_bytes(b"short")
    with pytest.raises(SecretFileError, match="32 bytes"):
        load_master_secret(path)


def test_host_client_imports_with_only_the_backend_package_on_pythonpath(tmp_path: Path) -> None:
    project_root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(project_root / "backend")

    completed = subprocess.run(
        [sys.executable, "-c", "import masar_platform.agents"],
        cwd=tmp_path,
        env=environment,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr


def test_runtime_agent_sources_avoid_python_38_incompatible_with_syntax() -> None:
    project_root = Path(__file__).resolve().parents[2]
    for path in sorted((project_root / "agents" / "runtime_agent").glob("*.py")):
        source = path.read_text(encoding="utf-8")
        assert "with (\\n" not in source
