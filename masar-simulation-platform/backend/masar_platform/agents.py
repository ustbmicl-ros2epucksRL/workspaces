"""Signed host client for the loopback runtime Agents."""

from __future__ import annotations

from collections import defaultdict
import hashlib
import hmac
import http.client
import ipaddress
import json
from pathlib import Path
import stat
from threading import Lock
import time
from typing import Any, Mapping
from urllib.parse import urlencode


class SecretFileError(RuntimeError):
    pass


class AgentClientError(RuntimeError):
    pass


class AgentUnavailable(AgentClientError):
    pass


class AgentHTTPError(AgentClientError):
    def __init__(self, status: int, code: str, message: str, details: Mapping[str, Any] | None = None) -> None:
        self.status = status
        self.code = code
        self.details = dict(details or {})
        super().__init__(message)


def load_master_secret(path: str | Path) -> bytes:
    secret_path = Path(path)
    try:
        info = secret_path.stat()
    except FileNotFoundError as exc:
        raise SecretFileError("Agent secret file does not exist") from exc
    if not stat.S_ISREG(info.st_mode):
        raise SecretFileError("Agent secret path must be a regular file")
    if stat.S_IMODE(info.st_mode) != 0o600:
        raise SecretFileError("Agent secret file mode must be 0600")
    secret = secret_path.read_bytes()
    if len(secret) != 32:
        raise SecretFileError("Agent secret must contain exactly 32 bytes")
    return secret


class AgentClient:
    def __init__(
        self,
        host: str,
        port: int,
        master_secret: bytes,
        *,
        connect_timeout: float = 3.0,
        read_timeout: float = 10.0,
        retry_attempts: int = 2,
        retry_delay: float = 0.02,
    ) -> None:
        try:
            address = ipaddress.ip_address(host)
        except ValueError as exc:
            raise ValueError("Agent host must be a loopback IP address") from exc
        if not address.is_loopback:
            raise ValueError("Agent host must be a loopback IP address")
        if len(master_secret) != 32:
            raise ValueError("master_secret must contain exactly 32 bytes")
        if retry_attempts < 0:
            raise ValueError("retry_attempts must not be negative")
        self.host = host
        self.port = port
        self.master_secret = master_secret
        self.connect_timeout = connect_timeout
        self.read_timeout = read_timeout
        self.retry_attempts = retry_attempts
        self.retry_delay = retry_delay
        self._sequences: dict[str, int] = defaultdict(int)
        self._sequence_lock = Lock()

    def health(self) -> dict[str, Any]:
        return self._request("GET", "/healthz", None, None, retry=True, signed=False)

    def start(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        return self._request("POST", f"/v1/runs/{run_id}/start", run_id, payload, retry=False)

    def stop(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        return self._request("POST", f"/v1/runs/{run_id}/stop", run_id, payload, retry=False)

    def probe(self, run_id: str) -> dict[str, Any]:
        return self._request("GET", f"/v1/runs/{run_id}/probe", run_id, None, retry=True)

    def logs(
        self,
        run_id: str,
        process_id: str,
        *,
        stream: str = "stdout",
        after: int = 0,
        limit: int = 65536,
    ) -> dict[str, Any]:
        query = urlencode({"process_id": process_id, "stream": stream, "after": after, "limit": limit})
        return self._request("GET", f"/v1/runs/{run_id}/logs?{query}", run_id, None, retry=False)

    def _request(
        self,
        method: str,
        target: str,
        run_id: str | None,
        payload: Mapping[str, Any] | None,
        *,
        retry: bool,
        signed: bool = True,
    ) -> dict[str, Any]:
        attempts = self.retry_attempts + 1 if retry else 1
        last_error: AgentClientError | None = None
        for attempt in range(attempts):
            try:
                return self._request_once(method, target, run_id, payload, signed=signed)
            except (AgentUnavailable, AgentHTTPError) as exc:
                last_error = exc
                retryable = isinstance(exc, AgentUnavailable) or exc.status >= 500
                if not retryable or attempt + 1 >= attempts:
                    raise
                time.sleep(self.retry_delay)
        assert last_error is not None
        raise last_error

    def _request_once(
        self,
        method: str,
        target: str,
        run_id: str | None,
        payload: Mapping[str, Any] | None,
        *,
        signed: bool,
    ) -> dict[str, Any]:
        body = b"" if payload is None else json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
        headers = {"Content-Type": "application/json", "Accept": "application/json"}
        if signed:
            if run_id is None:
                raise ValueError("signed Agent requests require run_id")
            sequence = self._next_sequence(run_id)
            headers.update(_signed_headers(self.master_secret, run_id, sequence, method, target, body))
        connection = http.client.HTTPConnection(self.host, self.port, timeout=self.connect_timeout)
        try:
            connection.connect()
            if connection.sock is not None:
                connection.sock.settimeout(self.read_timeout)
            connection.request(method, target, body=body, headers=headers)
            response = connection.getresponse()
            response_body = response.read()
        except (OSError, http.client.HTTPException) as exc:
            raise AgentUnavailable("runtime Agent is unavailable") from exc
        finally:
            connection.close()
        try:
            decoded = json.loads(response_body)
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise AgentUnavailable("runtime Agent returned invalid JSON") from exc
        if not isinstance(decoded, dict):
            raise AgentUnavailable("runtime Agent returned an invalid response")
        if not 200 <= response.status < 300:
            error = decoded.get("error", {})
            raise AgentHTTPError(
                response.status,
                str(error.get("code", "AGENT_ERROR")),
                str(error.get("message", "runtime Agent rejected the request")),
                error.get("details", {}),
            )
        data = decoded.get("data")
        if not isinstance(data, dict):
            raise AgentUnavailable("runtime Agent response has no data object")
        return data

    def _next_sequence(self, run_id: str) -> int:
        with self._sequence_lock:
            self._sequences[run_id] += 1
            return self._sequences[run_id]


def _signed_headers(
    master_secret: bytes,
    run_id: str,
    sequence: int,
    method: str,
    target: str,
    body: bytes,
) -> dict[str, str]:
    run_key = hmac.new(
        master_secret,
        b"masar-platform-run\0" + run_id.encode("utf-8"),
        hashlib.sha256,
    ).digest()
    canonical = "\n".join((method.upper(), target, str(sequence), hashlib.sha256(body).hexdigest())).encode("utf-8")
    signature = hmac.new(run_key, canonical, hashlib.sha256).hexdigest()
    return {
        "X-Platform-Run": run_id,
        "X-Platform-Sequence": str(sequence),
        "X-Platform-Signature": signature,
    }


__all__ = [
    "AgentClient",
    "AgentClientError",
    "AgentHTTPError",
    "AgentUnavailable",
    "SecretFileError",
    "load_master_secret",
]
