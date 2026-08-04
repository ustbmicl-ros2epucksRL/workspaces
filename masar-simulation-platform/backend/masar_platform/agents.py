"""Signed host client for the loopback runtime Agents."""

from __future__ import annotations

from collections import defaultdict
from contextlib import nullcontext
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
        self._signed_request_lock = Lock()

    def health(self) -> dict[str, Any]:
        return self._request("GET", "/healthz", None, None, retry=True, signed=False)

    def start(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        return self._request("POST", f"/v1/runs/{run_id}/start", run_id, payload, retry=False)

    def stop(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        grace = payload.get("grace_seconds", {})
        grace_total = (
            sum(float(grace.get(name, 0.0)) for name in ("sigint", "sigterm", "sigkill"))
            if isinstance(grace, Mapping)
            else 0.0
        )
        return self._request(
            "POST",
            f"/v1/runs/{run_id}/stop",
            run_id,
            payload,
            retry=False,
            read_timeout=max(self.read_timeout, grace_total + 5.0),
        )

    def probe(self, run_id: str) -> dict[str, Any]:
        return self._request("GET", f"/v1/runs/{run_id}/probe", run_id, None, retry=True)

    def probe_readiness(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Run an idempotent set of exact readiness probes inside the Agent."""

        return self._request("POST", f"/v1/runs/{run_id}/probe", run_id, payload, retry=True)

    def command(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        return self._request("POST", f"/v1/runs/{run_id}/command", run_id, payload, retry=False)

    def graph_snapshot(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        return self._request("POST", f"/v1/runs/{run_id}/graph", run_id, payload, retry=True)

    def webots_preflight(self, run_id: str, *, require_bridge: bool) -> dict[str, Any]:
        return self._request(
            "POST",
            f"/v1/runs/{run_id}/webots-preflight",
            run_id,
            {"require_bridge": require_bridge},
            retry=True,
            read_timeout=20.0,
        )

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
        read_timeout: float | None = None,
    ) -> dict[str, Any]:
        request_context = self._signed_request_lock if signed else nullcontext()
        with request_context:
            retries_remaining = self.retry_attempts if retry else 0
            sequence_recovery_available = signed
            while True:
                try:
                    return self._request_once(
                        method,
                        target,
                        run_id,
                        payload,
                        signed=signed,
                        read_timeout=read_timeout,
                    )
                except (AgentUnavailable, AgentHTTPError) as exc:
                    sequence_rejected = (
                        isinstance(exc, AgentHTTPError)
                        and exc.code in {"AUTH_STALE_SEQUENCE", "AUTH_REORDERED_SEQUENCE"}
                    )
                    if sequence_recovery_available and sequence_rejected and run_id is not None:
                        self._reseed_sequence(run_id)
                        sequence_recovery_available = False
                        continue
                    retryable = isinstance(exc, AgentUnavailable) or exc.status >= 500
                    if not retryable or retries_remaining <= 0:
                        raise
                    retries_remaining -= 1
                    time.sleep(self.retry_delay)

    def _request_once(
        self,
        method: str,
        target: str,
        run_id: str | None,
        payload: Mapping[str, Any] | None,
        *,
        signed: bool,
        read_timeout: float | None,
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
                connection.sock.settimeout(self.read_timeout if read_timeout is None else read_timeout)
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

    def _reseed_sequence(self, run_id: str) -> None:
        with self._sequence_lock:
            self._sequences[run_id] = max(self._sequences[run_id], time.time_ns())


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
