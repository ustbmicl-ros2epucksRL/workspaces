"""HMAC authentication and JSON response helpers for the runtime Agent."""

from __future__ import annotations

import hashlib
import hmac
import json
import threading
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional


PROTOCOL_VERSION = 1
RUN_HEADER = "X-Platform-Run"
SEQUENCE_HEADER = "X-Platform-Sequence"
SIGNATURE_HEADER = "X-Platform-Signature"


class AuthenticationError(ValueError):
    """A signed request failed validation."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class AuthenticationContext:
    run_id: str
    sequence: int


def derive_run_key(master_secret: bytes, run_id: str) -> bytes:
    if not isinstance(master_secret, bytes) or len(master_secret) != 32:
        raise ValueError("master_secret must contain exactly 32 bytes")
    if not isinstance(run_id, str) or not run_id:
        raise ValueError("run_id must be a non-empty string")
    return hmac.new(master_secret, b"masar-platform-run\0" + run_id.encode("utf-8"), hashlib.sha256).digest()


def sign_request(
    master_secret: bytes,
    run_id: str,
    sequence: int,
    method: str,
    target: str,
    body: bytes,
) -> str:
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= 0:
        raise ValueError("sequence must be a positive integer")
    if not isinstance(body, bytes):
        raise TypeError("body must be bytes")
    canonical = "\n".join(
        (
            method.upper(),
            target,
            str(sequence),
            hashlib.sha256(body).hexdigest(),
        )
    ).encode("utf-8")
    return hmac.new(derive_run_key(master_secret, run_id), canonical, hashlib.sha256).hexdigest()


def signed_headers(
    master_secret: bytes,
    run_id: str,
    sequence: int,
    method: str,
    target: str,
    body: bytes,
) -> Dict[str, str]:
    return {
        RUN_HEADER: run_id,
        SEQUENCE_HEADER: str(sequence),
        SIGNATURE_HEADER: sign_request(master_secret, run_id, sequence, method, target, body),
    }


class RequestAuthenticator:
    """Validate signatures and enforce a strictly increasing sequence per run."""

    def __init__(self, master_secret: bytes) -> None:
        derive_run_key(master_secret, "validation")
        self._master_secret = master_secret
        self._last_sequences: Dict[str, int] = {}
        self._lock = threading.Lock()

    def authenticate(
        self,
        method: str,
        target: str,
        body: bytes,
        headers: Mapping[str, str],
    ) -> AuthenticationContext:
        normalized = {str(name).lower(): str(value) for name, value in headers.items()}
        required = (RUN_HEADER, SEQUENCE_HEADER, SIGNATURE_HEADER)
        missing = [name for name in required if name.lower() not in normalized]
        if missing:
            raise AuthenticationError("AUTH_MISSING_HEADER", "missing signed request header")

        run_id = normalized[RUN_HEADER.lower()]
        sequence_text = normalized[SEQUENCE_HEADER.lower()]
        supplied_signature = normalized[SIGNATURE_HEADER.lower()]
        try:
            sequence = int(sequence_text)
        except ValueError as exc:
            raise AuthenticationError("AUTH_INVALID_SEQUENCE", "sequence must be a positive integer") from exc
        if sequence <= 0 or str(sequence) != sequence_text:
            raise AuthenticationError("AUTH_INVALID_SEQUENCE", "sequence must be a positive integer")

        try:
            expected = sign_request(self._master_secret, run_id, sequence, method, target, body)
        except (TypeError, ValueError) as exc:
            raise AuthenticationError("AUTH_INVALID_REQUEST", str(exc)) from exc
        if not hmac.compare_digest(supplied_signature, expected):
            raise AuthenticationError("AUTH_INVALID_SIGNATURE", "request signature is invalid")

        with self._lock:
            last = self._last_sequences.get(run_id, 0)
            if sequence == last:
                raise AuthenticationError("AUTH_STALE_SEQUENCE", "request sequence was already used")
            if sequence < last:
                raise AuthenticationError(
                    "AUTH_REORDERED_SEQUENCE", "request sequence is older than the accepted sequence"
                )
            self._last_sequences[run_id] = sequence
        return AuthenticationContext(run_id=run_id, sequence=sequence)


def success_response(data: Mapping[str, Any], request_id: Optional[str] = None) -> bytes:
    payload: Dict[str, Any] = {"ok": True, "data": dict(data)}
    if request_id is not None:
        payload["request_id"] = request_id
    return _json_bytes(payload)


def error_response(
    code: str,
    message: str,
    details: Optional[Mapping[str, Any]] = None,
    request_id: Optional[str] = None,
) -> bytes:
    payload: Dict[str, Any] = {
        "ok": False,
        "error": {"code": code, "message": message, "details": dict(details or {})},
    }
    if request_id is not None:
        payload["request_id"] = request_id
    return _json_bytes(payload)


def _json_bytes(payload: Mapping[str, Any]) -> bytes:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")


__all__ = [
    "AuthenticationContext",
    "AuthenticationError",
    "PROTOCOL_VERSION",
    "RequestAuthenticator",
    "derive_run_key",
    "error_response",
    "sign_request",
    "signed_headers",
    "success_response",
]
