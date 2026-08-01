"""Bounded, shell-free readiness probes executed inside a ROS container."""

from __future__ import annotations

import http.client
import ipaddress
import os
from pathlib import Path
import re
import socket
import subprocess
import time
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple
from urllib.parse import urlsplit


_TOPIC = re.compile(r"^/[A-Za-z0-9_./~-]+$")
_MESSAGE_TYPE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*/(?:msg/)?[A-Za-z][A-Za-z0-9_]*$")
_QOS_PROFILES = {
    "unknown",
    "system_default",
    "sensor_data",
    "services_default",
    "parameters",
    "parameter_events",
    "action_status_default",
}
_LOCAL_NO_PROXY = "127.0.0.1,localhost,::1"


class ProbeRequestError(ValueError):
    """Raised when a readiness request is not safe or well formed."""


class ProbeRunner:
    def __init__(self, agent_kind: str, workspace: Path) -> None:
        self.agent_kind = agent_kind
        self.workspace = Path(workspace).resolve()

    def run_many(
        self,
        probe_payloads: Sequence[Mapping[str, Any]],
        runtime: Mapping[str, Any],
    ) -> List[Dict[str, Any]]:
        normalized_runtime = _runtime_environment(runtime)
        if not isinstance(probe_payloads, (list, tuple)) or not probe_payloads:
            raise ProbeRequestError("probes must be a non-empty array")
        return [self.run(payload, normalized_runtime) for payload in probe_payloads]

    def run(self, payload: Mapping[str, Any], runtime_environment: Mapping[str, str]) -> Dict[str, Any]:
        probe = _normalize_probe(payload)
        if probe["type"] == "ros1_topic" and self.agent_kind != "ros1":
            raise ProbeRequestError("ROS1 topic probes require the ROS1 Agent")
        if probe["type"] == "ros2_topic" and self.agent_kind != "ros2":
            raise ProbeRequestError("ROS2 topic probes require the ROS2 Agent")

        started = time.monotonic()
        probe_type = probe["type"]
        if probe_type == "tcp":
            status, observed_type, detail = self._tcp(probe)
        elif probe_type == "http":
            status, observed_type, detail = self._http(probe)
        elif probe_type == "file":
            status, observed_type, detail = self._file(probe)
        elif probe_type == "ros1_topic":
            status, observed_type, detail = self._ros1_topic(probe, runtime_environment)
        else:
            status, observed_type, detail = self._ros2_topic(probe, runtime_environment)
        result = {
            "type": probe_type,
            "name": probe["name"],
            "required": probe["required"],
            "ready": status == "READY",
            "status": status,
            "message_type": probe["message_type"],
            "observed_type": observed_type,
            "latency_ms": round((time.monotonic() - started) * 1000, 3),
            "detail": detail,
        }
        return result

    def _tcp(self, probe: Mapping[str, Any]) -> Tuple[str, Optional[str], str]:
        host, port = _loopback_host_port(str(probe["name"]))
        try:
            connection = socket.create_connection((host, port), timeout=float(probe["max_age_seconds"]))
        except OSError:
            return "UNREACHABLE", None, "loopback TCP socket is not accepting connections"
        connection.close()
        return "READY", None, "loopback TCP socket accepted a connection"

    def _http(self, probe: Mapping[str, Any]) -> Tuple[str, Optional[str], str]:
        parsed = _loopback_http_url(str(probe["name"]))
        target = parsed.path or "/"
        if parsed.query:
            target += "?" + parsed.query
        connection = http.client.HTTPConnection(parsed.hostname, parsed.port, timeout=float(probe["max_age_seconds"]))
        try:
            connection.request("GET", target, headers={"Accept": "application/json"})
            response = connection.getresponse()
            response.read(4096)
        except (OSError, http.client.HTTPException):
            return "UNREACHABLE", None, "loopback HTTP endpoint is unavailable"
        finally:
            connection.close()
        if 200 <= response.status < 400:
            return "READY", None, "loopback HTTP endpoint returned a successful status"
        return "HTTP_ERROR", None, "loopback HTTP endpoint returned a non-success status"

    def _file(self, probe: Mapping[str, Any]) -> Tuple[str, Optional[str], str]:
        path = Path(str(probe["name"]))
        candidate = path.resolve() if path.is_absolute() else (self.workspace / path).resolve()
        try:
            candidate.relative_to(self.workspace)
        except ValueError as exc:
            raise ProbeRequestError("file probes must remain inside the configured workspace") from exc
        if candidate.is_file():
            return "READY", None, "workspace file exists"
        return "NOT_FOUND", None, "workspace file does not exist"

    def _ros1_topic(
        self,
        probe: Mapping[str, Any],
        runtime_environment: Mapping[str, str],
    ) -> Tuple[str, Optional[str], str]:
        observed = _command_output(
            ["rostopic", "type", str(probe["name"])],
            runtime_environment,
            float(probe["max_age_seconds"]),
        )
        if observed is None:
            return "NOT_FOUND", None, "ROS1 topic is absent from the graph"
        expected = str(probe["message_type"])
        if observed != expected:
            return "TYPE_MISMATCH", observed, "ROS1 topic type does not match the manifest"
        if not _message_observed(
            ["rostopic", "echo", "-n", "1", str(probe["name"])],
            runtime_environment,
            float(probe["max_age_seconds"]),
        ):
            return "STALE", observed, "ROS1 topic produced no fresh message within max_age_seconds"
        return "READY", observed, "ROS1 topic type and fresh message are valid"

    def _ros2_topic(
        self,
        probe: Mapping[str, Any],
        runtime_environment: Mapping[str, str],
    ) -> Tuple[str, Optional[str], str]:
        observed = _command_output(
            ["ros2", "topic", "type", str(probe["name"])],
            runtime_environment,
            float(probe["max_age_seconds"]),
        )
        if observed is None or observed.lower().startswith("unknown topic"):
            return "NOT_FOUND", None, "ROS2 topic is absent from the selected domain"
        expected = str(probe["message_type"])
        if observed != expected:
            return "TYPE_MISMATCH", observed, "ROS2 topic type does not match the manifest"
        command = [
            "ros2",
            "topic",
            "echo",
            str(probe["name"]),
            expected,
            "--once",
            "--qos-profile",
            str(probe["qos_profile"]),
        ]
        if not _message_observed(command, runtime_environment, float(probe["max_age_seconds"])):
            return "STALE", observed, "ROS2 topic produced no fresh message within max_age_seconds"
        return "READY", observed, "ROS2 topic type and fresh message are valid"


def _normalize_probe(payload: Mapping[str, Any]) -> Dict[str, Any]:
    if not isinstance(payload, Mapping):
        raise ProbeRequestError("each probe must be an object")
    allowed = {"type", "name", "message_type", "required", "max_age_seconds", "qos_profile"}
    unknown = set(payload) - allowed
    if unknown:
        raise ProbeRequestError("probe contains unknown fields: " + ", ".join(sorted(unknown)))
    probe_type = payload.get("type")
    if probe_type not in {"ros1_topic", "ros2_topic", "tcp", "http", "file"}:
        raise ProbeRequestError("unsupported probe type")
    name = payload.get("name")
    if not isinstance(name, str) or not name:
        raise ProbeRequestError("probe name must be a non-empty string")
    required = payload.get("required", True)
    if not isinstance(required, bool):
        raise ProbeRequestError("probe required must be boolean")
    max_age = payload.get("max_age_seconds", 1.0)
    if isinstance(max_age, bool) or not isinstance(max_age, (int, float)) or not 0 < max_age <= 30:
        raise ProbeRequestError("max_age_seconds must be greater than zero and at most 30")
    message_type = payload.get("message_type")
    if probe_type in {"ros1_topic", "ros2_topic"}:
        if not _TOPIC.fullmatch(name):
            raise ProbeRequestError("ROS topic names must be absolute and contain only safe characters")
        if not isinstance(message_type, str) or not _MESSAGE_TYPE.fullmatch(message_type):
            raise ProbeRequestError("ROS topic probes require a valid message_type")
    elif message_type is not None:
        raise ProbeRequestError("non-ROS probes do not accept message_type")
    qos_profile = payload.get("qos_profile", "system_default")
    if qos_profile not in _QOS_PROFILES:
        raise ProbeRequestError("unsupported qos_profile")
    if probe_type != "ros2_topic" and qos_profile != "system_default":
        raise ProbeRequestError("qos_profile is only configurable for ROS2 topic probes")
    return {
        "type": probe_type,
        "name": name,
        "message_type": message_type,
        "required": required,
        "max_age_seconds": float(max_age),
        "qos_profile": qos_profile,
    }


def _runtime_environment(payload: Mapping[str, Any]) -> Dict[str, str]:
    if not isinstance(payload, Mapping):
        raise ProbeRequestError("runtime must be an object")
    if set(payload) != {"ros1_master_uri", "ros2_domain_id"}:
        raise ProbeRequestError("runtime requires exactly ros1_master_uri and ros2_domain_id")
    parsed = _loopback_http_url(str(payload["ros1_master_uri"]))
    domain = payload["ros2_domain_id"]
    if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 232:
        raise ProbeRequestError("ros2_domain_id must be between 0 and 232")
    return {
        "ROS_MASTER_URI": parsed.geturl(),
        "ROS_DOMAIN_ID": str(domain),
        "NO_PROXY": _LOCAL_NO_PROXY,
        "no_proxy": _LOCAL_NO_PROXY,
    }


def runtime_environment(payload: Mapping[str, Any]) -> Dict[str, str]:
    """Validate and render the shared ROS runtime environment."""

    return _runtime_environment(payload)


def _loopback_host_port(value: str) -> Tuple[str, int]:
    parsed = urlsplit("//" + value)
    try:
        port = parsed.port
    except ValueError as exc:
        raise ProbeRequestError("TCP probe must use an explicit loopback IP and port") from exc
    host = _validated_loopback_host(parsed.hostname)
    if port is None or parsed.path:
        raise ProbeRequestError("TCP probe must use an explicit loopback IP and port")
    return host, port


def _loopback_http_url(value: str):
    parsed = urlsplit(value)
    if parsed.scheme != "http" or parsed.username or parsed.password or parsed.fragment:
        raise ProbeRequestError("HTTP probe URL must be an unauthenticated loopback HTTP URL")
    try:
        port = parsed.port
    except ValueError as exc:
        raise ProbeRequestError("HTTP probe URL must use an explicit loopback IP and port") from exc
    _validated_loopback_host(parsed.hostname)
    if port is None:
        raise ProbeRequestError("HTTP probe URL must use an explicit loopback IP and port")
    return parsed


def _validated_loopback_host(hostname: Optional[str]) -> str:
    if hostname == "localhost":
        return hostname
    try:
        address = ipaddress.ip_address(hostname or "")
    except ValueError as exc:
        raise ProbeRequestError("probe host must be a literal loopback address or localhost") from exc
    if not address.is_loopback:
        raise ProbeRequestError("probe host must be a literal loopback address or localhost")
    return str(address)


def _command_output(argv: List[str], environment: Mapping[str, str], timeout: float) -> Optional[str]:
    process_environment = os.environ.copy()
    process_environment.update(environment)
    try:
        completed = subprocess.run(
            argv,
            env=process_environment,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            check=False,
            universal_newlines=True,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    output = completed.stdout.strip()
    return output if completed.returncode == 0 and output else None


def _message_observed(argv: List[str], environment: Mapping[str, str], timeout: float) -> bool:
    process_environment = os.environ.copy()
    process_environment.update(environment)
    try:
        completed = subprocess.run(
            argv,
            env=process_environment,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return completed.returncode == 0


__all__ = ["ProbeRequestError", "ProbeRunner", "runtime_environment"]
