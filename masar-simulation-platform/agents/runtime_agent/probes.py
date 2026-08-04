"""Bounded, shell-free readiness probes executed inside a ROS container."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
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
WEBOTS_DISPLAY = ":103"
VIRTUALGL_DISPLAY = ":0"
ROS1_BRIDGE_SETUP = Path("/root/ros-humble-ros1-bridge/install/setup.bash")
ROS1_BRIDGE_EXECUTABLE = Path(
    "/root/ros-humble-ros1-bridge/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge"
)


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
        normalized_probes = [_normalize_probe(payload) for payload in probe_payloads]
        ros2_probes = [probe for probe in normalized_probes if probe["type"] == "ros2_topic"]
        if ros2_probes:
            if self.agent_kind != "ros2":
                raise ProbeRequestError("ROS2 topic probes require the ROS2 Agent")
            topic_types, received_topics, latency_ms = _observe_ros2_topics(
                ros2_probes,
                int(normalized_runtime["ROS_DOMAIN_ID"]),
            )
            ros2_results = {
                id(probe): _ros2_probe_result(probe, topic_types, received_topics, latency_ms)
                for probe in ros2_probes
            }
        else:
            ros2_results = {}

        non_ros2 = [probe for probe in normalized_probes if probe["type"] != "ros2_topic"]
        if not non_ros2:
            return [ros2_results[id(probe)] for probe in normalized_probes]
        with ThreadPoolExecutor(max_workers=min(len(probe_payloads), 8)) as executor:
            futures = {
                id(probe): executor.submit(self.run, probe, normalized_runtime)
                for probe in non_ros2
            }
            return [
                ros2_results[id(probe)]
                if probe["type"] == "ros2_topic"
                else futures[id(probe)].result()
                for probe in normalized_probes
            ]

    def graph_snapshot(self, runtime: Mapping[str, Any]) -> Dict[str, Any]:
        environment = _runtime_environment(runtime)
        if self.agent_kind == "ros1":
            node_command = ["rosnode", "list"]
            topic_command = ["rostopic", "list", "-v"]
        elif self.agent_kind == "ros2":
            node_command = ["ros2", "node", "list"]
            topic_command = ["ros2", "topic", "list", "-t"]
        else:
            raise ProbeRequestError("graph snapshots require a ROS1 or ROS2 Agent")
        nodes_output = _graph_command(node_command, environment)
        topics_output = _graph_command(topic_command, environment)
        nodes_available = nodes_output is not None
        topics_available = topics_output is not None
        return {
            "agent": self.agent_kind,
            "available": nodes_available and topics_available,
            "nodes_available": nodes_available,
            "topics_available": topics_available,
            "nodes": _nonempty_lines(nodes_output),
            "topics": _topic_snapshot(self.agent_kind, topics_output),
        }

    def webots_preflight(self, *, require_bridge: bool) -> Dict[str, Any]:
        """Verify the one supported Webots GPU/display path before a Run starts."""

        if self.agent_kind != "ros2":
            raise ProbeRequestError("Webots platform preflight requires the ROS2 Agent")
        renderer = _webots_renderer()
        if "nvidia" not in renderer.lower() or "llvmpipe" in renderer.lower():
            raise ProbeRequestError(
                f"Webots requires an NVIDIA VirtualGL renderer; observed: {renderer or 'unknown'}"
            )
        bridge_setup: str | None = None
        bridge_executable: str | None = None
        if require_bridge:
            if not ROS1_BRIDGE_SETUP.is_file():
                raise ProbeRequestError(f"required ros1_bridge setup is missing: {ROS1_BRIDGE_SETUP}")
            if not ROS1_BRIDGE_EXECUTABLE.is_file() or not os.access(ROS1_BRIDGE_EXECUTABLE, os.X_OK):
                raise ProbeRequestError(f"required ros1_bridge executable is missing: {ROS1_BRIDGE_EXECUTABLE}")
            bridge_setup = str(ROS1_BRIDGE_SETUP)
            bridge_executable = str(ROS1_BRIDGE_EXECUTABLE)
        return {
            "ready": True,
            "display": WEBOTS_DISPLAY,
            "renderer": renderer,
            "bridge_setup": bridge_setup,
            "bridge_executable": bridge_executable,
        }

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
        topic_types, received_topics, _latency_ms = _observe_ros2_topics(
            [probe],
            int(runtime_environment["ROS_DOMAIN_ID"]),
        )
        result = _ros2_probe_result(probe, topic_types, received_topics, 0.0)
        return result["status"], result["observed_type"], result["detail"]


def _observe_ros2_topics(
    probes: Sequence[Mapping[str, Any]],
    domain_id: int,
) -> Tuple[Dict[str, Tuple[str, ...]], set[str], float]:
    """Observe one batch of ROS2 topics without spawning one CLI process per topic."""

    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy import qos
        from rclpy.signals import SignalHandlerOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise ProbeRequestError("ROS2 readiness probes require rclpy") from exc

    context = Context()
    node = None
    executor = None
    received_topics: set[str] = set()
    observed_types: Dict[str, Tuple[str, ...]] = {}
    started = time.monotonic()
    try:
        rclpy.init(
            context=context,
            domain_id=domain_id,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        node = rclpy.create_node(
            f"masar_readiness_{os.getpid()}_{time.monotonic_ns()}",
            context=context,
            enable_rosout=False,
            start_parameter_services=False,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        for probe in probes:
            message_type = str(probe["message_type"])
            type_parts = message_type.split("/")
            import_type = message_type if len(type_parts) == 3 else f"{type_parts[0]}/msg/{type_parts[1]}"
            try:
                message_class = get_message(import_type)
                qos_profile = getattr(qos, f"qos_profile_{probe['qos_profile']}")
            except (AttributeError, LookupError, ValueError) as exc:
                raise ProbeRequestError(
                    f"ROS2 readiness cannot subscribe to {probe['name']} as {message_type}"
                ) from exc
            topic_name = str(probe["name"])
            node.create_subscription(
                message_class,
                topic_name,
                lambda _message, name=topic_name: received_topics.add(name),
                qos_profile,
            )

        deadline = started + max(float(probe["max_age_seconds"]) for probe in probes)
        expected_names = {str(probe["name"]) for probe in probes}
        while time.monotonic() < deadline and received_topics != expected_names:
            remaining = deadline - time.monotonic()
            executor.spin_once(timeout_sec=min(0.1, max(0.0, remaining)))
            for topic_name in expected_names:
                publisher_types = {
                    endpoint.topic_type
                    for endpoint in node.get_publishers_info_by_topic(topic_name)
                }
                if publisher_types:
                    observed_types[topic_name] = tuple(sorted(publisher_types))
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()

    latency_ms = round((time.monotonic() - started) * 1000, 3)
    return observed_types, received_topics, latency_ms


def _ros2_probe_result(
    probe: Mapping[str, Any],
    topic_types: Mapping[str, Sequence[str]],
    received_topics: set[str],
    latency_ms: float,
) -> Dict[str, Any]:
    topic_name = str(probe["name"])
    expected = str(probe["message_type"])
    available_types = tuple(topic_types.get(topic_name, ()))
    observed_type = expected if expected in available_types else (available_types[0] if available_types else None)
    if not available_types:
        status = "NOT_FOUND"
        detail = "ROS2 topic is absent from the selected domain"
    elif expected not in available_types:
        status = "TYPE_MISMATCH"
        detail = "ROS2 topic type does not match the manifest"
    elif topic_name not in received_topics:
        status = "STALE"
        detail = "ROS2 topic produced no fresh message within max_age_seconds"
    else:
        status = "READY"
        detail = "ROS2 topic type and fresh message are valid"
    return {
        "type": "ros2_topic",
        "name": topic_name,
        "required": probe["required"],
        "ready": status == "READY",
        "status": status,
        "message_type": expected,
        "observed_type": observed_type,
        "latency_ms": latency_ms,
        "detail": detail,
    }


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


def _graph_command(argv: List[str], environment: Mapping[str, str]) -> Optional[str]:
    process_environment = os.environ.copy()
    process_environment.update(environment)
    try:
        completed = subprocess.run(
            argv,
            env=process_environment,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
            check=False,
            universal_newlines=True,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if completed.returncode != 0:
        return None
    return completed.stdout[: 1024 * 1024]


def _webots_renderer() -> str:
    environment = os.environ.copy()
    environment.update(
        {
            "DISPLAY": WEBOTS_DISPLAY,
            "VGL_DISPLAY": VIRTUALGL_DISPLAY,
            "NO_PROXY": _LOCAL_NO_PROXY,
            "no_proxy": _LOCAL_NO_PROXY,
        }
    )
    try:
        completed = subprocess.run(
            ["vglrun", "-d", VIRTUALGL_DISPLAY, "glxinfo", "-B"],
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15.0,
            check=False,
            universal_newlines=True,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ProbeRequestError("VirtualGL renderer probe could not execute") from exc
    if completed.returncode != 0:
        detail = completed.stdout.strip()[-500:]
        raise ProbeRequestError(f"VirtualGL renderer probe failed: {detail or 'no output'}")
    for line in completed.stdout.splitlines():
        if "OpenGL renderer string:" in line:
            return line.split("OpenGL renderer string:", 1)[1].strip()
    raise ProbeRequestError("VirtualGL renderer probe returned no OpenGL renderer")


def _nonempty_lines(output: Optional[str]) -> List[str]:
    return [line.strip() for line in (output or "").splitlines() if line.strip()]


def _topic_snapshot(agent_kind: str, output: Optional[str]) -> List[Dict[str, Optional[str]]]:
    topics: List[Dict[str, Optional[str]]] = []
    seen = set()
    for line in _nonempty_lines(output):
        if agent_kind == "ros2" and " [" in line and line.endswith("]"):
            name, message_type = line[:-1].split(" [", 1)
        elif agent_kind == "ros1":
            match = re.match(r"^\s*\*\s+(\S+)\s+\[([^]]+)\]", line)
            if match is not None:
                name, message_type = match.groups()
            elif line.startswith("/"):
                name, message_type = line, None
            else:
                continue
        else:
            name, message_type = line, None
        key = (name, message_type)
        if key not in seen:
            seen.add(key)
            topics.append({"name": name, "message_type": message_type})
    return topics


__all__ = ["ProbeRequestError", "ProbeRunner", "runtime_environment"]
