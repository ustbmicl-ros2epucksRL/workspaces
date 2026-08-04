from __future__ import annotations

from pathlib import Path
import socket
import subprocess
from threading import Lock, Thread
import time

import pytest

from agents.runtime_agent import probes
from agents.runtime_agent.probes import ProbeRequestError, ProbeRunner


RUNTIME = {"ros1_master_uri": "http://127.0.0.1:11311", "ros2_domain_id": 42}


def test_readiness_probes_in_one_step_run_concurrently(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    runner = ProbeRunner("ros1", tmp_path)
    lock = Lock()
    active = 0
    peak_active = 0

    def slow_probe(payload, runtime_environment):
        nonlocal active, peak_active
        with lock:
            active += 1
            peak_active = max(peak_active, active)
        time.sleep(0.1)
        with lock:
            active -= 1
        return {"name": payload["name"], "ready": True}

    monkeypatch.setattr(runner, "run", slow_probe)
    results = runner.run_many(
        [
            {"type": "file", "name": "one"},
            {"type": "file", "name": "two"},
            {"type": "file", "name": "three"},
        ],
        RUNTIME,
    )

    assert [result["name"] for result in results] == ["one", "two", "three"]
    assert peak_active >= 2


def test_ros2_topic_readiness_uses_one_shared_observation_session(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = ProbeRunner("ros2", tmp_path)
    payloads = [
        {
            "type": "ros2_topic",
            "name": "/clock",
            "message_type": "rosgraph_msgs/msg/Clock",
            "max_age_seconds": 5,
        },
        *(
            {
                "type": "ros2_topic",
                "name": f"/robot_{index}/odom",
                "message_type": "nav_msgs/msg/Odometry",
                "qos_profile": "sensor_data",
                "max_age_seconds": 5,
            }
            for index in range(5)
        ),
    ]
    observed_batches: list[tuple[list[tuple[str, str]], int]] = []

    def observe(probes_to_observe, domain_id):
        names = [probe["name"] for probe in probes_to_observe]
        observed_batches.append(
            ([(probe["name"], probe["qos_profile"]) for probe in probes_to_observe], domain_id)
        )
        return (
            {probe["name"]: (probe["message_type"],) for probe in probes_to_observe},
            set(names),
            18.5,
        )

    monkeypatch.setattr(probes, "_observe_ros2_topics", observe, raising=False)
    monkeypatch.setattr(
        runner,
        "run",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("ROS2 topics must be batched")),
    )

    results = runner.run_many(payloads, RUNTIME)

    assert observed_batches == [
        (
            [
                (payload["name"], payload.get("qos_profile", "system_default"))
                for payload in payloads
            ],
            42,
        )
    ]
    assert [result["name"] for result in results] == [payload["name"] for payload in payloads]
    assert all(result["status"] == "READY" for result in results)
    assert all(result["latency_ms"] == 18.5 for result in results)


def test_tcp_and_workspace_file_probes_are_bounded_to_loopback(tmp_path: Path) -> None:
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    thread = Thread(target=lambda: server.accept()[0].close(), daemon=True)
    thread.start()
    marker = tmp_path / "ready.flag"
    marker.write_text("ready", encoding="utf-8")
    runner = ProbeRunner("ros2", tmp_path)
    try:
        results = runner.run_many(
            [
                {"type": "tcp", "name": f"127.0.0.1:{server.getsockname()[1]}"},
                {"type": "file", "name": "ready.flag"},
            ],
            RUNTIME,
        )
    finally:
        server.close()
        thread.join(timeout=1)

    assert [result["status"] for result in results] == ["READY", "READY"]
    with pytest.raises(ProbeRequestError, match="loopback"):
        runner.run_many([{"type": "tcp", "name": "8.8.8.8:53"}], RUNTIME)
    unreachable = runner.run_many([{"type": "tcp", "name": "localhost:1"}], RUNTIME)[0]
    assert unreachable["status"] == "UNREACHABLE"
    with pytest.raises(ProbeRequestError, match="workspace"):
        runner.run_many([{"type": "file", "name": "../outside"}], RUNTIME)


def test_ros2_probe_checks_exact_type_domain_qos_and_fresh_message(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[list[dict[str, object]], int]] = []

    def observe(probes_to_observe, domain_id):
        calls.append((list(probes_to_observe), domain_id))
        return ({"/robot_0/odom": ("nav_msgs/msg/Odometry",)}, {"/robot_0/odom"}, 7.25)

    monkeypatch.setattr(probes, "_observe_ros2_topics", observe, raising=False)
    result = ProbeRunner("ros2", tmp_path).run_many(
        [
            {
                "type": "ros2_topic",
                "name": "/robot_0/odom",
                "message_type": "nav_msgs/msg/Odometry",
                "qos_profile": "sensor_data",
            }
        ],
        RUNTIME,
    )[0]

    assert result["status"] == "READY"
    assert result["latency_ms"] == 7.25
    assert calls[0][1] == 42
    assert calls[0][0][0]["qos_profile"] == "sensor_data"


def test_ros_probe_reports_type_mismatch_and_stale_without_shell(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    observations = iter(
        [
            ({"/odom": ("std_msgs/msg/String",)}, set(), 1.0),
            ({"/odom": ("nav_msgs/msg/Odometry",)}, set(), 5.0),
        ]
    )

    monkeypatch.setattr(probes, "_observe_ros2_topics", lambda *_args: next(observations), raising=False)
    monkeypatch.setattr(
        probes.subprocess,
        "run",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("ROS2 probes must not spawn CLI")),
    )
    runner = ProbeRunner("ros2", tmp_path)
    mismatch = runner.run_many(
        [{"type": "ros2_topic", "name": "/odom", "message_type": "nav_msgs/msg/Odometry"}],
        RUNTIME,
    )[0]
    stale = runner.run_many(
        [{"type": "ros2_topic", "name": "/odom", "message_type": "nav_msgs/msg/Odometry"}],
        RUNTIME,
    )[0]

    assert mismatch["status"] == "TYPE_MISMATCH"
    assert mismatch["observed_type"] == "std_msgs/msg/String"
    assert stale["status"] == "STALE"
    assert stale["ready"] is False


def test_probe_request_requires_message_type_and_matching_agent(tmp_path: Path) -> None:
    runner = ProbeRunner("ros1", tmp_path)
    with pytest.raises(ProbeRequestError, match="message_type"):
        runner.run_many([{"type": "ros1_topic", "name": "/clock"}], RUNTIME)
    with pytest.raises(ProbeRequestError, match="ROS2 Agent"):
        runner.run_many(
            [{"type": "ros2_topic", "name": "/clock", "message_type": "rosgraph_msgs/msg/Clock"}],
            RUNTIME,
        )


@pytest.mark.parametrize(
    ("agent", "expected_commands"),
    (
        ("ros1", [("rosnode", "list"), ("rostopic", "list", "-v")]),
        ("ros2", [("ros2", "node", "list"), ("ros2", "topic", "list", "-t")]),
    ),
)
def test_graph_snapshot_is_bounded_shell_free_and_runtime_scoped(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    agent: str,
    expected_commands: list[tuple[str, ...]],
) -> None:
    calls: list[tuple[tuple[str, ...], dict[str, str]]] = []

    def fake_run(argv, **kwargs):
        calls.append((tuple(argv), kwargs["env"]))
        output = "/node_a\n" if any("node" in part for part in argv) else "/topic_a [std_msgs/msg/String]\n"
        return subprocess.CompletedProcess(argv, 0, stdout=output, stderr="")

    monkeypatch.setattr(probes.subprocess, "run", fake_run)

    snapshot = ProbeRunner(agent, tmp_path).graph_snapshot(RUNTIME)

    assert [call[0] for call in calls] == expected_commands
    assert all(call[1]["ROS_DOMAIN_ID"] == "42" for call in calls)
    assert snapshot["agent"] == agent
    assert snapshot["available"] is True
    assert snapshot["nodes_available"] is True
    assert snapshot["topics_available"] is True
    assert snapshot["nodes"] == ["/node_a"]
    assert snapshot["topics"]


def test_webots_platform_preflight_requires_nvidia_and_optional_bridge(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bridge_setup = tmp_path / "bridge" / "setup.bash"
    bridge_executable = tmp_path / "bridge" / "dynamic_bridge"
    bridge_setup.parent.mkdir(parents=True)
    bridge_setup.write_text("# test overlay\n", encoding="utf-8")
    bridge_executable.write_text("#!/bin/sh\n", encoding="utf-8")
    bridge_executable.chmod(0o755)
    monkeypatch.setattr(probes, "ROS1_BRIDGE_SETUP", bridge_setup)
    monkeypatch.setattr(probes, "ROS1_BRIDGE_EXECUTABLE", bridge_executable)
    monkeypatch.setattr(
        probes,
        "_webots_renderer",
        lambda: "NVIDIA RTX 6000 Ada Generation/PCIe/SSE2",
    )

    result = ProbeRunner("ros2", tmp_path).webots_preflight(require_bridge=True)

    assert result == {
        "ready": True,
        "display": ":103",
        "renderer": "NVIDIA RTX 6000 Ada Generation/PCIe/SSE2",
        "bridge_setup": str(bridge_setup),
        "bridge_executable": str(bridge_executable),
    }


def test_webots_platform_preflight_rejects_software_renderer(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(probes, "_webots_renderer", lambda: "llvmpipe (LLVM 15.0.7, 256 bits)")

    with pytest.raises(ProbeRequestError, match="NVIDIA"):
        ProbeRunner("ros2", tmp_path).webots_preflight(require_bridge=False)
