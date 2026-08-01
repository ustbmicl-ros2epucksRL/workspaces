from __future__ import annotations

from pathlib import Path
import socket
import subprocess
from threading import Thread

import pytest

from agents.runtime_agent import probes
from agents.runtime_agent.probes import ProbeRequestError, ProbeRunner


RUNTIME = {"ros1_master_uri": "http://127.0.0.1:11311", "ros2_domain_id": 42}


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
    calls: list[tuple[list[str], dict[str, str]]] = []

    def fake_run(argv, **kwargs):
        calls.append((list(argv), kwargs["env"]))
        if argv[:3] == ["ros2", "topic", "type"]:
            return subprocess.CompletedProcess(argv, 0, stdout="nav_msgs/msg/Odometry\n", stderr="")
        return subprocess.CompletedProcess(argv, 0, stdout="", stderr="")

    monkeypatch.setattr(probes.subprocess, "run", fake_run)
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
    assert calls[0][1]["ROS_DOMAIN_ID"] == "42"
    assert calls[0][1]["NO_PROXY"] == "127.0.0.1,localhost,::1"
    assert calls[1][0][-2:] == ["--qos-profile", "sensor_data"]


def test_ros_probe_reports_type_mismatch_and_stale_without_shell(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    responses = iter(
        [
            subprocess.CompletedProcess([], 0, stdout="std_msgs/String\n", stderr=""),
            subprocess.CompletedProcess([], 0, stdout="nav_msgs/Odometry\n", stderr=""),
        ]
    )

    def fake_run(argv, **kwargs):
        if argv[:3] == ["ros2", "topic", "echo"]:
            raise subprocess.TimeoutExpired(argv, kwargs["timeout"])
        return next(responses)

    monkeypatch.setattr(probes.subprocess, "run", fake_run)
    runner = ProbeRunner("ros2", tmp_path)
    mismatch = runner.run_many(
        [{"type": "ros2_topic", "name": "/odom", "message_type": "nav_msgs/msg/Odometry"}],
        RUNTIME,
    )[0]
    stale = runner.run_many(
        [{"type": "ros2_topic", "name": "/odom", "message_type": "nav_msgs/Odometry"}],
        RUNTIME,
    )[0]

    assert mismatch["status"] == "TYPE_MISMATCH"
    assert mismatch["observed_type"] == "std_msgs/String"
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
