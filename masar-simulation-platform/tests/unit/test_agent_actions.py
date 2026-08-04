import subprocess

import pytest

from agents.runtime_agent import actions
from agents.runtime_agent.actions import ActionNotAllowed, ActionPublisher


RUNTIME = {"ros1_master_uri": "http://127.0.0.1:11311", "ros2_domain_id": 42}


def test_fixed_ros2_action_uses_domain_and_contains_no_shell(monkeypatch: pytest.MonkeyPatch) -> None:
    captured = []

    def fake_run(argv, **kwargs):
        captured.append({"argv": argv, "env": kwargs["env"], "kwargs": kwargs})
        if argv == ["ros2", "topic", "info", "/swarm_command"]:
            return subprocess.CompletedProcess(
                argv,
                0,
                stdout="Type: communication_interfaces/msg/Int8Message\nPublisher count: 0\nSubscription count: 12\n",
                stderr="",
            )
        return subprocess.CompletedProcess(argv, 0, stdout="", stderr="")

    monkeypatch.setattr(actions.subprocess, "run", fake_run)

    result = ActionPublisher("ros2").publish("ros2.swarm.start", RUNTIME)

    assert captured[0]["argv"] == ["ros2", "topic", "info", "/swarm_command"]
    assert captured[1]["argv"] == [
        "ros2",
        "topic",
        "pub",
        "--once",
        "--wait-matching-subscriptions",
        "12",
        "--qos-durability",
        "volatile",
        "--qos-reliability",
        "reliable",
        "--keep-alive",
        "1",
        "/swarm_command",
        "communication_interfaces/msg/Int8Message",
        "{data: 1}",
    ]
    assert captured[1]["env"]["ROS_DOMAIN_ID"] == "42"
    assert captured[1]["kwargs"]["check"] is False
    assert "shell" not in captured[1]["kwargs"]
    assert result["published"] is True


def test_agent_kind_and_action_allowlist_cannot_be_overridden() -> None:
    publisher = ActionPublisher("ros1")

    with pytest.raises(ActionNotAllowed):
        publisher.publish("ros2.swarm.start", RUNTIME)
    with pytest.raises(ActionNotAllowed):
        publisher.publish("rostopic pub /arbitrary", RUNTIME)
