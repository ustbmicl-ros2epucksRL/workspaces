import subprocess

import pytest

from agents.runtime_agent import actions
from agents.runtime_agent.actions import ActionNotAllowed, ActionPublisher


RUNTIME = {"ros1_master_uri": "http://127.0.0.1:11311", "ros2_domain_id": 42}


def test_fixed_ros2_action_uses_domain_and_contains_no_shell(monkeypatch: pytest.MonkeyPatch) -> None:
    captured = {}

    def fake_run(argv, **kwargs):
        captured["argv"] = argv
        captured["env"] = kwargs["env"]
        captured["kwargs"] = kwargs
        return subprocess.CompletedProcess(argv, 0)

    monkeypatch.setattr(actions.subprocess, "run", fake_run)

    result = ActionPublisher("ros2").publish("ros2.swarm.start", RUNTIME)

    assert captured["argv"] == [
        "ros2",
        "topic",
        "pub",
        "--once",
        "/swarm_command",
        "communication_interfaces/msg/Int8Message",
        "{data: 1}",
    ]
    assert captured["env"]["ROS_DOMAIN_ID"] == "42"
    assert captured["kwargs"]["check"] is False
    assert "shell" not in captured["kwargs"]
    assert result["published"] is True


def test_agent_kind_and_action_allowlist_cannot_be_overridden() -> None:
    publisher = ActionPublisher("ros1")

    with pytest.raises(ActionNotAllowed):
        publisher.publish("ros2.swarm.start", RUNTIME)
    with pytest.raises(ActionNotAllowed):
        publisher.publish("rostopic pub /arbitrary", RUNTIME)
