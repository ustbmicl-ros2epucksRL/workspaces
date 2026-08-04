"""Fixed ROS command publisher used by the control API."""

from __future__ import annotations

import os
import re
import subprocess
from typing import Any, Dict, Mapping, Tuple

if __package__:
    from .probes import ProbeRequestError, runtime_environment
else:
    from probes import ProbeRequestError, runtime_environment  # type: ignore[no-redef]


class ActionNotAllowed(ValueError):
    pass


class ActionPublishError(RuntimeError):
    pass


_ACTIONS: Mapping[str, Tuple[str, Tuple[str, ...], str, str]] = {
    "ros1.mission.start": (
        "ros1",
        ("rostopic", "pub", "-1", "/system_mission_command", "robosar_messages/mission_command", "{data: 1}"),
        "/system_mission_command",
        "robosar_messages/mission_command",
    ),
    "ros1.mission.stop": (
        "ros1",
        ("rostopic", "pub", "-1", "/system_mission_command", "robosar_messages/mission_command", "{data: 2}"),
        "/system_mission_command",
        "robosar_messages/mission_command",
    ),
    "ros1.mission.home": (
        "ros1",
        ("rostopic", "pub", "-1", "/system_mission_command", "robosar_messages/mission_command", "{data: 3}"),
        "/system_mission_command",
        "robosar_messages/mission_command",
    ),
    "ros1.response.return_home": (
        "ros1",
        ("rostopic", "pub", "-1", "/response_group/return_home", "std_msgs/Bool", "{data: true}"),
        "/response_group/return_home",
        "std_msgs/Bool",
    ),
    "ros2.swarm.start": (
        "ros2",
        (
            "ros2",
            "topic",
            "pub",
            "--once",
            "/swarm_command",
            "communication_interfaces/msg/Int8Message",
            "{data: 1}",
        ),
        "/swarm_command",
        "communication_interfaces/msg/Int8Message",
    ),
    "ros2.swarm.stop": (
        "ros2",
        (
            "ros2",
            "topic",
            "pub",
            "--once",
            "/swarm_command",
            "communication_interfaces/msg/Int8Message",
            "{data: 0}",
        ),
        "/swarm_command",
        "communication_interfaces/msg/Int8Message",
    ),
}


class ActionPublisher:
    def __init__(self, agent_kind: str) -> None:
        self.agent_kind = agent_kind

    def publish(self, action: str, runtime: Mapping[str, Any]) -> Dict[str, Any]:
        definition = _ACTIONS.get(action)
        if definition is None or definition[0] != self.agent_kind:
            raise ActionNotAllowed("action is not allowed for this Agent")
        environment = os.environ.copy()
        try:
            environment.update(runtime_environment(runtime))
        except ProbeRequestError as exc:
            raise ActionNotAllowed(str(exc)) from exc
        _, argv, topic, message_type = definition
        publish_argv = list(argv)
        if action.startswith("ros2.swarm."):
            subscriber_count = _ros2_subscription_count(environment, topic)
            publish_argv[4:4] = [
                "--wait-matching-subscriptions",
                str(subscriber_count),
                "--qos-durability",
                "volatile",
                "--qos-reliability",
                "reliable",
                "--keep-alive",
                "1",
            ]
        try:
            completed = subprocess.run(
                publish_argv,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=10,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ActionPublishError("fixed ROS action publisher did not complete") from exc
        if completed.returncode != 0:
            raise ActionPublishError("fixed ROS action publisher returned a failure")
        return {"action": action, "topic": topic, "message_type": message_type, "published": True}


def _ros2_subscription_count(environment: Mapping[str, str], topic: str) -> int:
    try:
        completed = subprocess.run(
            ["ros2", "topic", "info", topic],
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ActionPublishError("could not inspect ROS2 command subscribers") from exc
    if completed.returncode != 0:
        raise ActionPublishError("could not inspect ROS2 command subscribers")
    match = re.search(r"^Subscription count:\s*(\d+)\s*$", completed.stdout, re.MULTILINE)
    if match is None or int(match.group(1)) < 1:
        raise ActionPublishError("ROS2 command topic has no subscribers")
    return int(match.group(1))


__all__ = ["ActionNotAllowed", "ActionPublishError", "ActionPublisher"]
