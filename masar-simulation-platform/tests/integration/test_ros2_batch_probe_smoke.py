"""ROS2-container smoke coverage for the batch readiness observer."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import subprocess

import pytest


pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("rclpy") is None or not Path("/root/ros2_ws/install/setup.bash").is_file(),
    reason="requires the ROS2 Runtime Agent environment",
)


def test_batch_observer_uses_its_own_ros_context() -> None:
    from agents.runtime_agent.probes import _observe_ros2_topics

    topic_types, received_topics, latency_ms = _observe_ros2_topics(
        [
            {
                "name": "/masar_readiness_missing_topic",
                "message_type": "std_msgs/msg/String",
                "qos_profile": "system_default",
                "max_age_seconds": 0.1,
            }
        ],
        42,
    )

    assert topic_types == {}
    assert received_topics == set()
    assert latency_ms >= 0


def test_ros2_agent_bootstrap_loads_the_webots_overlay(tmp_path: Path) -> None:
    fake_python = tmp_path / "python3"
    fake_python.write_text("#!/bin/sh\nprintf '%s' \"$PYTHONPATH\"\n", encoding="utf-8")
    fake_python.chmod(0o755)
    bootstrap = Path("/root/workspace/masar-simulation-platform/deploy/agent-bootstrap.sh")
    environment = os.environ | {"PATH": f"{tmp_path}:{os.environ['PATH']}"}

    completed = subprocess.run(
        ["bash", str(bootstrap), "ros2"],
        check=True,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    assert "/root/ros2_ws/install/webots_ros2_driver/local/lib/python3.10/dist-packages" in completed.stdout
