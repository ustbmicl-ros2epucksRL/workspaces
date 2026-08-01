"""Live read-only rosbridge smoke test for the Task 5 acceptance run.

Run with the host ``python3`` on this server; it already provides
``websocket-client``.  The script expects the two business containers, the
temporary ROS1 master and the platform sidecars to be running.
"""

from __future__ import annotations

import json
import subprocess
import time
from typing import Any, Callable

from websocket import WebSocket, create_connection


ORIGIN = "http://127.0.0.1:18090"


def receive_until(socket: WebSocket, predicate: Callable[[dict[str, Any]], bool]) -> dict[str, Any]:
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        message = json.loads(socket.recv())
        if predicate(message):
            return message
    raise AssertionError("expected rosbridge message was not received")


def publish(container: str, command: str) -> None:
    completed = subprocess.run(
        ["docker", "exec", container, "bash", "-lc", command],
        check=False,
        capture_output=True,
        text=True,
        timeout=20,
    )
    if completed.returncode != 0:
        raise AssertionError(completed.stderr or completed.stdout)


def verify(
    *,
    path: str,
    topic_type: str,
    container: str,
    publisher_command: str,
    graph_command: str,
) -> None:
    socket = create_connection(
        f"ws://127.0.0.1:18090/{path}/",
        origin=ORIGIN,
        timeout=8,
        http_proxy_host=None,
    )
    try:
        socket.send(
            json.dumps(
                {
                    "op": "subscribe",
                    "id": f"{path}-subscription",
                    "topic": "/platform_smoke",
                    "type": topic_type,
                    "queue_length": 1,
                    "throttle_rate": 0,
                }
            )
        )
        time.sleep(1)
        publish(container, publisher_command)
        observed = receive_until(
            socket,
            lambda message: message.get("op") == "publish"
            and message.get("topic") == "/platform_smoke"
            and message.get("msg", {}).get("data") == f"{path}-ok",
        )
        assert observed["msg"]["data"] == f"{path}-ok"

        denied_topic = f"/platform_denied_{path}"
        socket.send(
            json.dumps(
                {
                    "op": "advertise",
                    "id": f"{path}-denied-advertise",
                    "topic": denied_topic,
                    "type": topic_type,
                }
            )
        )
        socket.send(
            json.dumps(
                {
                    "op": "publish",
                    "id": f"{path}-denied-publish",
                    "topic": denied_topic,
                    "msg": {"data": "must-not-enter-ros"},
                }
            )
        )
        time.sleep(1)
        graph = subprocess.run(
            ["docker", "exec", container, "bash", "-lc", graph_command],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert graph.returncode == 0, graph.stderr
        assert denied_topic not in graph.stdout.splitlines()
    finally:
        socket.close()


def main() -> None:
    verify(
        path="ros1",
        topic_type="std_msgs/String",
        container="masar_ros1_srv-g",
        publisher_command=(
            "source /opt/ros/noetic/setup.bash && "
            "rostopic pub -1 /platform_smoke std_msgs/String \"data: 'ros1-ok'\""
        ),
        graph_command="source /opt/ros/noetic/setup.bash && rostopic list",
    )
    verify(
        path="ros2",
        topic_type="std_msgs/msg/String",
        container="masar_ros2_bridge-w",
        publisher_command=(
            "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && "
            "ros2 topic pub --once /platform_smoke std_msgs/msg/String \"{data: ros2-ok}\""
        ),
        graph_command=(
            "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && ros2 topic list"
        ),
    )
    print("ROS1 and ROS2 subscription paths passed; browser publish capability was denied.")


if __name__ == "__main__":
    main()
