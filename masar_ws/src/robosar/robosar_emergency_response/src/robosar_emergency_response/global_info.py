"""Helpers for publishing ROS2swarm-compatible /global_info JSON in ROS1."""

from __future__ import annotations

import json
import math
import threading
from typing import Any, Mapping

from .checkpoint_controller import RobotInfo, SceneObjectInfo


class GlobalInfoStateBuffer:
    """Thread-safe handoff from ROS callbacks to the periodic JSON builder.

    rospy invokes odometry, model-state and task callbacks on connection
    threads while ``rospy.Timer`` runs independently. Returning a detached
    odometry dictionary prevents a concurrent callback from invalidating an
    in-progress iteration and killing the timer thread.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._robot_odom = {}
        self._model_states = None
        self._active_task = None

    def update_robot(self, robot_name: str, odometry: Any) -> None:
        with self._lock:
            self._robot_odom[robot_name] = odometry

    def update_model_states(self, model_states: Any) -> None:
        with self._lock:
            self._model_states = model_states

    def update_active_task(self, active_task: Any) -> None:
        with self._lock:
            self._active_task = active_task

    def snapshot(self):
        with self._lock:
            return dict(self._robot_odom), self._model_states, self._active_task


def yaw_to_orientation_matrix(yaw: float) -> list[float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return [
        cos_yaw,
        -sin_yaw,
        0.0,
        sin_yaw,
        cos_yaw,
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def build_global_info_payload(
    timestamp: float,
    robots: Mapping[str, RobotInfo],
    objects: Mapping[str, SceneObjectInfo],
) -> str:
    robots_info = {}
    for name, info in robots.items():
        robots_info[name] = {
            "position": {
                "x": float(info.pose.x),
                "y": float(info.pose.y),
                "z": float(info.pose.z),
            },
            "velocity": {
                "x": float(info.velocity_x),
                "y": float(info.velocity_y),
            },
            "orientation": [float(value) for value in info.orientation],
        }

    objects_info = {}
    for name, info in objects.items():
        objects_info[name] = {
            "position": {
                "x": float(info.x),
                "y": float(info.y),
                "z": float(info.z),
            },
            "velocity": {
                "x": float(info.velocity_x),
                "y": float(info.velocity_y),
            },
        }

    return json.dumps(
        {
            "timestamp": float(timestamp),
            "robots": robots_info,
            "objects": objects_info,
        },
        sort_keys=True,
    )
