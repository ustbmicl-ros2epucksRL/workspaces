"""Immutable, explicit data/display/artifact profile registries."""

from __future__ import annotations

import re
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping


@dataclass(frozen=True, slots=True)
class TopicSpec:
    pattern: str
    message_types: tuple[str, ...]
    capability: str
    required: bool = False
    missing_label: str = "未提供"

    def matches(self, topic: str, message_type: str) -> bool:
        return bool(_topic_regex(self.pattern).fullmatch(topic)) and message_type in self.message_types

    @property
    def optional(self) -> bool:
        return not self.required


@dataclass(frozen=True, slots=True)
class TopicProfile:
    name: str
    topics: tuple[TopicSpec, ...]
    ui_rate_cap_hz: int
    recorder_max_bytes_per_second: int
    metrics: tuple[str, ...] = ()

    def matches(self, topic: str, message_type: str) -> bool:
        return any(spec.matches(topic, message_type) for spec in self.topics)

    @property
    def optional_topics(self) -> tuple[TopicSpec, ...]:
        return tuple(spec for spec in self.topics if not spec.required)

    @property
    def allowed_topics(self) -> tuple[TopicSpec, ...]:
        return self.topics

    @property
    def recorder_byte_cap(self) -> int:
        return self.recorder_max_bytes_per_second


@dataclass(frozen=True, slots=True)
class DisplayProfile:
    name: str
    panels: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ArtifactProfile:
    name: str
    data_profiles: tuple[str, ...]
    record_ros1: bool
    record_ros2: bool
    maximum_bytes_per_second: int


def _topic_regex(pattern: str) -> re.Pattern[str]:
    escaped = re.escape(pattern)
    escaped = escaped.replace(re.escape("<robot>"), r"[^/]+")
    escaped = escaped.replace(re.escape("tb_i"), r"tb_[0-9]+")
    escaped = escaped.replace(re.escape("<agent>"), r"[^/]+")
    return re.compile(escaped)


_ROS2_LASER = ("sensor_msgs/msg/LaserScan",)
_ROS2_ODOM = ("nav_msgs/msg/Odometry",)
_ROS2_TWIST = ("geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped")
_ROS1_LASER = ("sensor_msgs/LaserScan",)
_ROS1_ODOM = ("nav_msgs/Odometry",)
_ROS1_TWIST = ("geometry_msgs/Twist", "geometry_msgs/TwistStamped")


_SWARM_TOPICS = (
    TopicSpec("/<robot>/scan", _ROS2_LASER, "laser", required=True),
    TopicSpec("/<robot>/odom", _ROS2_ODOM, "odometry", required=True),
    TopicSpec("/<robot>/cmd_vel", _ROS2_TWIST, "command_velocity"),
    TopicSpec("/<robot>/camera", ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"), "camera"),
    TopicSpec("/<robot>/battery", ("sensor_msgs/msg/BatteryState",), "battery"),
    TopicSpec("/<robot>/safety", ("std_msgs/msg/Bool", "std_msgs/msg/String"), "safety"),
    TopicSpec("/global_info", ("std_msgs/msg/String",), "global_info"),
)

_RESPONSE_TOPICS = (
    TopicSpec("/tb_i/odom", _ROS1_ODOM, "odometry", required=True),
    TopicSpec("/tb_i/cmd_vel", _ROS1_TWIST, "command_velocity"),
    TopicSpec("/response_group/status", ("robosar_messages/ResponseGroupStatus",), "response_status"),
    TopicSpec("/response_group/behavior_mode", ("std_msgs/String",), "behavior_mode"),
    TopicSpec("/response_group/task_result", ("robosar_messages/EmergencyTask",), "task_result"),
)

_EXPLORATION_TOPICS = (
    TopicSpec("/sim_map", ("nav_msgs/OccupancyGrid",), "map", required=True),
    TopicSpec(
        "/frontier_filter/filtered_frontiers",
        ("geometry_msgs/PoseArray", "visualization_msgs/MarkerArray"),
        "frontiers",
    ),
    TopicSpec("/task_allocation_image", ("sensor_msgs/Image",), "task_allocation_image"),
    TopicSpec("/percent_completed", ("std_msgs/Float32",), "exploration_progress"),
    TopicSpec("/<agent>/status", ("std_msgs/String", "robosar_messages/AgentStatus"), "agent_status"),
    TopicSpec(
        "/emergency_objects",
        ("robosar_messages/EmergencyTask", "robosar_messages/EmergencyTaskArray"),
        "emergency_objects",
    ),
    TopicSpec("/emergency_events", ("robosar_messages/EmergencyTask",), "emergency_events"),
)

_EXPLORATION_WEBOTS_TOPICS = _EXPLORATION_TOPICS + tuple(
    TopicSpec(spec.pattern, spec.message_types, spec.capability, required=False)
    for spec in _SWARM_TOPICS
    if spec.pattern != "/global_info"
)


def _union_topics(*groups: tuple[TopicSpec, ...]) -> tuple[TopicSpec, ...]:
    result: list[TopicSpec] = []
    seen: set[tuple[str, tuple[str, ...]]] = set()
    for group in groups:
        for spec in group:
            key = (spec.pattern, spec.message_types)
            if key not in seen:
                seen.add(key)
                result.append(spec)
    return tuple(result)


TOPIC_PROFILES: Mapping[str, TopicProfile] = MappingProxyType(
    {
        "swarm_sensor": TopicProfile("swarm_sensor", _SWARM_TOPICS, 10, 2_000_000),
        "response_demo": TopicProfile("response_demo", _RESPONSE_TOPICS, 10, 2_000_000),
        "exploration": TopicProfile("exploration", _EXPLORATION_TOPICS, 10, 2_000_000),
        "exploration_webots": TopicProfile(
            "exploration_webots", _EXPLORATION_WEBOTS_TOPICS, 10, 2_000_000
        ),
        "rescue": TopicProfile("rescue", _union_topics(_EXPLORATION_TOPICS, _RESPONSE_TOPICS), 10, 8_000_000),
    }
)

DISPLAY_PROFILES: Mapping[str, DisplayProfile] = MappingProxyType(
    {
        "swarm": DisplayProfile("swarm", ("simulation", "sensors", "health", "logs")),
        "control": DisplayProfile("control", ("simulation", "sensors", "mission", "health", "logs")),
        "exploration": DisplayProfile(
            "exploration", ("simulation", "rviz", "map", "sensors", "mission", "health", "logs")
        ),
        "rescue": DisplayProfile(
            "rescue", ("simulation", "rviz", "map", "sensors", "mission", "response", "health", "logs")
        ),
    }
)

ARTIFACT_PROFILES: Mapping[str, ArtifactProfile] = MappingProxyType(
    {
        "none": ArtifactProfile("none", (), False, False, 0),
        "swarm_default": ArtifactProfile("swarm_default", ("swarm_sensor",), False, True, 2_000_000),
        "exploration_default": ArtifactProfile(
            "exploration_default", ("exploration", "exploration_webots"), True, True, 2_000_000
        ),
        "rescue_default": ArtifactProfile("rescue_default", ("rescue",), True, False, 8_000_000),
    }
)


def get_topic_profile(name: str) -> TopicProfile:
    try:
        return TOPIC_PROFILES[name]
    except KeyError as exc:
        raise KeyError(f"unknown topic profile: {name}") from exc


def get_display_profile(name: str) -> DisplayProfile:
    try:
        return DISPLAY_PROFILES[name]
    except KeyError as exc:
        raise KeyError(f"unknown display profile: {name}") from exc


def get_artifact_profile(name: str) -> ArtifactProfile:
    try:
        return ARTIFACT_PROFILES[name]
    except KeyError as exc:
        raise KeyError(f"unknown artifact profile: {name}") from exc


__all__ = [
    "ARTIFACT_PROFILES",
    "DISPLAY_PROFILES",
    "TOPIC_PROFILES",
    "ArtifactProfile",
    "DisplayProfile",
    "TopicProfile",
    "TopicSpec",
    "get_artifact_profile",
    "get_display_profile",
    "get_topic_profile",
]
