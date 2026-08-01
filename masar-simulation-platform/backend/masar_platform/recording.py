"""Manifest-derived ROS bag recorder plans with no caller-supplied topics."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from typing import Any, Mapping

from .contracts import ScenarioManifest
from .profiles import get_artifact_profile, get_topic_profile


@dataclass(frozen=True, slots=True)
class RecorderPlan:
    agent: str
    process_id: str
    command_ref: str
    argv: tuple[str, ...]
    environment: Mapping[str, str]
    output_path: Path
    maximum_bytes_per_second: int


def normalize_recording(recording: Mapping[str, Any] | None) -> dict[str, Any]:
    value = dict(recording or {"enabled": False, "profile": "default"})
    if set(value) != {"enabled", "profile"}:
        raise ValueError("recording requires exactly enabled and profile")
    if not isinstance(value["enabled"], bool):
        raise ValueError("recording.enabled must be boolean")
    if value["profile"] != "default":
        raise ValueError("only the default recording profile is supported")
    return {"enabled": value["enabled"], "profile": "default"}


def build_recorder_plans(
    manifest: ScenarioManifest,
    recording: Mapping[str, Any] | None,
    run_dir: str | Path,
) -> tuple[RecorderPlan, ...]:
    normalized = normalize_recording(recording)
    if not normalized["enabled"]:
        return ()
    artifact_profile = get_artifact_profile(manifest.artifact_profile)
    if not artifact_profile.record_ros1 and not artifact_profile.record_ros2:
        raise ValueError("recording is unavailable for this scenario")
    topic_profile = get_topic_profile(manifest.data_profile)
    root = Path(run_dir)
    runtime = {
        "ROS_MASTER_URI": manifest.runtime.ros1_master_uri,
        "ROS_DOMAIN_ID": str(manifest.runtime.ros2_domain_id),
        "NO_PROXY": "127.0.0.1,localhost,::1",
        "no_proxy": "127.0.0.1,localhost,::1",
    }
    plans: list[RecorderPlan] = []
    if artifact_profile.record_ros1:
        regex = _topic_expression(
            spec.pattern
            for spec in topic_profile.topics
            if any("/msg/" not in message_type for message_type in spec.message_types)
        )
        if regex:
            output = root / "bags" / "ros1" / "recording"
            plans.append(
                RecorderPlan(
                    "ros1",
                    "recorder_ros1",
                    f"record.ros1.{manifest.data_profile}",
                    ("rosbag", "record", "--regex", "--output-name", "@run/bags/ros1/recording", regex),
                    runtime,
                    output.parent,
                    artifact_profile.maximum_bytes_per_second,
                )
            )
    if artifact_profile.record_ros2:
        regex = _topic_expression(
            spec.pattern
            for spec in topic_profile.topics
            if any("/msg/" in message_type for message_type in spec.message_types)
        )
        if regex:
            output = root / "bags" / "ros2"
            plans.append(
                RecorderPlan(
                    "ros2",
                    "recorder_ros2",
                    f"record.ros2.{manifest.data_profile}",
                    ("ros2", "bag", "record", "--regex", regex, "--output", "@run/bags/ros2"),
                    runtime,
                    output,
                    artifact_profile.maximum_bytes_per_second,
                )
            )
    return tuple(plans)


def _topic_expression(patterns) -> str:
    expressions = []
    for pattern in patterns:
        escaped = re.escape(pattern)
        escaped = escaped.replace(re.escape("<robot>"), r"[^/]+")
        escaped = escaped.replace(re.escape("<agent>"), r"[^/]+")
        escaped = escaped.replace(re.escape("tb_i"), r"tb_[0-9]+")
        expressions.append(escaped)
    return "^(?:" + "|".join(expressions) + ")$" if expressions else ""


__all__ = ["RecorderPlan", "build_recorder_plans", "normalize_recording"]
