from masar_platform.profiles import (
    ARTIFACT_PROFILES,
    DISPLAY_PROFILES,
    TOPIC_PROFILES,
    get_topic_profile,
)


def test_required_topic_profiles_are_explicit_and_immutable() -> None:
    assert set(TOPIC_PROFILES) == {"swarm_sensor", "response_demo", "exploration", "exploration_webots", "rescue"}
    assert all(profile.ui_rate_cap_hz == 10 for profile in TOPIC_PROFILES.values())
    assert TOPIC_PROFILES["swarm_sensor"].recorder_max_bytes_per_second == 2_000_000
    assert TOPIC_PROFILES["rescue"].recorder_max_bytes_per_second == 8_000_000
    assert "/percent_completed" in {topic.pattern for topic in TOPIC_PROFILES["exploration"].topics}
    assert all(
        "mean" not in metric.lower() and "velocity" not in metric.lower()
        for metric in TOPIC_PROFILES["swarm_sensor"].metrics
    )


def test_profile_type_checks_and_optional_topics() -> None:
    profile = get_topic_profile("swarm_sensor")
    assert profile.matches("/robot_2/scan", "sensor_msgs/msg/LaserScan")
    assert profile.matches("/robot_2/odom", "nav_msgs/msg/Odometry")
    assert not profile.matches("/percent_completed", "std_msgs/msg/Float32")
    assert profile.optional_topics


def test_display_and_artifact_profiles_are_referenced_registries() -> None:
    assert {"swarm", "control", "exploration", "rescue"}.issubset(DISPLAY_PROFILES)
    assert {"none", "swarm_default", "exploration_default", "rescue_default"}.issubset(ARTIFACT_PROFILES)
