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


def test_swarm_profile_exposes_existing_discussed_dispersion_topics() -> None:
    profile = get_topic_profile("swarm_sensor")
    topics = {topic.pattern: topic.message_types for topic in profile.topics}

    assert topics["/<robot>/opinion"] == ("communication_interfaces/msg/OpinionMessage",)
    assert topics["/<robot>/dispersion_distance"] == ("communication_interfaces/msg/DoubleMessage",)
    assert topics["/majority_broadcast"] == ("communication_interfaces/msg/OpinionMessage",)
    assert profile.matches(
        "/robot_3/opinion",
        "communication_interfaces/msg/OpinionMessage",
    )


def test_swarm_profile_exposes_epuck_camera_and_individual_infrared_ranges() -> None:
    profile = get_topic_profile("swarm_sensor")
    topics = {topic.pattern: topic.message_types for topic in profile.topics}

    assert topics["/<robot>/camera/image_color"] == ("sensor_msgs/msg/Image",)
    assert topics["/<robot>/ps0"] == ("sensor_msgs/msg/Range",)
    assert topics["/<robot>/ps7"] == ("sensor_msgs/msg/Range",)
    assert profile.matches("/robot_1/camera/image_color", "sensor_msgs/msg/Image")
    assert profile.matches("/robot_1/ps4", "sensor_msgs/msg/Range")


def test_swarm_profile_exposes_burger_imu() -> None:
    profile = get_topic_profile("swarm_sensor")
    topics = {topic.pattern: topic.message_types for topic in profile.topics}

    assert topics["/<robot>/imu"] == ("sensor_msgs/msg/Imu",)
    assert profile.matches("/robot_2/imu", "sensor_msgs/msg/Imu")


def test_group_behavior_profiles_expose_simulator_world_pose_sources() -> None:
    swarm = {topic.pattern: topic for topic in TOPIC_PROFILES["swarm_sensor"].topics}
    response = {topic.pattern: topic for topic in TOPIC_PROFILES["response_demo"].topics}

    assert swarm["/global_info"].capability == "global_info"
    assert swarm["/global_info"].message_types == ("std_msgs/msg/String",)
    assert response["/gazebo/model_states"].capability == "gazebo_model_states"
    assert response["/gazebo/model_states"].message_types == ("gazebo_msgs/ModelStates",)


def test_response_profile_exposes_only_the_active_burger_realsense_color_stream() -> None:
    profile = get_topic_profile("response_demo")
    topics = {topic.pattern: topic for topic in profile.topics}

    assert topics["/tb_i/scan"].message_types == ("sensor_msgs/LaserScan",)
    assert topics["/tb_i/scan"].capability == "laser"
    assert profile.matches("/tb_2/scan", "sensor_msgs/LaserScan")
    assert topics["/tb_i/imu"].message_types == ("sensor_msgs/Imu",)
    assert topics["/tb_i/imu"].capability == "imu"
    assert profile.matches("/tb_2/imu", "sensor_msgs/Imu")
    assert topics["/tb_i/camera/color/image_raw"].message_types == ("sensor_msgs/Image",)
    assert topics["/tb_i/camera/color/image_raw"].capability == "camera"
    assert profile.matches("/tb_2/camera/color/image_raw", "sensor_msgs/Image")
    assert "/tb_i/camera/depth/image_raw" not in topics
    assert not profile.matches("/tb_2/camera/depth/image_raw", "sensor_msgs/Image")

    rescue_topics = {topic.pattern: topic for topic in get_topic_profile("rescue").topics}
    assert rescue_topics["/tb_i/scan"].capability == "laser"
    assert rescue_topics["/tb_i/imu"].capability == "imu"


def test_display_and_artifact_profiles_are_referenced_registries() -> None:
    assert {"swarm", "control", "exploration", "rescue"}.issubset(DISPLAY_PROFILES)
    assert {"none", "swarm_default", "exploration_default", "rescue_default"}.issubset(ARTIFACT_PROFILES)
    assert "mission" not in DISPLAY_PROFILES["control"].panels


def test_exploration_profile_uses_real_ros1_topic_types() -> None:
    profile = TOPIC_PROFILES["exploration"]
    expected = {
        "/information_map": ("nav_msgs/OccupancyGrid",),
        "/frontier_filter/filtered_frontiers": ("robosar_messages/PointArray",),
        "/task_allocation": ("robosar_messages/task_allocation",),
        "/task_allocation_image": ("sensor_msgs/Image",),
        "/percent_completed": ("std_msgs/Float32",),
        "/robosar_agent_bringup_node/all_agent_status": ("robosar_messages/agents_status",),
        "/robosar_sim/emergency_objects": ("robosar_messages/EmergencyObjectArray",),
        "/robosar_sim/emergency_events": ("robosar_messages/EmergencyObject",),
    }
    topics = {topic.pattern: topic.message_types for topic in profile.topics}
    assert expected.items() <= topics.items()
    assert "/tasks_completed" not in topics
    for inherited_name in ("exploration_webots", "rescue"):
        inherited = {topic.pattern: topic.message_types for topic in TOPIC_PROFILES[inherited_name].topics}
        assert expected.items() <= inherited.items()
        assert "/tasks_completed" not in inherited


def test_exploration_profile_exposes_real_gazebo_robot_observations() -> None:
    profile = TOPIC_PROFILES["exploration"]
    expected = {
        "/robosar_agent_bringup_node/<agent>/feedback/scan": ("sensor_msgs/LaserScan",),
        "/robosar_agent_bringup_node/<agent>/camera/image_raw": ("sensor_msgs/Image",),
        "/robosar_agent_bringup_node/<agent>/odom_data_euler": ("nav_msgs/Odometry",),
        "/robosar_agent_bringup_node/<agent>/cmd_vel": ("geometry_msgs/Twist",),
    }
    topics = {topic.pattern: topic.message_types for topic in profile.topics}

    assert expected.items() <= topics.items()
    assert profile.matches(
        "/robosar_agent_bringup_node/agent_4/feedback/scan",
        "sensor_msgs/LaserScan",
    )
    assert not profile.matches(
        "/robosar_agent_bringup_node/agent_4/feedback/scan",
        "sensor_msgs/msg/LaserScan",
    )
    for inherited_name in ("rescue",):
        inherited = {topic.pattern: topic.message_types for topic in TOPIC_PROFILES[inherited_name].topics}
        assert expected.items() <= inherited.items()


def test_rescue_profile_exposes_response_tasks_motion_and_continuous_status() -> None:
    profile = TOPIC_PROFILES["rescue"]
    expected = {
        "/emergency/tasks": ("robosar_messages/EmergencyTaskArray",),
        "/response_group/active_task": ("robosar_messages/EmergencyTask",),
        "/response_group/status": ("robosar_messages/ResponseGroupStatus",),
        "/response_group/behavior_mode": ("std_msgs/String",),
        "/response_group/motion_enabled": ("std_msgs/Bool",),
        "/response_group/task_result": ("robosar_messages/EmergencyTask",),
    }
    topics = {topic.pattern: topic.message_types for topic in profile.topics}

    assert expected.items() <= topics.items()
