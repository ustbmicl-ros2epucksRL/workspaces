import math
import os
import sys

import yaml


PACKAGE_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from robosar_emergency_response.checkpoint_controller import (  # noqa: E402
    MatrixPose,
    RobotInfo,
    SceneObjectInfo,
    build_roundflocking_heuristic_observation,
    build_roundflocking_observation,
    build_transport_observation,
    direct_transport_output_to_global_action,
    differential_twist_from_omni_output,
    direct_twist_from_network_output,
    signed_outputs_from_neat,
)
from robosar_emergency_response.cmd_mux import (  # noqa: E402
    MODE_NAV_FORMATION,
    MODE_TASK_CONTROL,
    Twist2D,
    ZERO_TWIST,
    apply_hard_scan_safety,
    clamp_command,
    select_command,
)
from robosar_emergency_response.agent_safety import (  # noqa: E402
    AgentSafetyPose,
    apply_external_priority_safety,
    apply_teammate_hard_safety,
    circle_range_exclusion,
)


def _assert_sequence_close(actual, expected, abs_tol=1e-7):
    assert len(actual) == len(expected)
    for left, right in zip(actual, expected):
        assert math.isclose(left, right, abs_tol=abs_tol)


def _robot(x, y, yaw, vx=0.0, vy=0.0):
    return RobotInfo(
        pose=MatrixPose(x, y, yaw),
        velocity_x=vx,
        velocity_y=vy,
        orientation=[
            math.cos(yaw),
            -math.sin(yaw),
            0.0,
            math.sin(yaw),
            math.cos(yaw),
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    )


def test_motion_gate_blocks_fresh_navigation_and_task_commands():
    for mode in (MODE_NAV_FORMATION, MODE_TASK_CONTROL):
        selected = select_command(
            mode=mode,
            motion_enabled=False,
            deform_cmd=Twist2D(0.2, 0.4),
            task_cmd=Twist2D(0.1, -0.3),
            now=10.0,
            last_deform_time=10.0,
            last_task_time=10.0,
            command_timeout=0.6,
        )
        assert selected == ZERO_TWIST


def test_open_motion_gate_selects_fresh_command_for_current_mode():
    selected = select_command(
        mode=MODE_NAV_FORMATION,
        motion_enabled=True,
        deform_cmd=Twist2D(0.2, 0.4),
        task_cmd=Twist2D(0.1, -0.3),
        now=10.0,
        last_deform_time=10.0,
        last_task_time=10.0,
        command_timeout=0.6,
    )
    assert selected == Twist2D(0.2, 0.4)


def test_roundflocking_observation_matches_ros2swarm_body_frame_contract():
    observation = build_roundflocking_observation(
        robot=_robot(2.0, 1.0, math.pi / 2.0, vx=0.0, vy=1.0),
        target=SceneObjectInfo(1.0, 1.0),
        lidar_ranges=[0.16] * 360,
        n_lidar_rays=12,
        max_range=3.5,
        world_scale=2.0,
    )

    _assert_sequence_close(observation[:6], [
        1.0,
        0.5,
        1.0,
        0.0,
        0.0,
        -0.5,
    ])
    _assert_sequence_close(observation[6:], [0.16] * 12)


def test_roundflocking_heuristic_observation_keeps_metric_lidar_clearance():
    observation = build_roundflocking_heuristic_observation(
        robot=_robot(0.6, 0.0, 0.0),
        target=SceneObjectInfo(0.0, 0.0),
        lidar_ranges=[3.5] * 360,
        n_lidar_rays=12,
        max_range=3.5,
        world_scale=1.75,
    )

    _assert_sequence_close(observation[6:], [3.5] * 12)


def test_roundflocking_heuristic_uses_sector_minimum_instead_of_missing_narrow_robot():
    ranges = [3.5] * 360
    # Sector zero spans beams [0, 30). Its old midpoint-only sample was beam
    # 15, so a TB3 occupying beams 4..8 was invisible to task control.
    for index in range(4, 9):
        ranges[index] = 0.31

    heuristic = build_roundflocking_heuristic_observation(
        robot=_robot(0.6, 0.0, 0.0),
        target=SceneObjectInfo(0.0, 0.0),
        lidar_ranges=ranges,
        n_lidar_rays=12,
        max_range=3.5,
        world_scale=1.0,
    )
    checkpoint = build_roundflocking_observation(
        robot=_robot(0.6, 0.0, 0.0),
        target=SceneObjectInfo(0.0, 0.0),
        lidar_ranges=ranges,
        n_lidar_rays=12,
        max_range=3.5,
        world_scale=1.0,
    )

    assert math.isclose(heuristic[6], 0.31, abs_tol=1e-6)
    assert math.isclose(checkpoint[6], 0.2, abs_tol=1e-6)


def test_transport_observation_matches_ros2swarm_body_frame_contract():
    observation = build_transport_observation(
        robot=_robot(0.0, 0.0, math.pi / 2.0, vx=0.0, vy=1.0),
        box=SceneObjectInfo(1.0, 0.0, velocity_x=0.0, velocity_y=1.0),
        target=SceneObjectInfo(2.0, 0.0),
        world_scale=2.0,
        goal_threshold=0.15,
    )

    _assert_sequence_close(observation, [
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.5,
        0.0,
        -0.5,
        1.0,
        0.0,
        0.0,
    ])


def test_neat_output_mapping_supports_sigmoid_and_tanh():
    assert signed_outputs_from_neat([0.75, 0.25], ["sigmoid", "sigmoid"]) == (0.5, -0.5)
    assert signed_outputs_from_neat([0.75, -0.25], ["tanh", "tanh"]) == (0.75, -0.25)


def test_ros2swarm_output_conversions_use_body_frame_actions():
    twist = differential_twist_from_omni_output(
        [1.0, 0.5],
        max_linear_velocity=0.15,
        max_angular_velocity=0.8,
        output_activations=["sigmoid", "sigmoid"],
    )
    assert math.isclose(twist.linear_x, 0.15)
    assert math.isclose(twist.angular_z, 0.0)

    direct = direct_twist_from_network_output(
        [0.25, 0.75],
        max_linear_velocity=0.15,
        max_angular_velocity=0.8,
        output_activations=["sigmoid", "sigmoid"],
    )
    assert math.isclose(direct.linear_x, -0.075)
    assert math.isclose(direct.angular_z, 0.4)

    action = direct_transport_output_to_global_action(
        [1.0, 0.5],
        robot_yaw=math.pi / 2.0,
        output_activations=["sigmoid", "sigmoid"],
    )
    _assert_sequence_close(action, [0.0, 1.0])


def test_hard_safety_stops_forward_motion_inside_clearance_but_allows_escape():
    command = Twist2D(0.15, 0.4)
    stopped = apply_hard_scan_safety(
        command, [0.15, float("inf")], 0.0, math.pi,
        hard_stop_distance=0.20, motion_half_width=0.11,
        scan_age=0.05, scan_timeout=0.30,
    )
    escaping = apply_hard_scan_safety(
        command, [float("inf"), 0.12], 0.0, math.pi,
        hard_stop_distance=0.20, motion_half_width=0.11,
        scan_age=0.05, scan_timeout=0.30,
    )

    assert stopped == Twist2D(0.0, 0.4)
    assert escaping == command


def test_hard_safety_keeps_zero_command_zero():
    assert apply_hard_scan_safety(
        ZERO_TWIST, [], 0.0, 0.0,
        hard_stop_distance=0.20, motion_half_width=0.11,
        scan_age=None, scan_timeout=0.30,
    ) == ZERO_TWIST


def test_hard_safety_ignores_self_returns_but_stops_outside_footprint():
    command = Twist2D(0.10, 0.0)
    self_return = apply_hard_scan_safety(
        command, [0.16], 0.0, 1.0,
        hard_stop_distance=0.20, motion_half_width=0.11,
        scan_age=0.05, scan_timeout=0.30, self_filter_radius=0.18,
    )
    external_return = apply_hard_scan_safety(
        command, [0.19], 0.0, 1.0,
        hard_stop_distance=0.20, motion_half_width=0.11,
        scan_age=0.05, scan_timeout=0.30, self_filter_radius=0.18,
    )
    assert self_return == command
    assert external_return.linear_x == 0.0


def test_hard_safety_allows_close_side_return_outside_swept_corridor():
    command = Twist2D(0.10, 0.2)
    side_return = apply_hard_scan_safety(
        command, [0.19], 1.2, 1.0,
        hard_stop_distance=0.20, motion_half_width=0.11,
        scan_age=0.05, scan_timeout=0.30,
    )

    assert side_return == command


def test_box_contact_exclusion_ignores_only_expected_box_return():
    observer = AgentSafetyPose(0.0, 0.0, 0.0)
    box = AgentSafetyPose(0.36, 0.0, 0.0)
    exclusion = circle_range_exclusion(observer, box, radius=0.25, range_padding=0.03)
    command = Twist2D(0.10, 0.0)

    expected_box = apply_hard_scan_safety(
        command, [0.11], 0.0, 1.0,
        hard_stop_distance=0.20, motion_half_width=0.11,
        scan_age=0.05, scan_timeout=0.30, range_exclusions=[exclusion],
    )
    unexpected_nearer_wall = apply_hard_scan_safety(
        command, [0.02], 0.0, 1.0,
        hard_stop_distance=0.20, motion_half_width=0.11,
        scan_age=0.05, scan_timeout=0.30, range_exclusions=[exclusion],
    )

    assert expected_box == command
    assert unexpected_nearer_wall.linear_x == 0.0


def test_transport_controlled_contact_can_pass_teammate_hard_safety():
    command = Twist2D(0.10, -0.3)
    poses = {
        "tb_0": AgentSafetyPose(0.0, 0.0, 0.0),
        "tb_1": AgentSafetyPose(0.13, 0.0, 0.0),
    }
    result = apply_teammate_hard_safety(
        command, "tb_0", poses, minimum_center_distance=0.22,
        allowed_controlled_contact_names={"tb_1"},
        controlled_contact_min_center_distance=0.11,
    )
    assert result == command


def test_transport_contact_exception_never_applies_to_exploration_robot():
    command = Twist2D(0.10, 0.0)
    poses = {
        "tb_0": AgentSafetyPose(0.0, 0.0, 0.0),
        "tb_1": AgentSafetyPose(-0.13, 0.0, 0.0),
        "agent_0": AgentSafetyPose(0.18, 0.0, 0.0),
    }
    assert apply_teammate_hard_safety(
        command,
        "tb_0",
        poses,
        minimum_center_distance=0.22,
        allowed_controlled_contact_names={"tb_1"},
        controlled_contact_min_center_distance=0.11,
    ) == Twist2D()


def test_transport_contact_chain_still_rejects_deep_overlap():
    command = Twist2D(0.10, 0.0)
    poses = {
        "tb_0": AgentSafetyPose(0.0, 0.0, 0.0),
        "tb_1": AgentSafetyPose(0.09, 0.0, 0.0),
    }
    assert apply_teammate_hard_safety(
        command,
        "tb_0",
        poses,
        minimum_center_distance=0.22,
        allowed_controlled_contact_names={"tb_1"},
        controlled_contact_min_center_distance=0.11,
    ) == Twist2D()


def test_navigation_still_stops_motion_toward_close_teammate():
    command = Twist2D(0.10, -0.3)
    poses = {
        "tb_0": AgentSafetyPose(0.0, 0.0, 0.0),
        "tb_1": AgentSafetyPose(0.18, 0.0, 0.0),
    }
    assert apply_teammate_hard_safety(
        command, "tb_0", poses, minimum_center_distance=0.22,
    ) == Twist2D(0.0, -0.3)


def test_teammate_hard_safety_allows_motion_that_increases_separation():
    command = Twist2D(0.10, 0.2)
    poses = {
        "tb_0": AgentSafetyPose(0.0, 0.0, 0.0),
        "tb_1": AgentSafetyPose(-0.18, 0.0, 0.0),
    }
    assert apply_teammate_hard_safety(
        command, "tb_0", poses, minimum_center_distance=0.22,
    ) == command


def test_task_priority_safety_reserves_time_for_exploration_robot_to_yield():
    command = Twist2D(0.15, -0.2)
    poses = {
        "tb_0": AgentSafetyPose(0.0, 0.0, 0.0),
        "agent_0": AgentSafetyPose(0.35, 0.0, 0.0),
    }

    result = apply_external_priority_safety(
        command,
        "tb_0",
        poses,
        protected_names={"agent_0"},
        minimum_center_distance=0.26,
        reaction_horizon=1.0,
        maximum_other_speed=0.22,
    )

    assert result == Twist2D(0.0, -0.2)


def test_task_priority_safety_scales_forward_motion_from_distance_margin():
    command = Twist2D(0.15, 0.3)
    poses = {
        "tb_0": AgentSafetyPose(0.0, 0.0, 0.0),
        "agent_0": AgentSafetyPose(0.60, 0.0, 0.0),
    }

    result = apply_external_priority_safety(
        command,
        "tb_0",
        poses,
        protected_names={"agent_0"},
        minimum_center_distance=0.26,
        reaction_horizon=1.0,
        maximum_other_speed=0.22,
    )

    assert math.isclose(result.linear_x, 0.12)
    assert math.isclose(result.angular_z, command.angular_z)


def test_task_priority_safety_does_not_block_motion_away_from_external_robot():
    command = Twist2D(0.15, 0.2)
    poses = {
        "tb_0": AgentSafetyPose(0.0, 0.0, 0.0),
        "agent_0": AgentSafetyPose(-0.30, 0.0, 0.0),
    }

    assert apply_external_priority_safety(
        command,
        "tb_0",
        poses,
        protected_names={"agent_0"},
        minimum_center_distance=0.26,
        reaction_horizon=1.0,
        maximum_other_speed=0.22,
    ) == command


def test_stale_scan_stops_motion_instead_of_bypassing_safety():
    command = Twist2D(0.10, 0.4)
    assert apply_hard_scan_safety(
        command, [float("inf")], 0.0, 1.0,
        hard_stop_distance=0.20, motion_half_width=0.11,
        scan_age=0.31, scan_timeout=0.30,
    ) == ZERO_TWIST


def test_hard_safety_velocity_clamp_is_deterministic():
    assert clamp_command(Twist2D(0.4, -2.0), 0.22, 1.2) == Twist2D(0.22, -1.2)


def test_hard_safety_config_contains_only_shared_boundary_parameters():
    safety_path = os.path.join(PACKAGE_ROOT, "config", "response_safety.yaml")
    with open(safety_path, "r", encoding="utf-8") as fh:
        safety = yaml.safe_load(fh)
    assert safety["hard_stop_distance"] == 0.20
    assert safety["hard_stop_half_width"] == 0.11
    assert safety["teammate_min_center_distance"] == 0.26
    assert safety["scan_timeout"] == 0.30
    for removed in (
        "potential_field_safety_blend",
        "teammate_obstacle_reverse_velocity",
        "transport_task_bypass_safety",
    ):
        assert removed not in safety
