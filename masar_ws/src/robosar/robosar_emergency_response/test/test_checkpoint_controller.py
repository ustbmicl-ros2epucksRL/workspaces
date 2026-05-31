import math
import os
import sys


PACKAGE_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from robosar_emergency_response.checkpoint_controller import (  # noqa: E402
    MatrixPose,
    RobotInfo,
    SceneObjectInfo,
    build_roundflocking_observation,
    build_transport_observation,
    direct_transport_output_to_global_action,
    differential_twist_from_omni_output,
    direct_twist_from_network_output,
    signed_outputs_from_neat,
)
from robosar_emergency_response.cmd_mux import (  # noqa: E402
    PotentialFieldDecision,
    Twist2D,
    apply_soft_potential_field_safety,
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


def test_soft_potential_field_safety_limits_turnaround_but_keeps_close_stop():
    command = Twist2D(0.15, 0.0)
    decision = PotentialFieldDecision(
        avoid_needed=True,
        command=Twist2D(-0.12, 1.0),
        min_range=0.12,
    )

    soft = apply_soft_potential_field_safety(
        command,
        decision,
        blend=0.4,
        max_angular_delta=0.5,
        min_forward_scale=0.35,
        critical_stop_distance=0.10,
    )
    assert soft.linear_x > 0.0
    assert math.isclose(soft.angular_z, 0.2)

    critical = apply_soft_potential_field_safety(
        command,
        PotentialFieldDecision(True, Twist2D(-0.12, 1.0), min_range=0.09),
        critical_stop_distance=0.10,
    )
    assert critical.linear_x <= 0.0

    victim_clearance_stop = apply_soft_potential_field_safety(
        command,
        PotentialFieldDecision(True, Twist2D(-0.12, 1.0), min_range=0.20),
        critical_stop_distance=0.24,
    )
    assert victim_clearance_stop.linear_x <= 0.0
