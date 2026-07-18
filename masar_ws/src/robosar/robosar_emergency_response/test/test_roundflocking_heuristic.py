import math
import os
import sys

import pytest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from robosar_emergency_response.roundflocking_heuristic import (  # noqa: E402
    RoundflockingHeuristicConfig,
    adaptive_roundflocking_radius,
    coordinated_roundflocking_global_actions,
    roundflocking_heuristic_global_action_from_observation,
    roundflocking_heuristic_twist_from_observation,
)


def _clear_observation(x, y):
    return [0.0, 0.0, 0.0, 0.0, x, y] + [3.5] * 12


def _tangential_component(action, x, y):
    radius = math.hypot(x, y)
    return action[0] * (-y / radius) + action[1] * (x / radius)


def test_coordinated_roundflocking_uniform_ring_moves_clockwise_together():
    positions = {
        "a": (0.60, 0.0),
        "b": (0.0, 0.60),
        "c": (-0.60, 0.0),
        "d": (0.0, -0.60),
    }
    actions = coordinated_roundflocking_global_actions(
        {name: _clear_observation(*position) for name, position in positions.items()},
        {name: 0.0 for name in positions},
        config=RoundflockingHeuristicConfig(
            circle_radius=0.60, normal_offset=0.10, gain=10.0, u_range=1.0,
        ),
    )

    tangential = [
        _tangential_component(actions[name], *positions[name]) for name in positions
    ]
    assert all(value < -0.9 for value in tangential)
    assert max(tangential) - min(tangential) < 1e-9


def test_coordinated_roundflocking_opens_largest_gap_without_crossing_or_reversing():
    positions = {
        name: (0.60 * math.cos(angle), 0.60 * math.sin(angle))
        for name, angle in zip(("a", "b", "c", "d"), (0.0, 0.1, 0.2, 0.3))
    }
    actions = coordinated_roundflocking_global_actions(
        {name: _clear_observation(*position) for name, position in positions.items()},
        {name: 0.0 for name in positions},
        config=RoundflockingHeuristicConfig(
            circle_radius=0.60, normal_offset=0.10, gain=10.0, u_range=1.0,
        ),
    )
    tangential = {
        name: _tangential_component(actions[name], *positions[name])
        for name in positions
    }

    # The robot immediately clockwise of the large empty arc advances fastest;
    # the opposite boundary waits.  Circular order is preserved and no robot
    # is sent counter-clockwise through its neighbours.
    assert tangential["a"] < tangential["b"] <= 0.0
    assert tangential["a"] < tangential["c"] <= 0.0
    assert tangential["a"] < tangential["d"] + 1e-9
    assert tangential["d"] <= 1e-9


def test_coordinated_roundflocking_retains_radial_ring_recovery():
    positions = {
        "outer": (0.90, 0.0),
        "north": (0.0, 0.60),
        "west": (-0.60, 0.0),
        "south": (0.0, -0.60),
    }
    actions = coordinated_roundflocking_global_actions(
        {name: _clear_observation(*position) for name, position in positions.items()},
        {name: 0.0 for name in positions},
        config=RoundflockingHeuristicConfig(
            circle_radius=0.60, normal_offset=0.10, gain=2.0, u_range=1.0,
        ),
    )

    assert actions["outer"][0] < 0.0


def test_roundflocking_heuristic_moves_from_outer_radius_toward_circle():
    config = RoundflockingHeuristicConfig(
        circle_radius=0.6,
        normal_offset=0.1,
        gain=1.0,
        u_range=10.0,
    )

    action = roundflocking_heuristic_global_action_from_observation(
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        config=config,
    )

    assert math.isclose(action[0], -0.4)
    assert math.isclose(action[1], -0.1)


def test_roundflocking_heuristic_uses_lidar_object_avoidance():
    config = RoundflockingHeuristicConfig(
        circle_radius=0.5,
        normal_offset=0.0,
        object_visible_range=0.45,
        object_avoid_distance=0.18,
        gain=1.0,
        u_range=10.0,
    )

    action = roundflocking_heuristic_global_action_from_observation(
        [0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.20, 1.0, 1.0, 1.0],
        config=config,
    )

    assert action[0] < 0.0
    assert action[1] < 0.0


def test_roundflocking_lidar_repulsion_is_rotated_from_body_to_world():
    config = RoundflockingHeuristicConfig(
        circle_radius=0.5,
        normal_offset=0.0,
        object_visible_range=0.45,
        object_avoid_distance=0.18,
        gain=1.0,
        u_range=10.0,
    )
    action = roundflocking_heuristic_global_action_from_observation(
        [0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.20, 1.0, 1.0, 1.0],
        config=config,
        robot_yaw=math.pi / 2.0,
    )

    assert action[0] > 0.0
    assert action[1] < 0.0


def test_roundflocking_heuristic_rejects_short_observation():
    with pytest.raises(ValueError, match="at least 6 values"):
        roundflocking_heuristic_global_action_from_observation([0.0] * 5)


def test_roundflocking_radius_shrinks_for_outer_or_tangential_obstacle():
    config = RoundflockingHeuristicConfig(
        circle_radius=0.60,
        minimum_circle_radius=0.38,
        object_visible_range=0.45,
    )

    outer = adaptive_roundflocking_radius(
        current_pos=(0.60, 0.0),
        obstacle_direction=(1.0, 0.0),
        obstacle_range=0.20,
        config=config,
    )
    tangent = adaptive_roundflocking_radius(
        current_pos=(0.60, 0.0),
        obstacle_direction=(0.0, 1.0),
        obstacle_range=0.20,
        config=config,
    )

    assert config.minimum_circle_radius < outer < config.circle_radius
    assert math.isclose(tangent, outer)


def test_roundflocking_radius_does_not_shrink_toward_target_side_obstacle():
    config = RoundflockingHeuristicConfig(
        circle_radius=0.60,
        minimum_circle_radius=0.38,
        object_visible_range=0.45,
    )
    radius = adaptive_roundflocking_radius(
        current_pos=(0.60, 0.0),
        obstacle_direction=(-1.0, 0.0),
        obstacle_range=0.20,
        config=config,
    )

    assert math.isclose(radius, config.circle_radius)


def test_roundflocking_twist_includes_orbit_curvature_feedforward():
    config = RoundflockingHeuristicConfig(
        circle_radius=0.60,
        minimum_circle_radius=0.38,
        normal_offset=0.10,
        gain=10.0,
        u_range=1.0,
    )
    twist = roundflocking_heuristic_twist_from_observation(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.60] + [1.0] * 12,
        robot_yaw=-math.pi / 2.0,
        max_linear_velocity=0.22,
        max_angular_velocity=0.8,
        config=config,
    )

    assert twist.linear_x > 0.20
    assert twist.angular_z < -0.30
