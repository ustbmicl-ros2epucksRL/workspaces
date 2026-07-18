"""VMAS-style heuristic controller for roundflocking tasks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from .checkpoint_controller import global_action_to_twist, rotate_body_to_global
from .cmd_mux import Twist2D


@dataclass(frozen=True)
class RoundflockingHeuristicConfig:
    circle_radius: float = 0.30
    minimum_circle_radius: float = 0.22
    normal_offset: float = 0.10
    object_visible_range: float = 0.45
    object_avoid_distance: float = 0.18
    gain: float = 10.0
    u_range: float = 1.0
    speed: float = 1.0


def coordinated_roundflocking_global_actions(
    observations: Mapping[str, Sequence[float]],
    robot_yaws: Mapping[str, float],
    config: RoundflockingHeuristicConfig | None = None,
    world_scale: float = 1.0,
) -> dict[str, tuple[float, float]]:
    """Return collision-resolving, evenly spaced clockwise orbit intents.

    Each robot follows the same radial/adaptive-radius field as the standalone
    heuristic.  Tangential speed is coordinated by the free angular gap in
    front of that robot (clockwise cyclic pursuit): a robot with an empty arc
    advances, while one following a close teammate slows down.  Circular order
    is therefore preserved, clustered starts spread without crossing chords,
    and no counter-clockwise recovery/reverse command is introduced.
    """
    if not observations:
        return {}
    cfg = config or RoundflockingHeuristicConfig()
    missing_yaws = set(observations) - set(robot_yaws)
    if missing_yaws:
        raise ValueError(
            "missing robot yaw for {}".format(", ".join(sorted(missing_yaws)))
        )

    scale = max(float(world_scale), 1e-6)
    contexts = {}
    for name, observation in observations.items():
        if len(observation) < 6:
            raise ValueError("roundflocking observation must have at least 6 values")
        yaw = float(robot_yaws[name])
        target_rel_body = (
            float(observation[4]) * scale,
            float(observation[5]) * scale,
        )
        current_pos = rotate_body_to_global(*target_rel_body, yaw)
        object_vec, closest_range, urgency = _nearest_obstacle_context(
            observation[6:], yaw, cfg,
        )
        desired_radius = adaptive_roundflocking_radius(
            current_pos=current_pos,
            obstacle_direction=object_vec,
            obstacle_range=closest_range,
            config=cfg,
        )
        contexts[name] = (
            current_pos, object_vec, urgency, desired_radius,
            math.atan2(current_pos[1], current_pos[0]),
        )

    ordered = sorted(contexts, key=lambda name: contexts[name][4])
    desired_gap = 2.0 * math.pi / float(len(ordered))
    nominal_tangent = min(
        abs(float(cfg.u_range)),
        abs(float(cfg.gain) * float(cfg.normal_offset)),
    )
    actions = {}
    for index, name in enumerate(ordered):
        current_pos, object_vec, urgency, desired_radius, angle = contexts[name]
        previous_angle = contexts[ordered[index - 1]][4]
        clockwise_gap = (angle - previous_angle) % (2.0 * math.pi)
        gap_speed_scale = min(1.0, clockwise_gap / max(desired_gap, 1e-6))

        radius = math.hypot(current_pos[0], current_pos[1])
        if radius <= 1e-9:
            outward = (1.0, 0.0)
        else:
            outward = (current_pos[0] / radius, current_pos[1] / radius)
        tangent_ccw = (-outward[1], outward[0])
        radial_action = _clamp(
            (desired_radius - radius) * cfg.gain, cfg.u_range,
        )
        tangent_action = -nominal_tangent * gap_speed_scale
        action_x = radial_action * outward[0] + tangent_action * tangent_ccw[0]
        action_y = radial_action * outward[1] + tangent_action * tangent_ccw[1]
        if object_vec is not None:
            avoidance = cfg.object_avoid_distance * urgency * cfg.gain
            action_x -= object_vec[0] * avoidance
            action_y -= object_vec[1] * avoidance
        actions[name] = (
            _clamp(action_x, cfg.u_range),
            _clamp(action_y, cfg.u_range),
        )
    return actions


def coordinated_roundflocking_twists(
    observations: Mapping[str, Sequence[float]],
    robot_yaws: Mapping[str, float],
    max_linear_velocity: float,
    max_angular_velocity: float,
    config: RoundflockingHeuristicConfig | None = None,
    world_scale: float = 1.0,
) -> dict[str, Twist2D]:
    """Adapt coordinated holonomic intents to forward-only differential drive."""
    cfg = config or RoundflockingHeuristicConfig()
    actions = coordinated_roundflocking_global_actions(
        observations, robot_yaws, config=cfg, world_scale=world_scale,
    )
    scale = max(float(world_scale), 1e-6)
    twists = {}
    for name, action in actions.items():
        twist = global_action_to_twist(
            action,
            robot_yaw=float(robot_yaws[name]),
            max_linear_velocity=max_linear_velocity,
            max_angular_velocity=max_angular_velocity,
            speed_scale=cfg.speed,
        )
        observation = observations[name]
        relative = rotate_body_to_global(
            float(observation[4]) * scale,
            float(observation[5]) * scale,
            float(robot_yaws[name]),
        )
        radius = max(
            max(1e-3, float(cfg.minimum_circle_radius)),
            min(max(float(cfg.circle_radius), 1e-3), math.hypot(*relative)),
        )
        cross = relative[0] * action[1] - relative[1] * action[0]
        curvature_sign = -1.0 if cross < -1e-9 else (1.0 if cross > 1e-9 else 0.0)
        angular_limit = abs(float(max_angular_velocity))
        angular = twist.angular_z + curvature_sign * twist.linear_x / radius
        twists[name] = Twist2D(
            twist.linear_x,
            max(-angular_limit, min(angular_limit, angular)),
        )
    return twists


def _nearest_obstacle_context(
    lidar: Sequence[float],
    robot_yaw: float,
    config: RoundflockingHeuristicConfig,
) -> tuple[tuple[float, float] | None, float, float]:
    if not lidar or min(lidar) >= config.object_visible_range:
        return None, math.inf, 0.0
    closest_index = min(range(len(lidar)), key=lambda index: lidar[index])
    closest_range = float(lidar[closest_index])
    object_dir = (closest_index + 0.5) / float(len(lidar)) * 2.0 * math.pi
    object_vec = rotate_body_to_global(
        math.cos(object_dir), math.sin(object_dir), robot_yaw,
    )
    urgency = _clamp(
        (config.object_visible_range - closest_range)
        / max(config.object_visible_range, 1e-6),
        1.0,
    )
    return object_vec, closest_range, urgency


def roundflocking_heuristic_twist_from_observation(
    observation: Sequence[float],
    robot_yaw: float,
    max_linear_velocity: float,
    max_angular_velocity: float,
    config: RoundflockingHeuristicConfig | None = None,
    world_scale: float = 1.0,
) -> Twist2D:
    cfg = config or RoundflockingHeuristicConfig()
    global_action = roundflocking_heuristic_global_action_from_observation(
        observation,
        config=cfg,
        robot_yaw=robot_yaw,
        world_scale=world_scale,
    )
    twist = global_action_to_twist(
        global_action,
        robot_yaw=robot_yaw,
        max_linear_velocity=max_linear_velocity,
        max_angular_velocity=max_angular_velocity,
        speed_scale=cfg.speed,
    )
    # The vector field is clockwise. A differential robot needs the kinematic
    # curvature feedforward omega=-v/r even when its heading is already exactly
    # tangent; without it, heading lag is the only source of turning and the
    # cosine projection collapses forward speed.
    scale = max(float(world_scale), 1e-6)
    radius = math.hypot(float(observation[4]) * scale, float(observation[5]) * scale)
    radius = max(
        max(1e-3, float(cfg.minimum_circle_radius)),
        min(max(float(cfg.circle_radius), 1e-3), radius),
    )
    angular = twist.angular_z - twist.linear_x / radius
    angular_limit = abs(float(max_angular_velocity))
    return Twist2D(
        twist.linear_x,
        max(-angular_limit, min(angular_limit, angular)),
    )


def roundflocking_heuristic_global_action_from_observation(
    observation: Sequence[float],
    config: RoundflockingHeuristicConfig | None = None,
    robot_yaw: float = 0.0,
    world_scale: float = 1.0,
) -> tuple[float, float]:
    """Return the VMAS flocking HeuristicPolicy action in global coordinates."""
    if len(observation) < 6:
        raise ValueError("roundflocking observation must have at least 6 values")

    cfg = config or RoundflockingHeuristicConfig()
    scale = max(float(world_scale), 1e-6)
    target_rel_body = (
        float(observation[4]) * scale,
        float(observation[5]) * scale,
    )
    current_pos = rotate_body_to_global(target_rel_body[0], target_rel_body[1], robot_yaw)
    lidar = [float(value) for value in observation[6:]]

    object_vec = None
    closest_range = math.inf
    urgency = 0.0
    if lidar and min(lidar) < cfg.object_visible_range:
        closest_index = min(range(len(lidar)), key=lambda index: lidar[index])
        closest_range = min(lidar)
        # build_roundflocking_observation samples each LiDAR sector at its
        # midpoint. That direction is robot-local and must be rotated before
        # it can be combined with the global circle-tracking action.
        object_dir = (closest_index + 0.5) / float(len(lidar)) * 2.0 * math.pi
        object_body = (math.cos(object_dir), math.sin(object_dir))
        object_vec = rotate_body_to_global(
            object_body[0], object_body[1], robot_yaw,
        )
        urgency = _clamp(
            (cfg.object_visible_range - closest_range)
            / max(cfg.object_visible_range, 1e-6),
            1.0,
        )

    circle_radius = adaptive_roundflocking_radius(
        current_pos=current_pos,
        obstacle_direction=object_vec,
        obstacle_range=closest_range,
        config=cfg,
    )
    des_pos = _desired_circle_motion_position(
        current_pos, cfg, circle_radius=circle_radius,
    )
    if object_vec is not None:
        des_pos = (
            des_pos[0] - object_vec[0] * cfg.object_avoid_distance * urgency,
            des_pos[1] - object_vec[1] * cfg.object_avoid_distance * urgency,
        )

    return (
        _clamp((des_pos[0] - current_pos[0]) * cfg.gain, cfg.u_range),
        _clamp((des_pos[1] - current_pos[1]) * cfg.gain, cfg.u_range),
    )


def _desired_circle_motion_position(
    current_pos: tuple[float, float],
    config: RoundflockingHeuristicConfig,
    circle_radius: float | None = None,
) -> tuple[float, float]:
    desired_radius = (
        config.circle_radius if circle_radius is None else float(circle_radius)
    )
    norm = math.hypot(current_pos[0], current_pos[1])
    if norm <= 1e-9:
        closest_point = (desired_radius, 0.0)
    else:
        closest_point = (
            current_pos[0] / norm * desired_radius,
            current_pos[1] / norm * desired_radius,
        )

    normal = (closest_point[1], -closest_point[0])
    normal_norm = math.hypot(normal[0], normal[1])
    if normal_norm <= 1e-9:
        return closest_point
    return (
        closest_point[0] + normal[0] / normal_norm * config.normal_offset,
        closest_point[1] + normal[1] / normal_norm * config.normal_offset,
    )


def adaptive_roundflocking_radius(
    current_pos: tuple[float, float],
    obstacle_direction: tuple[float, float] | None,
    obstacle_range: float,
    config: RoundflockingHeuristicConfig | None = None,
) -> float:
    """Choose a locally feasible orbit radius without shrinking toward the target."""
    cfg = config or RoundflockingHeuristicConfig()
    nominal = max(0.0, float(cfg.circle_radius))
    minimum = max(0.0, min(nominal, float(cfg.minimum_circle_radius)))
    if obstacle_direction is None or not math.isfinite(obstacle_range):
        return nominal
    norm = math.hypot(current_pos[0], current_pos[1])
    if norm <= 1e-9:
        return nominal
    outward = (current_pos[0] / norm, current_pos[1] / norm)
    # A negative dot product means the return lies on the target side. In that
    # case shrinking would reduce target clearance, so keep the nominal ring.
    if outward[0] * obstacle_direction[0] + outward[1] * obstacle_direction[1] < 0.0:
        return nominal
    urgency = _clamp(
        (cfg.object_visible_range - float(obstacle_range))
        / max(float(cfg.object_visible_range), 1e-6),
        1.0,
    )
    return nominal - (nominal - minimum) * max(0.0, urgency)


def _clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, float(value)))
