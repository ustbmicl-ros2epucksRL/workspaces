"""Transport heuristic policy adapted from the VMAS dribble controller."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .checkpoint_controller import global_action_to_twist
from .cmd_mux import Twist2D


@dataclass(frozen=True)
class TransportHeuristicConfig:
    lookahead: float = 0.0
    start_vel_dist_from_target_ratio: float = 0.5
    start_vel_behind_ratio: float = 0.5
    start_vel_mag: float = 1.0
    hit_vel_mag: float = 1.0
    package_radius: float = 0.15 / 2.0
    agent_radius: float = -0.02
    speed: float = 0.95
    u_range: float = 1.0


def transport_heuristic_global_action_from_observation(
    observation: Sequence[float],
    config: TransportHeuristicConfig | None = None,
) -> tuple[float, float]:
    """Compute the VMAS transport heuristic action from the 11D observation."""
    if len(observation) < 11:
        raise ValueError("transport observation must have at least 11 values")

    cfg = config or TransportHeuristicConfig()

    agent_pos = (
        float(observation[0]),
        float(observation[1]),
    )
    box_to_target = (
        float(observation[4]),
        float(observation[5]),
    )
    package_pos = (
        agent_pos[0] + float(observation[6]),
        agent_pos[1] + float(observation[7]),
    )
    goal_pos = (
        package_pos[0] - box_to_target[0],
        package_pos[1] - box_to_target[1],
    )

    control = _dribble_action(
        agent_pos=agent_pos,
        package_pos=package_pos,
        goal_pos=goal_pos,
        config=cfg,
    )
    action = (
        control[0] * cfg.speed * cfg.u_range,
        control[1] * cfg.speed * cfg.u_range,
    )
    return (
        _clamp(action[0], -cfg.u_range, cfg.u_range),
        _clamp(action[1], -cfg.u_range, cfg.u_range),
    )


def transport_heuristic_twist_from_observation(
    observation: Sequence[float],
    max_linear_velocity: float,
    max_angular_velocity: float,
    robot_yaw: float = 0.0,
    config: TransportHeuristicConfig | None = None,
) -> Twist2D:
    """Convert the VMAS transport heuristic action to a differential-drive command."""
    global_action = transport_heuristic_global_action_from_observation(
        observation,
        config=config,
    )
    return global_action_to_twist(
        global_action,
        robot_yaw=robot_yaw,
        max_linear_velocity=max_linear_velocity,
        max_angular_velocity=max_angular_velocity,
    )


def _dribble_action(
    agent_pos: tuple[float, float],
    package_pos: tuple[float, float],
    goal_pos: tuple[float, float],
    config: TransportHeuristicConfig,
) -> tuple[float, float]:
    package_disp = (goal_pos[0] - package_pos[0], goal_pos[1] - package_pos[1])
    ball_dist = _norm(package_disp)
    if ball_dist < 1e-6:
        return (0.0, 0.0)

    direction = (package_disp[0] / ball_dist, package_disp[1] / ball_dist)
    hit_pos = (
        package_pos[0] - direction[0] * (config.package_radius + config.agent_radius),
        package_pos[1] - direction[1] * (config.package_radius + config.agent_radius),
    )
    hit_vel = (direction[0] * config.hit_vel_mag, direction[1] * config.hit_vel_mag)
    start_vel = _start_velocity(
        pos=hit_pos,
        vel=hit_vel,
        start_pos=agent_pos,
        start_vel_mag=config.start_vel_mag * 2.0,
        dist_from_target_ratio=config.start_vel_dist_from_target_ratio,
        behind_ratio=config.start_vel_behind_ratio,
    )
    desired_pos = _hermite(agent_pos, hit_pos, start_vel, hit_vel, config.lookahead, deriv=0)
    desired_vel = _hermite(agent_pos, hit_pos, start_vel, hit_vel, config.lookahead, deriv=1)
    return (
        0.5 * (desired_pos[0] - agent_pos[0]) + 0.5 * desired_vel[0],
        0.5 * (desired_pos[1] - agent_pos[1]) + 0.5 * desired_vel[1],
    )


def _start_velocity(
    pos: tuple[float, float],
    vel: tuple[float, float],
    start_pos: tuple[float, float],
    start_vel_mag: float,
    dist_from_target_ratio: float,
    behind_ratio: float,
) -> tuple[float, float]:
    goal_disp = (pos[0] - start_pos[0], pos[1] - start_pos[1])
    goal_dist = _norm(goal_disp)
    if goal_dist < 1e-6:
        return (0.0, 0.0)

    vel_mag = _norm(vel)
    vel_dir = (vel[0] / vel_mag, vel[1] / vel_mag) if vel_mag > 1e-6 else (1.0, 0.0)
    goal_dir = (goal_disp[0] / goal_dist, goal_disp[1] / goal_dist)
    vel_normal = (-vel_dir[1], vel_dir[0])
    if _dot(goal_dir, vel_normal) > 0.0:
        vel_normal = (-vel_normal[0], -vel_normal[1])

    dist_behind_target = dist_from_target_ratio * goal_dist
    point_dir = (
        -vel_dir[0] * behind_ratio + vel_normal[0] * (1.0 - behind_ratio),
        -vel_dir[1] * behind_ratio + vel_normal[1] * (1.0 - behind_ratio),
    )
    target_pos = (
        pos[0] + point_dir[0] * dist_behind_target,
        pos[1] + point_dir[1] * dist_behind_target,
    )
    target_disp = (target_pos[0] - start_pos[0], target_pos[1] - start_pos[1])
    target_dist = _norm(target_disp)
    if target_dist < 1e-6:
        return (0.0, 0.0)
    return (
        target_disp[0] / target_dist * start_vel_mag,
        target_disp[1] / target_dist * start_vel_mag,
    )


def _hermite(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p0dot: tuple[float, float],
    p1dot: tuple[float, float],
    u: float,
    deriv: int,
) -> tuple[float, float]:
    u = float(u)
    if deriv == 0:
        basis = (
            2.0 * u**3 - 3.0 * u**2 + 1.0,
            -2.0 * u**3 + 3.0 * u**2,
            u**3 - 2.0 * u**2 + u,
            u**3 - u**2,
        )
    elif deriv == 1:
        basis = (
            6.0 * u**2 - 6.0 * u,
            -6.0 * u**2 + 6.0 * u,
            3.0 * u**2 - 4.0 * u + 1.0,
            3.0 * u**2 - 2.0 * u,
        )
    else:
        raise ValueError("only position and first derivative are supported")
    return (
        basis[0] * p0[0] + basis[1] * p1[0] + basis[2] * p0dot[0] + basis[3] * p1dot[0],
        basis[0] * p0[1] + basis[1] * p1[1] + basis[2] * p0dot[1] + basis[3] * p1dot[1],
    )


def _norm(vector: tuple[float, float]) -> float:
    return math.hypot(vector[0], vector[1])


def _dot(left: tuple[float, float], right: tuple[float, float]) -> float:
    return left[0] * right[0] + left[1] * right[1]


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
