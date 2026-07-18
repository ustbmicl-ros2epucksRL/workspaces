#  Copyright (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  LICENSE file in the root directory of this source tree.
"""Partially observable navigation with coordinated crossing traffic.

This scenario intentionally inherits the reward and termination semantics of
``navigation.py``.  In particular, the goal landmark keeps VMAS's default
radius while ``done`` uses the agent radius.  The only task changes are the
episode layout, inertial drag, and whether instantaneous self velocity is visible.

Agents start on a perturbed circle and receive a near-opposite goal.  Their
shortest paths cross in a central conflict region.  The goal is locally
occluded only inside that region, and velocity can be hidden, making route
continuation and collision recovery short-term state-estimation problems while
keeping the original observation dimension.
"""

import math

import torch
from torch import Tensor

from vmas.scenarios.navigation import Scenario as NavigationScenario
from vmas.simulator.heuristic_policy import BaseHeuristicPolicy


class Scenario(NavigationScenario):
    """Navigation variant whose direct routes create coordinated crossings."""

    def make_world(self, batch_dim: int, device: torch.device, **kwargs):
        self.crossing_probability = float(kwargs.pop("crossing_probability", 1.0))
        self.crossing_radius = float(kwargs.pop("crossing_radius", 0.8))
        self.crossing_angle_jitter = float(kwargs.pop("crossing_angle_jitter", 0.08))
        self.crossing_radius_jitter = float(kwargs.pop("crossing_radius_jitter", 0.05))
        self.crossing_goal_angle_offset = float(
            kwargs.pop("crossing_goal_angle_offset", 0.0)
        )
        self.randomize_goal_angle_direction = bool(
            kwargs.pop("randomize_goal_angle_direction", False)
        )
        self.goal_occlusion_radius = float(kwargs.pop("goal_occlusion_radius", 0.0))
        self.observe_velocity = bool(kwargs.pop("observe_velocity", False))
        self.world_drag = float(kwargs.pop("world_drag", 0.25))

        if not 0.0 <= self.crossing_probability <= 1.0:
            raise ValueError("crossing_probability must be in [0, 1]")
        if self.crossing_radius <= 0.0:
            raise ValueError("crossing_radius must be positive")
        if self.crossing_angle_jitter < 0.0:
            raise ValueError("crossing_angle_jitter must be non-negative")
        if self.crossing_radius_jitter < 0.0:
            raise ValueError("crossing_radius_jitter must be non-negative")
        if self.goal_occlusion_radius < 0.0:
            raise ValueError("goal_occlusion_radius must be non-negative")
        if not 0.0 <= self.world_drag < 1.0:
            raise ValueError("world_drag must be in [0, 1)")

        world = super().make_world(batch_dim, device, **kwargs)
        # Expose drag as a task parameter while retaining Navigation's default
        # value in the recommended configuration.
        world._drag = self.world_drag
        if self.agents_with_same_goal != 1 or self.split_goals:
            raise ValueError(
                "coordinated_navigation requires independent goals: "
                "agents_with_same_goal=1 and split_goals=False"
            )
        if self.observe_all_goals:
            raise ValueError(
                "coordinated_navigation uses local goal occlusion and therefore "
                "requires observe_all_goals=False"
            )
        return world

    def reset_world_at(self, env_index: int = None):
        # Keep the base reset as the fallback for the non-crossing fraction and
        # to initialise all reward-shaping buffers exactly as navigation does.
        super().reset_world_at(env_index)

        count = self.world.batch_dim if env_index is None else 1
        device = self.world.device
        crossing_mask = torch.rand(count, device=device) < self.crossing_probability
        if not bool(crossing_mask.any()):
            return

        base_angles = torch.arange(
            self.n_agents, device=device, dtype=torch.float32
        ) * (2.0 * math.pi / self.n_agents)
        rotation = torch.rand(count, 1, device=device) * (2.0 * math.pi)
        angle_noise = (
            torch.rand(count, self.n_agents, device=device) * 2.0 - 1.0
        ) * self.crossing_angle_jitter
        angles = base_angles.unsqueeze(0) + rotation + angle_noise

        radius_noise = (
            torch.rand(count, self.n_agents, device=device) * 2.0 - 1.0
        ) * self.crossing_radius_jitter
        radii = (self.crossing_radius + radius_noise).clamp_min(
            self.agent_radius + 0.05
        )
        starts = torch.stack(
            (radii * torch.cos(angles), radii * torch.sin(angles)), dim=-1
        )
        if self.randomize_goal_angle_direction:
            goal_direction = torch.where(
                torch.rand(count, 1, device=device) < 0.5,
                -torch.ones(count, 1, device=device),
                torch.ones(count, 1, device=device),
            )
        else:
            goal_direction = torch.ones(count, 1, device=device)
        goal_angles = (
            angles + math.pi + goal_direction * self.crossing_goal_angle_offset
        )
        goals = torch.stack(
            (radii * torch.cos(goal_angles), radii * torch.sin(goal_angles)), dim=-1
        )

        for agent_index, agent in enumerate(self.world.agents):
            if env_index is None:
                mask = crossing_mask.unsqueeze(-1)
                start = torch.where(mask, starts[:, agent_index], agent.state.pos)
                goal = torch.where(mask, goals[:, agent_index], agent.goal.state.pos)
                agent.set_pos(start, batch_index=None)
                agent.goal.set_pos(goal, batch_index=None)
                agent.pos_shaping = (
                    torch.linalg.vector_norm(start - goal, dim=-1)
                    * self.pos_shaping_factor
                )
            elif bool(crossing_mask[0]):
                start = starts[0, agent_index]
                goal = goals[0, agent_index]
                agent.set_pos(start, batch_index=env_index)
                agent.goal.set_pos(goal, batch_index=env_index)
                agent.pos_shaping[env_index] = (
                    torch.linalg.vector_norm(start - goal) * self.pos_shaping_factor
                )

    def observation(self, agent):
        observation = super().observation(agent)
        # Preserve navigation's observation ordering and dimensionality.  This
        # prevents architecture-size changes from confounding FF/REC comparisons.
        observation = observation.clone()
        if not self.observe_velocity:
            observation[..., 2:4] = 0.0

        if self.goal_occlusion_radius > 0.0:
            in_conflict_region = (
                torch.linalg.vector_norm(agent.state.pos, dim=-1)
                < self.goal_occlusion_radius
            )
            observation[in_conflict_region, 4:6] = 0.0
        return observation


class HeuristicPolicy(BaseHeuristicPolicy):
    """Decentralized observation-only controller inspired by Navigation.

    Goal seeking and stopping follow Navigation's controller design. Collision
    avoidance uses only the acting agent's lidar, and hidden motion is estimated
    from that agent's own position history. No action depends on teammate
    observations, agent indices, world state, or the reset layout.
    """

    def __init__(
        self,
        goal_gain: float = 1.5,
        motion_damping: float = 20.0,
        repulsion_gain: float = 3.2,
        sidestep_gain: float = 1.0,
        yield_gain: float = 0.8,
        lidar_range: float = 0.35,
        goal_stop_radius: float = 0.045,
        motion_smoothing: float = 0.5,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.goal_gain = float(goal_gain)
        self.motion_damping = float(motion_damping)
        self.repulsion_gain = float(repulsion_gain)
        self.sidestep_gain = float(sidestep_gain)
        self.yield_gain = float(yield_gain)
        self.lidar_range = float(lidar_range)
        self.goal_stop_radius = float(goal_stop_radius)
        self.motion_smoothing = float(motion_smoothing)
        if self.lidar_range <= 0.0:
            raise ValueError("lidar_range must be positive")
        if not 0.0 <= self.motion_smoothing < 1.0:
            raise ValueError("motion_smoothing must be in [0, 1)")

        self._remembered_goals: list[Tensor] | None = None
        self._goal_memory_valid: list[Tensor] | None = None
        self._previous_positions: list[Tensor] | None = None
        self._estimated_motions: list[Tensor] | None = None

    def reset(self) -> None:
        self._remembered_goals = None
        self._goal_memory_valid = None
        self._previous_positions = None
        self._estimated_motions = None

    def compute_actions(self, observations, u_ranges):
        if len(observations) == 0:
            return []
        if self._memory_needs_reset(observations):
            self._initialize_memory(observations)

        actions = []
        for index, observation in enumerate(observations):
            position = observation[:, :2]
            measured_motion = position - self._previous_positions[index]
            smoothing = self.motion_smoothing
            self._estimated_motions[index] = (
                smoothing * self._estimated_motions[index]
                + (1.0 - smoothing) * measured_motion
            )
            self._previous_positions[index] = position.clone()

            goal_offset = observation[:, 4:6]
            goal_visible = goal_offset.abs().sum(dim=-1, keepdim=True) > 1e-8
            observed_goal = position - goal_offset
            self._remembered_goals[index] = torch.where(
                goal_visible,
                observed_goal,
                self._remembered_goals[index],
            )
            self._goal_memory_valid[index] |= goal_visible
            to_goal = torch.where(
                self._goal_memory_valid[index],
                self._remembered_goals[index] - position,
                torch.zeros_like(position),
            )
            action = self._local_action(
                observation,
                to_goal,
                self._estimated_motions[index],
            )
            actions.append(self._clamp_action(action, u_ranges[index]))
        return actions

    def compute_action(self, observation: Tensor, u_range) -> Tensor:
        """Stateless fallback using only the current goal and lidar readings."""
        action = self._local_action(
            observation,
            -observation[:, 4:6],
            torch.zeros_like(observation[:, :2]),
        )
        return self._clamp_action(action, u_range)

    def _local_action(
        self,
        observation: Tensor,
        to_goal: Tensor,
        estimated_motion: Tensor,
    ) -> Tensor:
        goal_distance = torch.linalg.vector_norm(
            to_goal, dim=-1, keepdim=True
        ).clamp_min(1e-6)
        goal_direction = to_goal / goal_distance
        avoidance, obstacle_pressure = self._lidar_avoidance(
            observation, goal_direction
        )
        goal_action = self.goal_gain * to_goal
        goal_action *= (1.0 - self.yield_gain * obstacle_pressure).clamp_min(0.0)
        action = goal_action - self.motion_damping * estimated_motion + avoidance
        return torch.where(
            goal_distance < self.goal_stop_radius,
            -self.motion_damping * estimated_motion,
            action,
        )

    def _lidar_avoidance(
        self, observation: Tensor, goal_direction: Tensor
    ) -> tuple[Tensor, Tensor]:
        lidar = observation[:, 6:]
        if lidar.shape[-1] == 0:
            return torch.zeros_like(goal_direction), torch.zeros_like(
                goal_direction[:, :1]
            )

        n_rays = lidar.shape[-1]
        angles = torch.arange(
            n_rays, device=observation.device, dtype=observation.dtype
        ) * (2.0 * math.pi / n_rays)
        toward_obstacle = torch.stack(
            (torch.cos(angles), torch.sin(angles)), dim=-1
        ).unsqueeze(0)
        away = -toward_obstacle
        proximity = (lidar / self.lidar_range).clamp(0.0, 1.0).unsqueeze(-1)
        weight = proximity.square()
        repulsion = (away * weight).sum(dim=1) * self.repulsion_gain

        tangent = torch.stack([-away[..., 1], away[..., 0]], dim=-1)
        tangent = tangent.expand(observation.shape[0], -1, -1)
        tangent_sign = torch.where(
            (tangent * goal_direction.unsqueeze(1)).sum(dim=-1, keepdim=True) >= 0.0,
            1.0,
            -1.0,
        )
        sidestep = (tangent * tangent_sign * weight).sum(dim=1) * self.sidestep_gain
        obstacle_pressure = weight.max(dim=1).values
        return repulsion + sidestep, obstacle_pressure

    def _initialize_memory(self, observations) -> None:
        self._remembered_goals = [
            torch.zeros_like(observation[:, :2]) for observation in observations
        ]
        self._goal_memory_valid = [
            torch.zeros(
                observation.shape[0],
                1,
                dtype=torch.bool,
                device=observation.device,
            )
            for observation in observations
        ]
        self._previous_positions = [
            observation[:, :2].clone() for observation in observations
        ]
        self._estimated_motions = [
            torch.zeros_like(observation[:, :2]) for observation in observations
        ]

    def _memory_needs_reset(self, observations) -> bool:
        memories = (
            self._remembered_goals,
            self._goal_memory_valid,
            self._previous_positions,
            self._estimated_motions,
        )
        if any(memory is None for memory in memories):
            return True
        if any(len(memory) != len(observations) for memory in memories):
            return True
        return any(
            remembered.shape != observation[:, :2].shape
            or remembered.device != observation.device
            or memory_valid.shape != observation[:, :1].shape
            or memory_valid.device != observation.device
            or previous_position.shape != observation[:, :2].shape
            or previous_position.device != observation.device
            or estimated_motion.shape != observation[:, :2].shape
            or estimated_motion.device != observation.device
            for remembered, memory_valid, previous_position, estimated_motion, observation in zip(
                self._remembered_goals,
                self._goal_memory_valid,
                self._previous_positions,
                self._estimated_motions,
                observations,
            )
        )

    @staticmethod
    def _clamp_action(action: Tensor, u_range) -> Tensor:
        if torch.is_tensor(u_range):
            limit = u_range.to(device=action.device, dtype=action.dtype)
            return torch.maximum(torch.minimum(action, limit), -limit)
        return torch.clamp(action, -float(u_range), float(u_range))
