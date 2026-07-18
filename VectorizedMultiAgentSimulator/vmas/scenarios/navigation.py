#  Copyright (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  LICENSE file in the root directory of this source tree.
import typing
from typing import Callable, Dict, List

import torch
from torch import Tensor

from vmas import render_interactively
from vmas.simulator.core import Agent, Entity, Landmark, Sphere, World
from vmas.simulator.heuristic_policy import BaseHeuristicPolicy
from vmas.simulator.scenario import BaseScenario
from vmas.simulator.sensors import Lidar
from vmas.simulator.utils import Color, ScenarioUtils, X, Y

if typing.TYPE_CHECKING:
    from vmas.simulator.rendering import Geom


class Scenario(BaseScenario):
    def make_world(self, batch_dim: int, device: torch.device, **kwargs):
        # 场景/环境参数入口：可通过 make_env(..., **kwargs) 覆盖默认配置。
        self.plot_grid = False
        self.n_agents = kwargs.pop("n_agents", 4)  # 智能体数量
        self.collisions = kwargs.pop("collisions", True)  # 是否启用碰撞和激光雷达

        self.world_spawning_x = kwargs.pop(
            "world_spawning_x", 1
        )  # 生成初始位置的X轴半径
        self.world_spawning_y = kwargs.pop(
            "world_spawning_y", 1
        )  # 生成初始位置的Y轴半径
        self.enforce_bounds = kwargs.pop(
            "enforce_bounds", False
        )  # 是否将世界边界限制在上述范围内（否则无限大）

        self.agents_with_same_goal = kwargs.pop(
            "agents_with_same_goal", 1
        )  # 前多少个智能体共享同一目标
        self.split_goals = kwargs.pop(
            "split_goals", False
        )  # 是否按队伍拆分目标（需要偶数智能体且各半队伍共用同一目标）
        self.observe_all_goals = kwargs.pop(
            "observe_all_goals", False
        )  # 观测中是否包含所有目标的相对位置（True 时为异质/协作行为）

        self.lidar_range = kwargs.pop("lidar_range", 0.35)  # 激光雷达最远测距
        self.agent_radius = kwargs.pop("agent_radius", 0.1)  # 智能体半径
        self.comms_range = kwargs.pop("comms_range", 0)  # 通讯/连线显示的距离阈值
        self.n_lidar_rays = kwargs.pop("n_lidar_rays", 12)  # 激光雷达射线数量

        self.shared_rew = kwargs.pop("shared_rew", True)  # 奖励是否在所有智能体间共享
        self.pos_shaping_factor = kwargs.pop(
            "pos_shaping_factor", 1
        )  # 位置 shaping 奖励的距离缩放系数
        self.final_reward = kwargs.pop(
            "final_reward", 0.04
        )  # 全部智能体到达目标后的终止奖励

        self.agent_collision_penalty = kwargs.pop(
            "agent_collision_penalty", -1
        )  # 碰撞时的惩罚（负值）
        ScenarioUtils.check_kwargs_consumed(kwargs)

        self.min_distance_between_entities = self.agent_radius * 2 + 0.05
        self.min_collision_distance = 0.005

        if self.enforce_bounds:
            self.x_semidim = self.world_spawning_x
            self.y_semidim = self.world_spawning_y
        else:
            self.x_semidim = None
            self.y_semidim = None

        assert 1 <= self.agents_with_same_goal <= self.n_agents
        if self.agents_with_same_goal > 1:
            assert (
                not self.collisions
            ), "If agents share goals they cannot be collidables"
        # agents_with_same_goal == n_agents: all agent same goal
        # agents_with_same_goal = x: the first x agents share the goal
        # agents_with_same_goal = 1: all independent goals
        if self.split_goals:
            assert (
                self.n_agents % 2 == 0
                and self.agents_with_same_goal == self.n_agents // 2
            ), "Splitting the goals is allowed when the agents are even and half the team has the same goal"

        # Make world
        # 环境级别参数（如 dt、substeps、摩擦/阻尼、边界）在这里传入 World(...).
        world = World(
            batch_dim,
            device,
            substeps=2,
            x_semidim=self.x_semidim,
            y_semidim=self.y_semidim,
        )

        known_colors = [
            (0.22, 0.49, 0.72),
            (1.00, 0.50, 0),
            (0.30, 0.69, 0.29),
            (0.97, 0.51, 0.75),
            (0.60, 0.31, 0.64),
            (0.89, 0.10, 0.11),
            (0.87, 0.87, 0),
        ]
        colors = torch.randn(
            (max(self.n_agents - len(known_colors), 0), 3), device=device
        )
        entity_filter_agents: Callable[[Entity], bool] = lambda e: isinstance(e, Agent)

        # Add agents
        # 智能体参数（形状/半径、传感器、动力学/动作范围等）在这里配置。
        for i in range(self.n_agents):
            color = (
                known_colors[i]
                if i < len(known_colors)
                else colors[i - len(known_colors)]
            )

            # 约束：所有智能体使用相同的动作范围与缩放系数
            # 如需配置动力学/动作范围/摩擦等，可在 Agent(...) 里显式传参。
            agent = Agent(
                name=f"agent_{i}",
                collide=self.collisions,
                color=color,
                shape=Sphere(radius=self.agent_radius),
                render_action=True,
                sensors=(
                    [
                        Lidar(
                            world,
                            n_rays=self.n_lidar_rays,
                            max_range=self.lidar_range,
                            entity_filter=entity_filter_agents,
                        ),
                    ]
                    if self.collisions
                    else None
                ),
            )
            agent.pos_rew = torch.zeros(batch_dim, device=device)
            agent.agent_collision_rew = agent.pos_rew.clone()
            world.add_agent(agent)

            # Add goals
            goal = Landmark(
                name=f"goal {i}",
                collide=False,
                color=color,
            )
            world.add_landmark(goal)
            agent.goal = goal

        self.pos_rew = torch.zeros(batch_dim, device=device)
        self.final_rew = self.pos_rew.clone()

        return world

    def reset_world_at(self, env_index: int = None):
        ScenarioUtils.spawn_entities_randomly(
            self.world.agents,
            self.world,
            env_index,
            self.min_distance_between_entities,
            (-self.world_spawning_x, self.world_spawning_x),
            (-self.world_spawning_y, self.world_spawning_y),
        )

        occupied_positions = torch.stack(
            [agent.state.pos for agent in self.world.agents], dim=1
        )
        if env_index is not None:
            occupied_positions = occupied_positions[env_index].unsqueeze(0)

        goal_poses = []
        for _ in self.world.agents:
            position = ScenarioUtils.find_random_pos_for_entity(
                occupied_positions=occupied_positions,
                env_index=env_index,
                world=self.world,
                min_dist_between_entities=self.min_distance_between_entities,
                x_bounds=(-self.world_spawning_x, self.world_spawning_x),
                y_bounds=(-self.world_spawning_y, self.world_spawning_y),
            )
            goal_poses.append(position.squeeze(1))
            occupied_positions = torch.cat([occupied_positions, position], dim=1)

        for i, agent in enumerate(self.world.agents):
            if self.split_goals:
                goal_index = int(i // self.agents_with_same_goal)
            else:
                goal_index = 0 if i < self.agents_with_same_goal else i

            agent.goal.set_pos(goal_poses[goal_index], batch_index=env_index)

            if env_index is None:
                agent.pos_shaping = (
                    torch.linalg.vector_norm(
                        agent.state.pos - agent.goal.state.pos,
                        dim=1,
                    )
                    * self.pos_shaping_factor
                )
            else:
                agent.pos_shaping[env_index] = (
                    torch.linalg.vector_norm(
                        agent.state.pos[env_index] - agent.goal.state.pos[env_index]
                    )
                    * self.pos_shaping_factor
                )

    def reward(self, agent: Agent):
        """
        计算智能体的奖励
        
        奖励包含三个部分:
        1. pos_rew: 位置shaping奖励（靠近目标获得正奖励）
        2. final_rew: 所有智能体都到达目标时的最终奖励
        3. agent_collision_rew: 智能体间碰撞的惩罚
        
        参数:
            agent (Agent): 要计算奖励的智能体
            
        返回:
            torch.Tensor: 奖励值，形状 (batch_dim,)
        """
        # 检查是否是第一个智能体（避免重复计算全局奖励）
        is_first = agent == self.world.agents[0]

        if is_first:
            # ========== 初始化奖励张量 ==========
            # 将位置奖励和最终奖励重置为0
            self.pos_rew[:] = 0
            self.final_rew[:] = 0

            # ========== 计算所有智能体的位置奖励 ==========
            for a in self.world.agents:
                # 累加每个智能体的位置奖励（shaping reward）
                self.pos_rew += self.agent_reward(a)
                # 初始化每个智能体的碰撞惩罚为0
                a.agent_collision_rew[:] = 0

            # ========== 检查是否所有智能体都到达目标 ==========
            # 将所有智能体的on_goal状态堆叠成张量，形状: (batch_dim, n_agents)
            # torch.all检查每个环境中是否所有智能体都在目标上
            self.all_goal_reached = torch.all(
                torch.stack([a.on_goal for a in self.world.agents], dim=-1),
                dim=-1,  # 沿智能体维度检查
            )  # 结果形状: (batch_dim,)

            # ========== 给予最终奖励 ==========
            # 只在所有智能体都到达目标的环境中给予最终奖励
            self.final_rew[self.all_goal_reached] = self.final_reward

            # ========== 计算智能体间碰撞惩罚 ==========
            # 遍历所有智能体对 (i, j)，其中 i < j（避免重复检查）
            for i, a in enumerate(self.world.agents):
                for j, b in enumerate(self.world.agents):
                    # 跳过 i >= j 的情况（避免自己和自己比较，避免重复）
                    if i <= j:
                        continue
                    # 检查两个智能体是否发生碰撞
                    if self.world.collides(a, b):
                        # 计算两个智能体之间的距离
                        distance = self.world.get_distance(a, b)
                        # 如果距离小于等于最小碰撞距离，给予惩罚
                        # 对两个智能体都施加惩罚（双向惩罚）
                        a.agent_collision_rew[
                            distance <= self.min_collision_distance
                        ] += self.agent_collision_penalty
                        b.agent_collision_rew[
                            distance <= self.min_collision_distance
                        ] += self.agent_collision_penalty

        # ========== 返回最终奖励 ==========
        # 如果是共享奖励，所有智能体获得相同的位置奖励
        # 否则，每个智能体获得自己的位置奖励
        pos_reward = self.pos_rew if self.shared_rew else agent.pos_rew
        # 返回: 位置奖励 + 最终奖励 + 碰撞惩罚
        return pos_reward + self.final_rew + agent.agent_collision_rew

    def agent_reward(self, agent: Agent):
        agent.distance_to_goal = torch.linalg.vector_norm(
            agent.state.pos - agent.goal.state.pos,
            dim=-1,
        )
        agent.on_goal = agent.distance_to_goal < agent.shape.radius       # 0.10
        #agent.on_goal = agent.distance_to_goal < agent.goal.shape.radius  # 0.05

        pos_shaping = agent.distance_to_goal * self.pos_shaping_factor
        agent.pos_rew = agent.pos_shaping - pos_shaping
        agent.pos_shaping = pos_shaping
        return agent.pos_rew

    def observation(self, agent: Agent):
        """
        计算智能体的观测
        
        观测内容:
        1. 智能体自身位置 (2维)
        2. 智能体自身速度 (2维)
        3. 到目标的相对位置 (2维或更多，取决于observe_all_goals)
        4. [可选] 激光雷达测量值 (如果启用碰撞检测)
        
        参数:
            agent (Agent): 要计算观测的智能体
            
        返回:
            torch.Tensor: 观测向量，形状 (batch_dim, obs_dim)
        """
        # ========== 收集目标位置信息 ==========
        goal_poses = []
        if self.observe_all_goals:
            # 如果观察所有目标，收集到所有智能体目标的相对位置
            # 这允许智能体知道其他智能体的目标位置
            for a in self.world.agents:
                # 计算当前智能体到其他智能体目标的相对位置
                # 注意: 这里是 agent.state.pos - a.goal.state.pos
                # 即从目标指向智能体的向量
                goal_poses.append(agent.state.pos - a.goal.state.pos)
        else:
            # 只观察自己的目标
            # 当前智能体到自己目标的相对位置
            goal_poses.append(agent.state.pos - agent.goal.state.pos)
        
        # ========== 拼接所有观测信息 ==========
        return torch.cat(
            [
                agent.state.pos,   # 智能体当前位置 (batch_dim, 2)
                agent.state.vel,   # 智能体当前速度 (batch_dim, 2)
            ]
            + goal_poses           # 目标位置信息 (列表，每个元素形状 (batch_dim, 2))
            + (
                # 如果启用碰撞检测，添加激光雷达测量值
                # _max_range - measure() 给出每个方向上最近障碍物的距离
                # 形状: (batch_dim, n_lidar_rays)
                [agent.sensors[0]._max_range - agent.sensors[0].measure()]
                if self.collisions
                else []  # 没有碰撞检测时不添加激光雷达数据
            ),
            dim=-1,  # 沿最后一维拼接所有观测
        )
        # 最终形状: (batch_dim, 4 + 2*n_goals + [n_lidar_rays])

    def done(self):
        return torch.stack(
            [
                torch.linalg.vector_norm(
                    agent.state.pos - agent.goal.state.pos,
                    dim=-1,
                )
                < agent.shape.radius
                for agent in self.world.agents
            ],
            dim=-1,
        ).all(-1)

    def info(self, agent: Agent) -> Dict[str, Tensor]:
        return {
            "pos_rew": self.pos_rew if self.shared_rew else agent.pos_rew,
            "final_rew": self.final_rew,
            "agent_collisions": agent.agent_collision_rew,
        }

    def extra_render(self, env_index: int = 0) -> "List[Geom]":
        from vmas.simulator import rendering

        geoms: List[Geom] = []

        # Communication lines
        for i, agent1 in enumerate(self.world.agents):
            for j, agent2 in enumerate(self.world.agents):
                if j <= i:
                    continue
                agent_dist = torch.linalg.vector_norm(
                    agent1.state.pos - agent2.state.pos, dim=-1
                )
                if agent_dist[env_index] <= self.comms_range:
                    color = Color.BLACK.value
                    line = rendering.Line(
                        (agent1.state.pos[env_index]),
                        (agent2.state.pos[env_index]),
                        width=1,
                    )
                    xform = rendering.Transform()
                    line.add_attr(xform)
                    line.set_color(*color)
                    geoms.append(line)

        return geoms


class LegacyClfQpHeuristicPolicy(BaseHeuristicPolicy):
    """Original VMAS CLF-QP heuristic kept as a comparison candidate."""

    def __init__(self, clf_epsilon=0.2, clf_slack=100.0, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.clf_epsilon = clf_epsilon  # Exponential CLF convergence rate
        self.clf_slack = clf_slack  # weights on CLF-QP slack variable
        self._qp_controllers = {}

    def _u_range_cache_key(self, u_range):
        if torch.is_tensor(u_range):
            if u_range.numel() == 1:
                return float(u_range.detach().cpu().item())
            return tuple(float(value) for value in u_range.detach().cpu().reshape(-1))
        if isinstance(u_range, (int, float)):
            return float(u_range)
        try:
            return float(u_range)
        except (TypeError, ValueError):
            return tuple(float(value) for value in u_range)

    def _get_qp_controller(self, u_range):
        u_range_key = self._u_range_cache_key(u_range)
        cache_key = (u_range_key, float(self.clf_epsilon), float(self.clf_slack))
        controller = self._qp_controllers.get(cache_key)
        if controller is None:
            controller = self._build_qp_controller(u_range_key)
            self._qp_controllers[cache_key] = controller
        return controller

    def _build_qp_controller(self, u_range):
        # Install it with: pip install cvxpylayers
        import cvxpy as cp
        from cvxpylayers.torch import CvxpyLayer

        if isinstance(u_range, tuple):
            u_range = cp.Constant(list(u_range))

        # Define Quadratic Program (QP) based controller
        u = cp.Variable(2)
        V_param = cp.Parameter(1)  # Lyapunov Function: V(x): x -> R, dim: (1,1)
        lfV_param = cp.Parameter(1)
        lgV_params = cp.Parameter(
            2
        )  # Lie derivative of Lyapunov Function, dim: (1, action_dim)
        clf_slack = cp.Variable(1)  # CLF constraint slack variable, dim: (1,1)

        constraints = []

        # QP Cost F = u^T @ u + clf_slack**2
        qp_objective = cp.Minimize(cp.sum_squares(u) + self.clf_slack * clf_slack**2)

        # control bounds between u_range
        constraints += [u <= u_range]
        constraints += [u >= -u_range]
        # CLF constraint
        constraints += [
            lfV_param + lgV_params @ u + self.clf_epsilon * V_param + clf_slack <= 0
        ]

        QP_problem = cp.Problem(qp_objective, constraints)

        # Initialize CVXPY layers
        return CvxpyLayer(
            QP_problem,
            parameters=[V_param, lfV_param, lgV_params],
            variables=[u],
        )

    def compute_actions(self, observations, u_ranges):
        if len(observations) == 0:
            return []

        actions = [None] * len(observations)
        groups = {}
        for index, u_range in enumerate(u_ranges):
            groups.setdefault(self._u_range_cache_key(u_range), []).append(index)

        for indices in groups.values():
            batch_sizes = [observations[index].shape[0] for index in indices]
            batched_observation = torch.cat(
                [observations[index] for index in indices],
                dim=0,
            )
            batched_action = self.compute_action(
                batched_observation,
                u_range=u_ranges[indices[0]],
            )
            for index, action in zip(indices, batched_action.split(batch_sizes, dim=0)):
                actions[index] = action

        return actions

    def compute_action(self, observation: Tensor, u_range: Tensor) -> Tensor:
        """
        QP inputs:
        These values need to computed apriri based on observation before passing into QP

        V: Lyapunov function value
        lfV: Lie derivative of Lyapunov function
        lgV: Lie derivative of Lyapunov function
        CLF_slack: CLF constraint slack variable

        QP outputs:
        u: action
        CLF_slack: CLF constraint slack variable, 0 if CLF constraint is satisfied
        """
        self.n_env = observation.shape[0]
        self.device = observation.device
        agent_pos = observation[:, :2]
        agent_vel = observation[:, 2:4]
        goal_pos = (-1.0) * (observation[:, 4:6] - agent_pos)

        # Pre-compute tensors for the CLF and CBF constraints,
        # Lyapunov Function from: https://arxiv.org/pdf/1903.03692.pdf

        # Laypunov function
        V_value = (
            (agent_pos[:, X] - goal_pos[:, X]) ** 2
            + 0.5 * (agent_pos[:, X] - goal_pos[:, X]) * agent_vel[:, X]
            + agent_vel[:, X] ** 2
            + (agent_pos[:, Y] - goal_pos[:, Y]) ** 2
            + 0.5 * (agent_pos[:, Y] - goal_pos[:, Y]) * agent_vel[:, Y]
            + agent_vel[:, Y] ** 2
        )

        LfV_val = (2 * (agent_pos[:, X] - goal_pos[:, X]) + agent_vel[:, X]) * (
            agent_vel[:, X]
        ) + (2 * (agent_pos[:, Y] - goal_pos[:, Y]) + agent_vel[:, Y]) * (
            agent_vel[:, Y]
        )
        LgV_vals = torch.stack(
            [
                0.5 * (agent_pos[:, X] - goal_pos[:, X]) + 2 * agent_vel[:, X],
                0.5 * (agent_pos[:, Y] - goal_pos[:, Y]) + 2 * agent_vel[:, Y],
            ],
            dim=1,
        )
        QP_controller = self._get_qp_controller(u_range)

        # Solve QP
        CVXpylayer_parameters = [
            V_value.unsqueeze(1),
            LfV_val.unsqueeze(1),
            LgV_vals,
        ]
        action = QP_controller(*CVXpylayer_parameters, solver_args={"max_iters": 500})[
            0
        ]

        return action


OriginalHeuristicPolicy = LegacyClfQpHeuristicPolicy


class HeuristicPolicy(BaseHeuristicPolicy):
    """Goal-seeking navigation heuristic with reciprocal local avoidance."""

    def __init__(
        self,
        goal_gain: float = 2.5,
        velocity_damping: float = 2.4,
        repulsion_gain: float = 3.2,
        sidestep_gain: float = 1.0,
        yield_gain: float = 0.0,
        avoid_radius: float = 0.55,
        goal_stop_radius: float = 0.045,
        stop_damping: float = 2.5,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.goal_gain = goal_gain
        self.velocity_damping = velocity_damping
        self.repulsion_gain = repulsion_gain
        self.sidestep_gain = sidestep_gain
        self.yield_gain = yield_gain
        self.avoid_radius = avoid_radius
        self.goal_stop_radius = goal_stop_radius
        self.stop_damping = stop_damping

    def compute_actions(self, observations, u_ranges):
        if len(observations) == 0:
            return []

        obs = torch.stack(list(observations), dim=1)
        pos = obs[..., :2]
        vel = obs[..., 2:4]
        to_goal = -self._goal_relative_positions(obs)
        goal_dist = torch.linalg.vector_norm(to_goal, dim=-1, keepdim=True).clamp_min(
            1e-6
        )
        goal_dir = to_goal / goal_dist

        action = self.goal_gain * to_goal - self.velocity_damping * vel
        action = action + self._avoidance_action(pos, goal_dir)
        action = torch.where(
            goal_dist < self.goal_stop_radius,
            -self.stop_damping * vel,
            action,
        )

        return [
            self._clamp_action(action[:, index], u_ranges[index])
            for index in range(len(observations))
        ]

    def compute_action(self, observation: Tensor, u_range: Tensor) -> Tensor:
        to_goal = -observation[:, 4:6]
        vel = observation[:, 2:4]
        action = self.goal_gain * to_goal - self.velocity_damping * vel
        goal_dist = torch.linalg.vector_norm(to_goal, dim=-1, keepdim=True)
        action = torch.where(
            goal_dist < self.goal_stop_radius,
            -self.stop_damping * vel,
            action,
        )
        return self._clamp_action(action, u_range)

    @staticmethod
    def _goal_relative_positions(obs: Tensor) -> Tensor:
        return obs[..., 4:6]

    def _avoidance_action(self, pos: Tensor, goal_dir: Tensor) -> Tensor:
        n_agents = pos.shape[1]
        if n_agents <= 1:
            return torch.zeros_like(pos)

        rel_pos = pos.unsqueeze(2) - pos.unsqueeze(1)
        distance = torch.linalg.vector_norm(rel_pos, dim=-1, keepdim=True).clamp_min(
            1e-6
        )
        eye = torch.eye(
            n_agents,
            device=pos.device,
            dtype=torch.bool,
        ).view(1, n_agents, n_agents, 1)
        active = (distance < self.avoid_radius) & (~eye)
        proximity = ((self.avoid_radius - distance) / self.avoid_radius).clamp_min(0.0)

        away = rel_pos / distance
        repulsion = (
            away * proximity.square() * active
        ).sum(dim=2) * self.repulsion_gain

        tangent = torch.stack([-away[..., Y], away[..., X]], dim=-1)
        indices = torch.arange(n_agents, device=pos.device)
        sidestep_sign = torch.where(
            indices.view(1, n_agents, 1, 1) < indices.view(1, 1, n_agents, 1),
            1.0,
            -1.0,
        )
        sidestep = (
            tangent * sidestep_sign * proximity.square() * active
        ).sum(dim=2) * self.sidestep_gain

        lower_priority = (
            indices.view(1, n_agents, 1, 1) > indices.view(1, 1, n_agents, 1)
        )
        yield_term = (
            goal_dir
            * (proximity * active * lower_priority).sum(dim=2)
            * self.yield_gain
        )

        return repulsion + sidestep - yield_term

    @staticmethod
    def _clamp_action(action: Tensor, u_range) -> Tensor:
        if torch.is_tensor(u_range):
            return torch.max(torch.min(action, u_range), -u_range)
        return torch.clamp(action, -float(u_range), float(u_range))


if __name__ == "__main__":
    render_interactively(
        __file__,
        control_two_agents=True,
    )
