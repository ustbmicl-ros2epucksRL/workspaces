#  Copyright (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  LICENSE file in the root directory of this source tree.
from typing import Callable, Dict

import torch
from torch import Tensor

from vmas import render_interactively
from vmas.simulator.core import Agent, Entity, Landmark, Sphere, World
from vmas.simulator.heuristic_policy import BaseHeuristicPolicy
from vmas.simulator.scenario import BaseScenario
from vmas.simulator.sensors import Lidar
from vmas.simulator.utils import Color, ScenarioUtils, X, Y


class Scenario(BaseScenario):
    def make_world(self, batch_dim: int, device: torch.device, **kwargs):
        n_agents = kwargs.pop("n_agents", 4)
        n_obstacles = kwargs.pop("n_obstacles", 5)
        self._min_dist_between_entities = kwargs.pop("min_dist_between_entities", 0.15)

        self.n_lidar_rays = kwargs.pop("n_lidar_rays", 12)

        self.collision_reward = kwargs.pop("collision_reward", -0.1)
        self.dist_shaping_factor = kwargs.pop("dist_shaping_factor", 1)
        self.target_radius = kwargs.pop("target_radius", 0.6)
        self.target_radius_tolerance = kwargs.pop("target_radius_tolerance", 0.0)
        self.target_dist_reward_factor = kwargs.pop("target_dist_reward_factor", 0.5)
        self.surround_arc_reward_factor = kwargs.pop("surround_arc_reward_factor", 0.05)
        # Added: optional static target placed at the origin
        self.static_at_origin = kwargs.pop("static_at_origin", False)
        ScenarioUtils.check_kwargs_consumed(kwargs)

        self.plot_grid = True
        self.desired_distance = 0.1
        self.min_collision_distance = 0.05
        self.x_dim = 1
        self.y_dim = 1

        # Make world
        world = World(batch_dim, device, collision_force=400, substeps=5)
        # Add agents
        target_action_script = (
            self.static_target_action_script_creator()
            if self.static_at_origin
            else self.action_script_creator()
        )
        self._target = Agent(
            name="target",
            collide=True,
            color=Color.GREEN,
            render_action=True,
            action_script=target_action_script,
            movable=not self.static_at_origin,
        )
        world.add_agent(self._target)
        goal_entity_filter: Callable[[Entity], bool] = lambda e: not isinstance(
            e, Agent
        )
        for i in range(n_agents):
            agent = Agent(
                name=f"agent_{i}",
                collide=True,
                sensors=[
                    Lidar(
                        world,
                        n_rays=self.n_lidar_rays,
                        max_range=0.2,
                        entity_filter=goal_entity_filter,
                    )
                ],
                render_action=True,
            )
            agent.collision_rew = torch.zeros(batch_dim, device=device)
            agent.dist_rew = agent.collision_rew.clone()

            world.add_agent(agent)

        self.surround_arc_rew = torch.zeros(batch_dim, device=device)

        # Add landmarks
        self.obstacles = []
        for i in range(n_obstacles):
            obstacle = Landmark(
                name=f"obstacle_{i}",
                collide=True,
                movable=False,
                shape=Sphere(radius=0.1),
                color=Color.RED,
            )
            world.add_landmark(obstacle)
            self.obstacles.append(obstacle)

        return world

    def action_script_creator(self):
        def action_script(agent, world):
            t = self.t / 30
            agent.action.u = torch.stack([torch.cos(t), torch.sin(t)], dim=1)

        return action_script

    def static_target_action_script_creator(self):
        def action_script(agent, world):
            # Added: keep optional target stationary at the origin
            agent.action.u = torch.zeros((world.batch_dim, world.dim_p), device=world.device)

        return action_script

    def reset_world_at(self, env_index: int = None):
        target_pos = torch.zeros(
            (
                (1, self.world.dim_p)
                if env_index is not None
                else (self.world.batch_dim, self.world.dim_p)
            ),
            device=self.world.device,
            dtype=torch.float32,
        )

        if self.static_at_origin:
            target_pos[:] = 0
        else:
            target_pos[:, Y] = -self.y_dim
        self._target.set_pos(target_pos, batch_index=env_index)
        ScenarioUtils.spawn_entities_randomly(
            self.obstacles + self.world.policy_agents,
            self.world,
            env_index,
            self._min_dist_between_entities,
            x_bounds=(-self.x_dim, self.x_dim),
            y_bounds=(-self.y_dim, self.y_dim),
            occupied_positions=target_pos.unsqueeze(1),
        )

        for agent in self.world.policy_agents:
            if env_index is None:
                agent.distance_shaping = (
                    torch.stack(
                        [
                            torch.linalg.vector_norm(
                                agent.state.pos - a.state.pos, dim=-1
                            )
                            for a in self.world.agents
                            if a != agent
                        ],
                        dim=1,
                    )
                    - self.desired_distance
                ).pow(2).mean(-1) * self.dist_shaping_factor

            else:
                agent.distance_shaping[env_index] = (
                    torch.stack(
                        [
                            torch.linalg.vector_norm(
                                agent.state.pos[env_index] - a.state.pos[env_index]
                            )
                            for a in self.world.agents
                            if a != agent
                        ],
                        dim=0,
                    )
                    - self.desired_distance
                ).pow(2).mean(-1) * self.dist_shaping_factor

        if env_index is None:
            self.t = torch.zeros(self.world.batch_dim, device=self.world.device)
        else:
            self.t[env_index] = 0

    def reward(self, agent: Agent):
        """
        计算智能体的奖励
        
        场景目标: 模拟鸟群/鱼群的集群行为（flocking）
        奖励机制:
        1. 距离奖励: 保持智能体之间的理想距离（不太远也不太近）
        2. 碰撞惩罚: 避免智能体间的碰撞以及智能体与障碍物的碰撞
        
        参数:
            agent (Agent): 要计算奖励的智能体
            
        返回:
            torch.Tensor: 奖励值，形状 (batch_dim,)
        """
        # 检查是否是第一个策略智能体（用于全局计算）
        is_first = self.world.policy_agents.index(agent) == 0

        if is_first:
            # ========== 更新时间步计数器 ==========
            self.t += 1

            # ========== 计算碰撞惩罚 ==========
            # 避免智能体之间的碰撞
            if self.collision_reward != 0:
                # 初始化所有策略智能体的碰撞奖励为0
                for a in self.world.policy_agents:
                    a.collision_rew[:] = 0

                # 遍历所有智能体对，检查碰撞
                for i, a in enumerate(self.world.agents):
                    for j, b in enumerate(self.world.agents):
                        # 跳过 j <= i 的情况（避免重复和自己与自己）
                        if j <= i:
                            continue
                        
                        # 检查两个智能体是否发生碰撞
                        # 碰撞条件: 距离 <= 最小碰撞距离
                        collision = (
                            self.world.get_distance(a, b) <= self.min_collision_distance
                        )
                        
                        # 只对策略控制的智能体（非脚本控制）施加惩罚
                        if a.action_script is None:
                            a.collision_rew[collision] += self.collision_reward
                        if b.action_script is None:
                            b.collision_rew[collision] += self.collision_reward

                # ========== 计算智能体与障碍物的碰撞惩罚 ==========
                # 遍历所有策略智能体和障碍物，检查碰撞
                for a in self.world.policy_agents:
                    for obstacle in self.obstacles:
                        # 检查智能体与障碍物是否发生碰撞
                        # 碰撞条件: 距离 <= 最小碰撞距离
                        obstacle_collision = (
                            self.world.get_distance(a, obstacle) <= self.min_collision_distance
                        )
                        # 对发生碰撞的智能体施加惩罚
                        a.collision_rew[obstacle_collision] += self.collision_reward

            policy_positions = torch.stack(
                [a.state.pos for a in self.world.policy_agents],
                dim=1,
            )
            relative_positions = policy_positions - self._target.state.pos.unsqueeze(1)
            angles = torch.atan2(relative_positions[..., Y], relative_positions[..., X])
            angles = torch.remainder(angles, 2 * torch.pi)
            sorted_angles, _ = torch.sort(angles, dim=1)
            if len(self.world.policy_agents) == 1:
                max_uncovered_arc = torch.full_like(sorted_angles[:, 0], 2 * torch.pi)
            else:
                gaps = sorted_angles[:, 1:] - sorted_angles[:, :-1]
                wrap_gap = sorted_angles[:, :1] + 2 * torch.pi - sorted_angles[:, -1:]
                max_uncovered_arc = torch.cat([gaps, wrap_gap], dim=1).max(dim=1).values
            self.surround_arc_rew = -max_uncovered_arc * self.surround_arc_reward_factor

        # ========== 计算距离奖励（保持集群） ==========
        # 目标: 保持智能体之间的理想距离（separation）
        # 1. 计算当前智能体到所有其他智能体的距离
        agents_dist_shaping = (
            torch.stack(
                [
                    # 计算到每个其他智能体的欧氏距离
                    torch.linalg.vector_norm(agent.state.pos - a.state.pos, dim=-1)
                    for a in self.world.agents
                    if a != agent  # 排除自己
                ],
                dim=1,  # 形状: (batch_dim, n_agents-1)
            )
            - self.desired_distance  # 2. 减去理想距离（偏差）
        ).pow(2).mean(-1) * self.dist_shaping_factor  # 3. 平方误差的平均 * 缩放因子
        # 结果形状: (batch_dim,)
        
        # ========== 计算shaping reward ==========
        # shaping reward = 上一步的shaping - 当前的shaping
        # 如果当前shaping更小（更接近理想距离），reward为正
        agent.dist_rew = agent.distance_shaping - agents_dist_shaping
        
        # 更新shaping值（用于下一步计算）
        agent.distance_shaping = agents_dist_shaping

        # 添加靠近绿色目标的奖励
        dist_to_target = torch.linalg.vector_norm(
            agent.state.pos - self._target.state.pos, dim=-1
        )
        # 奖励在半径0.6左右运动（类似HeuristicPolicy的圆周运动）
        target_dist_error = torch.abs(dist_to_target - self.target_radius)
        target_dist_reward = (
            -torch.relu(target_dist_error - self.target_radius_tolerance).pow(2)
            * self.target_dist_reward_factor
        )

        # ========== 返回总奖励 ==========
        # 总奖励 = 碰撞惩罚 + 距离奖励
        return (
            agent.collision_rew
            + target_dist_reward
            + self.surround_arc_rew
            #+ agent.dist_rew
        )

    def observation(self, agent: Agent):
        """
        计算智能体的观测
        
        观测内容:
        1. 智能体自身位置 (2维)
        2. 智能体自身速度 (2维)
        3. 智能体到目标的相对位置 (2维)
        4. 激光雷达测量值 (n_lidar_rays维，检测其他智能体)
        
        参数:
            agent (Agent): 要计算观测的智能体
            
        返回:
            torch.Tensor: 观测向量，形状 (batch_dim, 6 + n_lidar_rays)
        """
        # ========== 拼接所有观测信息 ==========
        return torch.cat(
            [
                agent.state.pos,                          # 智能体位置 (batch_dim, 2)
                agent.state.vel,                          # 智能体速度 (batch_dim, 2)
                agent.state.pos - self._target.state.pos, # 到目标的相对位置 (batch_dim, 2)
                                                           # 用于引导整个群体移动方向
                agent.sensors[0].measure(),               # 激光雷达测量 (batch_dim, n_lidar_rays)
                                                           # 检测周围其他智能体的位置，用于避障和保持队形
            ],
            dim=-1,  # 沿最后一维拼接
        )
        # 最终形状: (batch_dim, 6 + n_lidar_rays)

    def info(self, agent: Agent) -> Dict[str, Tensor]:
        info = {
            "agent_collision_rew": agent.collision_rew,
            "agent_distance_rew": agent.dist_rew,
            "surround_arc_rew": self.surround_arc_rew,
        }

        return info


class HeuristicPolicy(BaseHeuristicPolicy):
    def compute_action(self, observation: torch.Tensor, u_range: float) -> torch.Tensor:
        assert self.continuous_actions

        # First calculate the closest point to a circle of radius circle_radius given the current position
        circle_origin = torch.zeros(1, 2)
        circle_radius = 0.6
        current_pos = observation[:, :2]
        v = current_pos - circle_origin
        closest_point_on_circ = (
            circle_origin + v / torch.linalg.norm(v, dim=1).unsqueeze(1) * circle_radius
        )

        # calculate the normal vector of the vector from the origin of the circle to that closest point
        # on the circle. Adding this scaled normal vector to the other vector gives us a target point we
        # try to reach, thus resulting in a circular motion.
        closest_point_on_circ_normal = torch.stack(
            [closest_point_on_circ[:, Y], -closest_point_on_circ[:, X]], dim=1
        )
        closest_point_on_circ_normal /= torch.linalg.norm(
            closest_point_on_circ_normal, dim=1
        ).unsqueeze(1)
        closest_point_on_circ_normal *= 0.1
        des_pos = closest_point_on_circ + closest_point_on_circ_normal

        # Move away from other agents and obstcles within visibility range
        lidar = observation[:, 6:18]
        object_visible = torch.any(lidar < 0.1, dim=1)
        _, object_dir_index = torch.min(lidar, dim=1)
        object_dir = object_dir_index / lidar.shape[1] * 2 * torch.pi
        object_vec = torch.stack([torch.cos(object_dir), torch.sin(object_dir)], dim=1)
        des_pos_object = current_pos - object_vec * 0.1
        des_pos[object_visible] = des_pos_object[object_visible]

        action = torch.clamp(
            (des_pos - current_pos) * 10,
            min=-u_range,
            max=u_range,
        )

        return action


if __name__ == "__main__":
    render_interactively(__file__, control_two_agents=True)
