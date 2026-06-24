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
        self.n_agents = kwargs.pop("n_agents", 5)
        self.n_targets = kwargs.pop("n_targets", 7)
        self.x_semidim = kwargs.pop("x_semidim", 1)
        self.y_semidim = kwargs.pop("y_semidim", 1)
        self._min_dist_between_entities = kwargs.pop("min_dist_between_entities", 0.2)
        self._lidar_range = kwargs.pop("lidar_range", 0.35)
        self._covering_range = kwargs.pop("covering_range", 0.25)

        self.use_agent_lidar = kwargs.pop("use_agent_lidar", False)
        self.n_lidar_rays_entities = kwargs.pop("n_lidar_rays_entities", 15)
        self.n_lidar_rays_agents = kwargs.pop("n_lidar_rays_agents", 12)

        self._agents_per_target = kwargs.pop("agents_per_target", 2)
        self.targets_respawn = kwargs.pop("targets_respawn", True)
        self.shared_reward = kwargs.pop("shared_reward", False)

        self.agent_collision_penalty = kwargs.pop("agent_collision_penalty", 0)
        self.covering_rew_coeff = kwargs.pop("covering_rew_coeff", 1.0)
        self.time_penalty = kwargs.pop("time_penalty", 0)
        ScenarioUtils.check_kwargs_consumed(kwargs)

        self._comms_range = self._lidar_range
        self.min_collision_distance = 0.005
        self.agent_radius = 0.05
        self.target_radius = self.agent_radius

        self.viewer_zoom = 1
        self.target_color = Color.GREEN

        # Make world
        world = World(
            batch_dim,
            device,
            x_semidim=self.x_semidim,
            y_semidim=self.y_semidim,
            collision_force=500,
            substeps=2,
            drag=0.25,
        )

        # Add agents
        entity_filter_agents: Callable[[Entity], bool] = lambda e: e.name.startswith(
            "agent"
        )
        entity_filter_targets: Callable[[Entity], bool] = lambda e: e.name.startswith(
            "target"
        )
        for i in range(self.n_agents):
            # Constraint: all agents have same action range and multiplier
            agent = Agent(
                name=f"agent_{i}",
                collide=True,
                shape=Sphere(radius=self.agent_radius),
                sensors=(
                    [
                        Lidar(
                            world,
                            n_rays=self.n_lidar_rays_entities,
                            max_range=self._lidar_range,
                            entity_filter=entity_filter_targets,
                            render_color=Color.GREEN,
                        )
                    ]
                    + (
                        [
                            Lidar(
                                world,
                                angle_start=0.05,
                                angle_end=2 * torch.pi + 0.05,
                                n_rays=self.n_lidar_rays_agents,
                                max_range=self._lidar_range,
                                entity_filter=entity_filter_agents,
                                render_color=Color.BLUE,
                            )
                        ]
                        if self.use_agent_lidar
                        else []
                    )
                ),
            )
            agent.collision_rew = torch.zeros(batch_dim, device=device)
            agent.covering_reward = agent.collision_rew.clone()
            world.add_agent(agent)

        self._targets = []
        for i in range(self.n_targets):
            target = Landmark(
                name=f"target_{i}",
                collide=True,
                movable=False,
                shape=Sphere(radius=self.target_radius),
                color=self.target_color,
            )
            world.add_landmark(target)
            self._targets.append(target)

        self.covered_targets = torch.zeros(batch_dim, self.n_targets, device=device)
        self.shared_covering_rew = torch.zeros(batch_dim, device=device)

        return world

    def reset_world_at(self, env_index: int = None):
        placable_entities = self._targets[: self.n_targets] + self.world.agents
        if env_index is None:
            self.all_time_covered_targets = torch.full(
                (self.world.batch_dim, self.n_targets),
                False,
                device=self.world.device,
            )
        else:
            self.all_time_covered_targets[env_index] = False
        ScenarioUtils.spawn_entities_randomly(
            entities=placable_entities,
            world=self.world,
            env_index=env_index,
            min_dist_between_entities=self._min_dist_between_entities,
            x_bounds=(-self.world.x_semidim, self.world.x_semidim),
            y_bounds=(-self.world.y_semidim, self.world.y_semidim),
        )
        for target in self._targets[self.n_targets :]:
            target.set_pos(self.get_outside_pos(env_index), batch_index=env_index)

    def reward(self, agent: Agent):
        """
        计算智能体的奖励
        
        场景目标: 多个智能体协作发现并覆盖目标区域
        奖励机制:
        1. 覆盖奖励: 当足够数量的智能体覆盖目标时获得奖励
        2. 时间惩罚: 每个时间步的小惩罚（鼓励快速完成）
        3. 碰撞惩罚: 智能体间碰撞的惩罚
        
        参数:
            agent (Agent): 要计算奖励的智能体
            
        返回:
            torch.Tensor: 奖励值，形状 (batch_dim,)
        """
        # 检查是否是第一个和最后一个智能体（用于全局计算）
        is_first = agent == self.world.agents[0]
        is_last = agent == self.world.agents[-1]

        if is_first:
            # ========== 初始化时间惩罚 ==========
            # 每个时间步都施加固定的时间惩罚（通常是负值）
            # 鼓励智能体尽快完成任务
            self.time_rew = torch.full(
                (self.world.batch_dim,),
                self.time_penalty,  # 时间惩罚值（例如 -0.01）
                device=self.world.device,
            )
            
            # ========== 收集所有位置信息 ==========
            # 所有智能体的位置，形状: (batch_dim, n_agents, 2)
            self.agents_pos = torch.stack(
                [a.state.pos for a in self.world.agents], dim=1
            )
            # 所有目标的位置，形状: (batch_dim, n_targets, 2)
            self.targets_pos = torch.stack([t.state.pos for t in self._targets], dim=1)
            
            # ========== 计算距离矩阵 ==========
            # 计算所有智能体到所有目标的距离
            # 形状: (batch_dim, n_agents, n_targets)
            self.agents_targets_dists = torch.cdist(self.agents_pos, self.targets_pos)
            
            # ========== 计算每个目标被多少智能体覆盖 ==========
            # 检查距离是否小于覆盖范围，转换为整数后求和
            # 形状: (batch_dim, n_targets)
            self.agents_per_target = torch.sum(
                (self.agents_targets_dists < self._covering_range).type(torch.int),
                dim=1,  # 沿智能体维度求和
            )
            
            # ========== 判断目标是否被充分覆盖 ==========
            # 如果覆盖某目标的智能体数量 >= 要求的数量，则该目标被覆盖
            # 形状: (batch_dim, n_targets)
            self.covered_targets = self.agents_per_target >= self._agents_per_target

            # ========== 计算共享覆盖奖励 ==========
            # 初始化共享覆盖奖励为0
            self.shared_covering_rew[:] = 0
            # 累加所有智能体的覆盖奖励
            for a in self.world.agents:
                self.shared_covering_rew += self.agent_reward(a)
            # 除以2避免重复计数（因为每个目标可能被多个智能体覆盖）
            self.shared_covering_rew[self.shared_covering_rew != 0] /= 2

        # ========== 计算碰撞惩罚 ==========
        # 避免智能体之间的碰撞
        agent.collision_rew[:] = 0  # 初始化碰撞奖励为0
        for a in self.world.agents:
            # 跳过智能体自己
            if a != agent:
                # 如果两个智能体的距离小于最小碰撞距离，施加惩罚
                agent.collision_rew[
                    self.world.get_distance(a, agent) < self.min_collision_distance
                ] += self.agent_collision_penalty  # 累加碰撞惩罚（负值）

        if is_last:
            # ========== 处理目标重生 ==========
            if self.targets_respawn:
                # 如果启用目标重生，被覆盖的目标会在新位置重新生成
                
                # 收集所有智能体的位置（占据的空间）
                occupied_positions_agents = [self.agents_pos]
                
                # 遍历每个目标，为被覆盖的目标寻找新位置
                for i, target in enumerate(self._targets):
                    # 收集除当前目标外的其他目标位置
                    occupied_positions_targets = [
                        o.state.pos.unsqueeze(1)
                        for o in self._targets
                        if o is not target  # 排除当前目标
                    ]
                    
                    # 合并所有已占据的位置（智能体 + 其他目标）
                    occupied_positions = torch.cat(
                        occupied_positions_agents + occupied_positions_targets,
                        dim=1,  # 形状: (batch_dim, n_occupied, 2)
                    )
                    
                    # 为目标寻找一个随机的新位置（避开已占据的位置）
                    pos = ScenarioUtils.find_random_pos_for_entity(
                        occupied_positions,
                        env_index=None,  # 为所有环境寻找位置
                        world=self.world,
                        min_dist_between_entities=self._min_dist_between_entities,
                        x_bounds=(-self.world.x_semidim, self.world.x_semidim),
                        y_bounds=(-self.world.y_semidim, self.world.y_semidim),
                    )

                    # 将被覆盖的目标移动到新位置
                    target.state.pos[self.covered_targets[:, i]] = pos[
                        self.covered_targets[:, i]
                    ].squeeze(1)
            else:
                # ----- 如果不重生目标，将其移到外部（不可见） -----
                # 累计所有时间步被覆盖的目标
                self.all_time_covered_targets += self.covered_targets
                
                # 将被覆盖的目标移到世界外部（隐藏）
                for i, target in enumerate(self._targets):
                    target.state.pos[self.covered_targets[:, i]] = self.get_outside_pos(
                        None
                    )[self.covered_targets[:, i]]
        
        # ========== 选择覆盖奖励 ==========
        # 根据是否共享奖励选择个体或共享的覆盖奖励
        covering_rew = (
            agent.covering_reward  # 个体奖励
            if not self.shared_reward
            else self.shared_covering_rew  # 共享奖励
        )

        # ========== 返回总奖励 ==========
        # 总奖励 = 碰撞惩罚 + 覆盖奖励 + 时间惩罚
        return agent.collision_rew + covering_rew + self.time_rew

    def get_outside_pos(self, env_index):
        return torch.empty(
            (
                (1, self.world.dim_p)
                if env_index is not None
                else (self.world.batch_dim, self.world.dim_p)
            ),
            device=self.world.device,
        ).uniform_(-1000 * self.world.x_semidim, -10 * self.world.x_semidim)

    def agent_reward(self, agent):
        agent_index = self.world.agents.index(agent)

        agent.covering_reward[:] = 0
        targets_covered_by_agent = (
            self.agents_targets_dists[:, agent_index] < self._covering_range
        )
        num_covered_targets_covered_by_agent = (
            targets_covered_by_agent * self.covered_targets
        ).sum(dim=-1)
        agent.covering_reward += (
            num_covered_targets_covered_by_agent * self.covering_rew_coeff
        )
        return agent.covering_reward

    def observation(self, agent: Agent):
        """
        计算智能体的观测
        
        观测内容:
        1. 智能体自身位置 (2维)
        2. 智能体自身速度 (2维)
        3. 目标激光雷达测量值 (n_lidar_rays维，检测目标位置)
        4. [可选] 智能体激光雷达测量值 (n_lidar_rays维，检测其他智能体)
        
        参数:
            agent (Agent): 要计算观测的智能体
            
        返回:
            torch.Tensor: 观测向量，形状 (batch_dim, obs_dim)
        """
        # ========== 获取目标激光雷达测量值 ==========
        # sensors[0]是目标激光雷达，用于检测和定位目标
        # 返回每个射线方向上最近目标的距离
        # 形状: (batch_dim, n_lidar_rays)
        lidar_1_measures = agent.sensors[0].measure()
        
        # ========== 拼接所有观测 ==========
        return torch.cat(
            [
                agent.state.pos,      # 智能体位置 (batch_dim, 2)
                agent.state.vel,      # 智能体速度 (batch_dim, 2)
                lidar_1_measures      # 目标激光雷达 (batch_dim, n_lidar_rays)
            ]
            + (
                # 如果启用智能体激光雷达，添加对其他智能体的检测
                # sensors[1]用于检测和避开其他智能体
                [agent.sensors[1].measure()] if self.use_agent_lidar else []
            ),
            dim=-1,  # 沿最后一维拼接
        )
        # 最终形状: (batch_dim, 4 + n_lidar_rays + [n_lidar_rays])

    def info(self, agent: Agent) -> Dict[str, Tensor]:
        info = {
            "covering_reward": (
                agent.covering_reward
                if not self.shared_reward
                else self.shared_covering_rew
            ),
            "collision_rew": agent.collision_rew,
            "targets_covered": self.covered_targets.sum(-1),
        }
        return info

    def done(self):
        return self.all_time_covered_targets.all(dim=-1)

    def extra_render(self, env_index: int = 0) -> "List[Geom]":
        from vmas.simulator import rendering

        geoms: List[Geom] = []
        # Target ranges
        for target in self._targets:
            range_circle = rendering.make_circle(self._covering_range, filled=False)
            xform = rendering.Transform()
            xform.set_translation(*target.state.pos[env_index])
            range_circle.add_attr(xform)
            range_circle.set_color(*self.target_color.value)
            geoms.append(range_circle)
        # Communication lines
        for i, agent1 in enumerate(self.world.agents):
            for j, agent2 in enumerate(self.world.agents):
                if j <= i:
                    continue
                agent_dist = torch.linalg.vector_norm(
                    agent1.state.pos - agent2.state.pos, dim=-1
                )
                if agent_dist[env_index] <= self._comms_range:
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


class HeuristicPolicy(BaseHeuristicPolicy):
    def compute_action(self, observation: torch.Tensor, u_range: float) -> torch.Tensor:
        assert self.continuous_actions

        # First calculate the closest point to a circle of radius circle_radius given the current position
        circle_origin = torch.zeros(1, 2, device=observation.device)
        circle_radius = 0.75
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

        # Move towards targets within visibility range
        lidar_targets = observation[:, 4:19]
        target_visible = torch.any(lidar_targets < 0.3, dim=1)
        _, target_dir_index = torch.min(lidar_targets, dim=1)
        target_dir = target_dir_index / lidar_targets.shape[1] * 2 * torch.pi
        target_vec = torch.stack([torch.cos(target_dir), torch.sin(target_dir)], dim=1)
        des_pos_target = current_pos + target_vec * 0.1
        des_pos[target_visible] = des_pos_target[target_visible]

        if observation.shape[-1] > 19:
            # Move away from other agents within visibility range
            lidar_agents = observation[:, 19:31]
            agent_visible = torch.any(lidar_agents < 0.15, dim=1)
            _, agent_dir_index = torch.min(lidar_agents, dim=1)
            agent_dir = agent_dir_index / lidar_agents.shape[1] * 2 * torch.pi
            agent_vec = torch.stack([torch.cos(agent_dir), torch.sin(agent_dir)], dim=1)
            des_pos_agent = current_pos - agent_vec * 0.1
            des_pos[agent_visible] = des_pos_agent[agent_visible]

        action = torch.clamp(
            (des_pos - current_pos) * 10,
            min=-u_range,
            max=u_range,
        )

        return action


if __name__ == "__main__":
    render_interactively(__file__, control_two_agents=True)
