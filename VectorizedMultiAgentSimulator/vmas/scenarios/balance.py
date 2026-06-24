#  Copyright (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  LICENSE file in the root directory of this source tree.

import torch

from vmas import render_interactively
from vmas.simulator.core import Agent, Box, Landmark, Line, Sphere, World
from vmas.simulator.heuristic_policy import BaseHeuristicPolicy
from vmas.simulator.scenario import BaseScenario
from vmas.simulator.utils import Color, ScenarioUtils, Y


class Scenario(BaseScenario):
    def make_world(self, batch_dim: int, device: torch.device, **kwargs):
        self.n_agents = kwargs.pop("n_agents", 3)
        self.package_mass = kwargs.pop("package_mass", 5)
        self.random_package_pos_on_line = kwargs.pop("random_package_pos_on_line", True)
        ScenarioUtils.check_kwargs_consumed(kwargs)

        assert self.n_agents > 1

        self.line_length = 0.8
        self.agent_radius = 0.03

        self.shaping_factor = 100
        self.fall_reward = -10

        self.visualize_semidims = False

        # Make world
        world = World(batch_dim, device, gravity=(0.0, -0.05), y_semidim=1)
        # Add agents
        for i in range(self.n_agents):
            agent = Agent(
                name=f"agent_{i}",
                shape=Sphere(self.agent_radius),
                u_multiplier=0.7,
            )
            world.add_agent(agent)

        goal = Landmark(
            name="goal",
            collide=False,
            shape=Sphere(),
            color=Color.LIGHT_GREEN,
        )
        world.add_landmark(goal)
        self.package = Landmark(
            name="package",
            collide=True,
            movable=True,
            shape=Sphere(),
            mass=self.package_mass,
            color=Color.RED,
        )
        self.package.goal = goal
        world.add_landmark(self.package)
        # Add landmarks

        self.line = Landmark(
            name="line",
            shape=Line(length=self.line_length),
            collide=True,
            movable=True,
            rotatable=True,
            mass=5,
            color=Color.BLACK,
        )
        world.add_landmark(self.line)

        self.floor = Landmark(
            name="floor",
            collide=True,
            shape=Box(length=10, width=1),
            color=Color.WHITE,
        )
        world.add_landmark(self.floor)

        self.pos_rew = torch.zeros(batch_dim, device=device, dtype=torch.float32)
        self.ground_rew = self.pos_rew.clone()

        return world

    def reset_world_at(self, env_index: int = None):
        goal_pos = torch.cat(
            [
                torch.zeros(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    -1.0,
                    1.0,
                ),
                torch.zeros(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    0.0,
                    self.world.y_semidim,
                ),
            ],
            dim=1,
        )
        line_pos = torch.cat(
            [
                torch.zeros(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    -1.0 + self.line_length / 2,
                    1.0 - self.line_length / 2,
                ),
                torch.full(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    -self.world.y_semidim + self.agent_radius * 2,
                    device=self.world.device,
                    dtype=torch.float32,
                ),
            ],
            dim=1,
        )
        package_rel_pos = torch.cat(
            [
                torch.zeros(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    (
                        -self.line_length / 2 + self.package.shape.radius
                        if self.random_package_pos_on_line
                        else 0.0
                    ),
                    (
                        self.line_length / 2 - self.package.shape.radius
                        if self.random_package_pos_on_line
                        else 0.0
                    ),
                ),
                torch.full(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    self.package.shape.radius,
                    device=self.world.device,
                    dtype=torch.float32,
                ),
            ],
            dim=1,
        )

        for i, agent in enumerate(self.world.agents):
            agent.set_pos(
                line_pos
                + torch.tensor(
                    [
                        -(self.line_length - agent.shape.radius) / 2
                        + i
                        * (self.line_length - agent.shape.radius)
                        / (self.n_agents - 1),
                        -self.agent_radius * 2,
                    ],
                    device=self.world.device,
                    dtype=torch.float32,
                ),
                batch_index=env_index,
            )

        self.line.set_pos(
            line_pos,
            batch_index=env_index,
        )
        self.package.goal.set_pos(
            goal_pos,
            batch_index=env_index,
        )
        self.line.set_rot(
            torch.zeros(1, device=self.world.device, dtype=torch.float32),
            batch_index=env_index,
        )
        self.package.set_pos(
            line_pos + package_rel_pos,
            batch_index=env_index,
        )

        self.floor.set_pos(
            torch.tensor(
                [
                    0,
                    -self.world.y_semidim
                    - self.floor.shape.width / 2
                    - self.agent_radius,
                ],
                device=self.world.device,
            ),
            batch_index=env_index,
        )
        self.compute_on_the_ground()
        if env_index is None:
            self.global_shaping = (
                torch.linalg.vector_norm(
                    self.package.state.pos - self.package.goal.state.pos, dim=1
                )
                * self.shaping_factor
            )
        else:
            self.global_shaping[env_index] = (
                torch.linalg.vector_norm(
                    self.package.state.pos[env_index]
                    - self.package.goal.state.pos[env_index]
                )
                * self.shaping_factor
            )

    def compute_on_the_ground(self):
        self.on_the_ground = self.world.is_overlapping(
            self.line, self.floor
        ) + self.world.is_overlapping(self.package, self.floor)

    def reward(self, agent: Agent):
        """
        计算智能体的奖励
        
        场景目标: 多个智能体协作在平衡杆上运送包裹到目标位置
        奖励机制:
        1. 位置奖励: 包裹接近目标获得正奖励（shaping reward）
        2. 掉落惩罚: 包裹掉到地上获得负奖励
        
        参数:
            agent (Agent): 要计算奖励的智能体
            
        返回:
            torch.Tensor: 奖励值，形状 (batch_dim,)
        """
        # 检查是否是第一个智能体（避免重复计算全局奖励）
        is_first = agent == self.world.agents[0]

        if is_first:
            # ========== 初始化奖励张量 ==========
            self.pos_rew[:] = 0      # 位置奖励
            self.ground_rew[:] = 0   # 掉落惩罚

            # ========== 检查包裹是否掉到地上 ==========
            # 计算包裹是否接触地面或在平衡杆下方
            self.compute_on_the_ground()
            
            # ========== 计算包裹到目标的距离 ==========
            # 欧氏距离，形状: (batch_dim,)
            self.package_dist = torch.linalg.vector_norm(
                self.package.state.pos - self.package.goal.state.pos, dim=1
            )

            # ========== 计算掉落惩罚 ==========
            # 如果包裹在地上，施加掉落惩罚（通常是负值）
            self.ground_rew[self.on_the_ground] = self.fall_reward

            # ========== 计算位置shaping奖励 ==========
            # 当前的shaping值 = 距离 * 缩放因子
            global_shaping = self.package_dist * self.shaping_factor
            # Shaping reward = 上一步的shaping - 当前shaping
            # 如果距离减小（包裹靠近目标），reward为正
            self.pos_rew = self.global_shaping - global_shaping
            # 更新global_shaping为当前值（用于下一步计算）
            self.global_shaping = global_shaping

        # ========== 返回总奖励 ==========
        # 总奖励 = 掉落惩罚 + 位置奖励
        return self.ground_rew + self.pos_rew

    def observation(self, agent: Agent):
        """
        计算智能体的观测
        
        观测内容:
        1. 智能体自身状态 (4维):
           - 位置 (2维)
           - 速度 (2维)
        2. 相对位置信息 (4维):
           - 智能体到包裹的相对位置 (2维)
           - 智能体到平衡杆中心的相对位置 (2维)
        3. 任务目标信息 (2维):
           - 包裹到目标的相对位置 (2维)
        4. 包裹和平衡杆的运动状态 (6维):
           - 包裹速度 (2维)
           - 平衡杆线速度 (2维)
           - 平衡杆角速度 (1维)
           - 平衡杆旋转角度 (1维，归一化到[0, π])
        
        参数:
            agent (Agent): 要计算观测的智能体
            
        返回:
            torch.Tensor: 观测向量，形状 (batch_dim, 16)
        """
        # ========== 拼接所有观测信息 ==========
        # 获取智能体参考坐标系下所有实体的位置
        return torch.cat(
            [
                agent.state.pos,                                      # 智能体位置 (batch_dim, 2)
                agent.state.vel,                                      # 智能体速度 (batch_dim, 2)
                agent.state.pos - self.package.state.pos,             # 到包裹的相对位置 (batch_dim, 2)
                agent.state.pos - self.line.state.pos,                # 到平衡杆中心的相对位置 (batch_dim, 2)
                self.package.state.pos - self.package.goal.state.pos, # 包裹到目标的相对位置 (batch_dim, 2)
                self.package.state.vel,                               # 包裹速度 (batch_dim, 2)
                self.line.state.vel,                                  # 平衡杆线速度 (batch_dim, 2)
                self.line.state.ang_vel,                              # 平衡杆角速度 (batch_dim, 1)
                self.line.state.rot % torch.pi,                       # 平衡杆旋转角度，归一化到[0, π] (batch_dim, 1)
            ],
            dim=-1,  # 沿最后一维拼接
        )
        # 最终形状: (batch_dim, 16)

    def done(self):
        return self.on_the_ground + self.world.is_overlapping(
            self.package, self.package.goal
        )

    def info(self, agent: Agent):
        info = {"pos_rew": self.pos_rew, "ground_rew": self.ground_rew}
        return info


class HeuristicPolicy(BaseHeuristicPolicy):
    def compute_action(self, observation: torch.Tensor, u_range: float) -> torch.Tensor:
        batch_dim = observation.shape[0]

        index_package_goal_pos = 8
        dist_package_goal = observation[
            :, index_package_goal_pos : index_package_goal_pos + 2
        ]
        y_distance_ge_0 = dist_package_goal[:, Y] >= 0

        if self.continuous_actions:
            action_agent = torch.clamp(
                torch.stack(
                    [
                        torch.zeros(batch_dim, device=observation.device),
                        -dist_package_goal[:, Y],
                    ],
                    dim=1,
                ),
                min=-u_range,
                max=u_range,
            )
            action_agent[:, Y][y_distance_ge_0] = 0
        else:
            action_agent = torch.full((batch_dim,), 4, device=observation.device)
            action_agent[y_distance_ge_0] = 0
        return action_agent


if __name__ == "__main__":
    render_interactively(
        __file__,
        n_agents=3,
        package_mass=5,
        random_package_pos_on_line=True,
        control_two_agents=True,
    )
